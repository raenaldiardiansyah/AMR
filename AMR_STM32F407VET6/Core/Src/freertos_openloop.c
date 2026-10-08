/* ============================================================
 * freertos_openloop.c - BTS7960 MOTOR CHARACTERIZATION FIRMWARE
 * Target: Large AMR Lab Robot (STM32F407VET6 + BTS7960)
 * ============================================================
 * Command accepted: "P:LEFT_PWM,RIGHT_PWM\r\n" (0 to 999)
 * Telemetry sent (20 Hz): {"l":LEFT_TICKS,"r":RIGHT_TICKS,"lp":LEFT_PWM,"rp":RIGHT_PWM}\r\n
 * ============================================================
 * 2026-10-08: disalin dari AMR_Orange_STM32F407VET6.rar (versi robot besar),
 * menggantikan versi L298N robot kecil. Tambahan: HAL_IWDG_Refresh, karena proyek
 * ini mengaktifkan IWDG (~2 s) di main.c; tanpa refresh MCU reset terus.
 * Masih di-exclude dari build (.cproject); tukar dengan freertos.c untuk karakterisasi.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "tim.h"
#include "iwdg.h"

#define CMD_BUFFER_SIZE  64
#define CMD_TIMEOUT_MS   2000

static uint8_t uart_rx_buffer[CMD_BUFFER_SIZE];
static uint8_t uart_rx_index = 0;

static volatile uint16_t g_left_pwm  = 0;
static volatile uint16_t g_right_pwm = 0;
static volatile uint32_t g_last_cmd_time = 0;

static int32_t g_total_left_ticks  = 0;
static int32_t g_total_right_ticks = 0;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 3000 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};

void StartDefaultTask(void *argument);
void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

/* Direct 4-channel PWM control for dual BTS7960 drivers */
static void Motor_SetRawPWM_BTS7960(uint16_t left_pwm, uint16_t right_pwm)
{
    if (left_pwm  > 999) left_pwm  = 999;
    if (right_pwm > 999) right_pwm = 999;

    /* Left Wheel: CH1 (PC6) = Forward, CH2 (PC7) = Reverse */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

    /* Right Wheel: CH3 (PC8) = Forward, CH4 (PC9) = Reverse */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, right_pwm);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
}

void StartDefaultTask(void *argument)
{
    /* 1. Start Encoders */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); /* Right Wheel */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); /* Left Wheel */

    /* 2. Start all 4 PWM Channels on TIM8 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    Motor_SetRawPWM_BTS7960(0, 0);

    g_last_cmd_time = HAL_GetTick();

    char start_msg[] = "[CHAR_MODE:BTS7960_READY] Send P:LEFT,RIGHT to command PWM\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)start_msg, strlen(start_msg), 200);

    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);

    uint16_t prev_left_raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    uint16_t prev_right_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    char tx_buffer[128];
    uint8_t telemetry_counter = 0;

    /* Loop timing initialization: exact 10 ms (100 Hz) rate without drift */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);

    for (;;)
    {
        /* A. Compute 16-bit Timer Tick Deltas */
        uint16_t curr_left_raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
        uint16_t curr_right_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

        int16_t delta_left  = (int16_t)(curr_left_raw  - prev_left_raw);
        int16_t delta_right = (int16_t)(curr_right_raw - prev_right_raw);

        g_total_left_ticks  += delta_left;
        g_total_right_ticks += delta_right;

        prev_left_raw  = curr_left_raw;
        prev_right_raw = curr_right_raw;

        /* B. Safety Timeout Check */
        uint32_t now = HAL_GetTick();
        if ((now - g_last_cmd_time) > CMD_TIMEOUT_MS) {
            g_left_pwm  = 0;
            g_right_pwm = 0;
        }

        /* C. Output PWM to BTS7960 */
        Motor_SetRawPWM_BTS7960(g_left_pwm, g_right_pwm);

        /* D. Send Telemetry at 20 Hz */
        telemetry_counter++;
        if (telemetry_counter >= 5) {
            telemetry_counter = 0;

            int len = snprintf(tx_buffer, sizeof(tx_buffer),
                "{\"l\":%ld,\"r\":%ld,\"lp\":%u,\"rp\":%u}\r\n",
                (long)g_total_left_ticks,
                (long)g_total_right_ticks,
                (unsigned)g_left_pwm,
                (unsigned)g_right_pwm);

            if (len > 0 && len < (int)sizeof(tx_buffer)) {
                HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 10);
            }
        }

        /* E. Watchdog feed */
        HAL_IWDG_Refresh(&hiwdg);

        /* F. Non-drifting delay */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint8_t received_byte = uart_rx_buffer[uart_rx_index];

    if (received_byte == '\n' || received_byte == '\r') {
        if (uart_rx_index > 0) {
            uart_rx_buffer[uart_rx_index] = '\0';

            uint16_t lp = 0;
            uint16_t rp = 0;

            if (sscanf((char*)uart_rx_buffer, "P:%hu,%hu", &lp, &rp) == 2) {
                if (lp > 999) lp = 999;
                if (rp > 999) rp = 999;

                g_left_pwm      = lp;
                g_right_pwm     = rp;
                g_last_cmd_time = HAL_GetTick();
            }
        }
        uart_rx_index = 0;
    } else {
        uart_rx_index++;
        if (uart_rx_index >= (CMD_BUFFER_SIZE - 1)) {
            uart_rx_index = 0;
        }
    }

    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}
