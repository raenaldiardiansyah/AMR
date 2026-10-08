/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications - ROBUST POLLING MODE
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "tim.h"
#include "i2c.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "stm32f4xx_hal_iwdg.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// UART RX Buffer for receiving velocity commands
#define CMD_BUFFER_SIZE 64
uint8_t uart_rx_buffer[CMD_BUFFER_SIZE];
uint8_t uart_rx_index = 0;

// Safety timeout: Stop motors if no command received
#define CMD_TIMEOUT_MS 2000

// Jarak antar roda kiri-kanan (center ban ke center ban), mm.
// HARUS SAMA dengan wheel_base di odometry_node.py (meter x 1000).
// Lama: 292 (STM32) / 299 (odometry_node). Baru: 456.
#define WHEELBASE_MM 456

// MPU6050 IMU data structure
MPU6050_Data imu_data;
uint8_t imu_initialized = 0;  // Flag: 0 = not initialized, 1 = OK

// IMU statistics (for monitoring)
uint32_t imu_read_count = 0;
uint32_t imu_error_count = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // 1. Start the Encoders
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // 2. Initialize Motor Control System
  Motor_Init();

  // 3. IMU Initialization with proper I2C full reset between retries
    // 2026-10-08: IWDG timeout ~2 s, sedangkan 5x retry IMU ~2,4 s. Tanpa refresh di
    // sini STM32 reset terus kalau MPU6050 tidak terdeteksi (tidak pernah masuk loop utama).
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(500);

    for (int attempt = 0; attempt < 5; attempt++) {
        HAL_IWDG_Refresh(&hiwdg);

        // Full peripheral reset before each attempt.
        // This is much stronger than __HAL_I2C_DISABLE/__HAL_I2C_ENABLE.
        // DeInit releases the peripheral completely, then MX_I2C1_Init
        // re-applies all clock and GPIO settings from scratch.
        HAL_I2C_DeInit(&hi2c1);
        osDelay(20);
        MX_I2C1_Init();
        osDelay(50);

        HAL_StatusTypeDef init_result = MPU6050_Init(&hi2c1);

        if (init_result == HAL_OK) {
            imu_initialized = 1;
            char init_msg[] = "[IMU:OK]\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)init_msg, strlen(init_msg), 100);
            break;
        }

        char fail_msg[32];
        int flen = snprintf(fail_msg, sizeof(fail_msg), "[IMU:RETRY:%d]\r\n", attempt + 1);
        HAL_UART_Transmit(&huart1, (uint8_t*)fail_msg, flen, 100);
        osDelay(300);
    }

    if (!imu_initialized) {
        char init_msg[] = "[IMU:FAIL:CHECK_WIRING]\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)init_msg, strlen(init_msg), 100);
    }

  // 4. Start UART RX Interrupt untuk menerima perintah motor dari Raspberry
  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);

  char tx_buffer[128];
  int16_t left_ticks = 0;
  int16_t right_ticks = 0;

  static int16_t prev_left_ticks  = 0;
  static int16_t prev_right_ticks = 0;
  uint32_t last_telemetry_tick = HAL_GetTick();

  // LOOP UTAMA - BERJALAN TEPAT DI 100Hz (Setiap 10ms)
  for(;;)
  {
	  // A. BACA ENCODER RODA SECARA KONTINU + HITUNG DELTA + JALANKAN PID
	  left_ticks  = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
	  right_ticks = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

	  // Compute delta with overflow handling
	  int32_t delta_left  = (int32_t)left_ticks  - (int32_t)prev_left_ticks;
	  int32_t delta_right = (int32_t)right_ticks - (int32_t)prev_right_ticks;
	  if (delta_left  >  32767) delta_left  -= 65536;
	  if (delta_left  < -32768) delta_left  += 65536;
	  if (delta_right >  32767) delta_right -= 65536;
	  if (delta_right < -32768) delta_right += 65536;

	  // Accumulate delta for PID to consume
	  g_encoder_left_delta  += delta_left;
	  g_encoder_right_delta += delta_right;

	  // Store current as previous for next cycle
	  prev_left_ticks  = left_ticks;
	  prev_right_ticks = right_ticks;

	  // Run PID velocity controller (reads g_encoder deltas and outputs PWM)
	  Motor_PID_Update();

      // B. BACA IMU SECARA PAKSA (POLLING) - TANPA PEDULI INTERUPSI
      if (imu_initialized) {
          // Hanya satu transaksi I2C tunggal untuk mencegah kemacetan bus
          if (MPU6050_ReadData(&hi2c1, &imu_data) == HAL_OK) {
              imu_read_count++;
          } else {
				imu_error_count++;
				if (imu_error_count > 5) {
					// Full re-init, same as startup recovery
					HAL_I2C_DeInit(&hi2c1);
					osDelay(5);
					MX_I2C1_Init();
					osDelay(10);
					if (MPU6050_Init(&hi2c1) == HAL_OK) {
						imu_error_count = 0;
						char msg[] = "[IMU:RECOVERED]\r\n";
						HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
					} else {
						imu_initialized = 0;
						imu_error_count = 0;
					}
				}
			}
      }

      // B2. PERIODIC RE-INIT if IMU was lost at runtime
            if (!imu_initialized) {
                static uint32_t reinit_counter = 0;
                reinit_counter++;
                if (reinit_counter >= 500) {  // Every 5 seconds (500 x 10ms)
                    reinit_counter = 0;
                    HAL_I2C_DeInit(&hi2c1);
                    osDelay(5);
                    MX_I2C1_Init();
                    osDelay(50);
                    if (MPU6050_Init(&hi2c1) == HAL_OK) {
                        imu_initialized = 1;
                        imu_error_count = 0;
                        char msg[] = "[IMU:RECOVERED]\r\n";
                        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
                    }
                }
            }

      // C. TELEMETRI KE RASPBERRY PI (Dikirim setiap 5 siklus = 20Hz)
		uint32_t now_tick = HAL_GetTick();
		uint32_t elapsed_ms = now_tick - last_telemetry_tick;

		if (elapsed_ms >= 50) {
			last_telemetry_tick = now_tick;
			int len;

			if (imu_initialized) {
				// Konversi ke bilangan bulat agar komunikasi Serial lebih ringan
				int16_t gz_mrad = (int16_t)(imu_data.gyro_z * 1000);
				int16_t ax_cm = (int16_t)(imu_data.accel_x * 100);
				int16_t ay_cm = (int16_t)(imu_data.accel_y * 100);
				int16_t az_cm = (int16_t)(imu_data.accel_z * 100);

				len = sprintf(tx_buffer,
					"{\"l\":%d,\"r\":%d,\"gz\":%d,\"ax\":%d,\"ay\":%d,\"az\":%d,\"rd\":%lu,\"err\":%lu,\"dt\":%lu}\r\n",
					left_ticks,
					right_ticks,
					gz_mrad,
					ax_cm,
					ay_cm,
					az_cm,
					imu_read_count, // Indikator kesuksesan baca
					imu_error_count, // Indikator kemacetan
					(unsigned long)elapsed_ms  // NEW: actual ms since last telemetry
				);
			} else {
				len = sprintf(tx_buffer, "{\"l\":%d,\"r\":%d,\"dt\":%lu}\r\n",
					left_ticks, right_ticks, (unsigned long)elapsed_ms);
			}

			if (len > 0) {
				HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 10);
			}
		}

      // D. SAFETY TIMEOUT MOTOR
      if (Motor_CheckTimeout(HAL_GetTick(), CMD_TIMEOUT_MS)) {
          Motor_Emergency_Stop();
      }

      //E. WATCHDOG FEED
      HAL_IWDG_Refresh(&hiwdg);

      // JEDA SIKLUS MUTLAK 10 MILIDETIK (100Hz)
      osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint8_t received_byte = uart_rx_buffer[uart_rx_index];

        if (received_byte == '\n' || received_byte == '\r') {
            if (uart_rx_index == 0) {
                HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
                return;
            }

            uart_rx_buffer[uart_rx_index] = '\0';

            int16_t v_cmd = 0;
            int16_t w_cmd = 0;

            if (sscanf((char*)uart_rx_buffer, "V:%hd,W:%hd", &v_cmd, &w_cmd) == 2) {

                // Correct differential drive kinematics
                // v_cmd in mm/s, w_cmd in mrad/s, wheelbase = WHEELBASE_MM
                // angular_contribution = w_rad * wheelbase/2 = (w_cmd/1000) * (WHEELBASE_MM/2)
                int16_t angular_contribution = (int16_t)(((int32_t)w_cmd * (WHEELBASE_MM / 2)) / 1000);

                int16_t left_speed  = v_cmd - angular_contribution;
                int16_t right_speed = v_cmd + angular_contribution;

                // MINIMUM SPEED GUARANTEE FOR PURE ROTATION
                // When v_cmd is zero (pure rotation), if the angular contribution
                // is too small to overcome motor deadband, boost it to minimum
                // moving speed while preserving direction sign.
                // Left deadband ends at ~47 mm/s, right at ~65 mm/s.
                // We use 80 mm/s as safe minimum for both wheels.
                #define MIN_ROTATION_SPEED_MMPS 80

                if (v_cmd == 0 && w_cmd != 0) {
                    if (left_speed > 0 && left_speed < MIN_ROTATION_SPEED_MMPS)
                        left_speed = MIN_ROTATION_SPEED_MMPS;
                    else if (left_speed < 0 && left_speed > -MIN_ROTATION_SPEED_MMPS)
                        left_speed = -MIN_ROTATION_SPEED_MMPS;

                    if (right_speed > 0 && right_speed < MIN_ROTATION_SPEED_MMPS)
                        right_speed = MIN_ROTATION_SPEED_MMPS;
                    else if (right_speed < 0 && right_speed > -MIN_ROTATION_SPEED_MMPS)
                        right_speed = -MIN_ROTATION_SPEED_MMPS;
                }

                Motor_SetTargetSpeed(left_speed, right_speed);
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
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Tidak melakukan apa-apa.
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // Matikan motor segera supaya robot tidak lari
    Motor_Emergency_Stop();

    // Kirim pesan debug ke Pi supaya kamu tahu task mana yang overflow
    char msg[80];
    int len = snprintf(msg, sizeof(msg),
        "[FATAL] Stack overflow in task: %s\r\n", pcTaskName);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    // Tunggu sebentar supaya pesan terkirim,
    // lalu biarkan IWDG reset STM32 otomatis dalam 8 detik
    HAL_Delay(500);
    while(1) {}
}

void vApplicationMallocFailedHook(void)
{
    Motor_Emergency_Stop();

    char msg[] = "[FATAL] FreeRTOS heap exhausted\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

    HAL_Delay(500);
    while(1) {}
}
/* USER CODE END Application */

