#include "motor_control.h"
#include <stdlib.h>

// Global motor command structure
MotorCommand_t g_motor_cmd = {0, 0, 0};

// Maximum PWM value (sesuaikan dengan nilai Period/ARR pada TIM8 Anda)
#define PWM_MAX 999
#define SPEED_MAX 1000

extern TIM_HandleTypeDef htim8;

void Motor_Init(void) {
    // Mulai 4 Saluran PWM pada TIM8 untuk 2 driver BTS7960
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // Kiri Maju
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // Kiri Mundur
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // Kanan Maju
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // Kanan Mundur

    // Pastikan semua motor berhenti saat inisialisasi
    Motor_SetSpeed(0, 0);
    g_motor_cmd.last_cmd_time = HAL_GetTick();
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed) {
    // Batasi nilai kecepatan
    if (left_speed > SPEED_MAX) left_speed = SPEED_MAX;
    if (left_speed < -SPEED_MAX) left_speed = -SPEED_MAX;
    if (right_speed > SPEED_MAX) right_speed = SPEED_MAX;
    if (right_speed < -SPEED_MAX) right_speed = -SPEED_MAX;

    // Kalkulasi nilai CCR (Duty Cycle)
    uint16_t left_pwm = (abs(left_speed) * PWM_MAX) / SPEED_MAX;
    uint16_t right_pwm = (abs(right_speed) * PWM_MAX) / SPEED_MAX;

    // === KENDALI MOTOR KIRI (TIM8 CH1 & CH2) ===
    if (left_speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    } else if (left_speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, left_pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    }

    // === KENDALI MOTOR KANAN (TIM8 CH3 & CH4) ===
    if (right_speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
    } else if (right_speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, right_pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
    }

    // Perbarui waktu pelacakan perintah
    g_motor_cmd.left_speed = left_speed;
    g_motor_cmd.right_speed = right_speed;
    g_motor_cmd.last_cmd_time = HAL_GetTick();
}

void Motor_Emergency_Stop(void) {
    Motor_SetSpeed(0, 0);
}

uint8_t Motor_CheckTimeout(uint32_t current_time, uint32_t timeout_ms) {
    return ((current_time - g_motor_cmd.last_cmd_time) > timeout_ms);
}
