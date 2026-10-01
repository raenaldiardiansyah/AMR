#include "motor_control.h"
#include <stdlib.h>
#include <math.h>

MotorCommand_t g_motor_cmd = {0, 0, 0};
PID_State_t g_pid_left  = {0};
PID_State_t g_pid_right = {0};

volatile int32_t g_encoder_left_delta  = 0;
volatile int32_t g_encoder_right_delta = 0;

#define PWM_MIN_MOVING_LEFT   150
#define PWM_MIN_MOVING_RIGHT  200
#define WHEEL_DIAMETER_MM     150.0f  /* ban 15 cm; lama: 68 */
#define TICKS_PER_REV         2400.0f /* Autonics E40S6-600: 600 PPR x4, 1:1 dgn roda */
#define MM_PER_TICK           ((3.14159265f * WHEEL_DIAMETER_MM) / TICKS_PER_REV)
#define PID_DT                0.01f
#define SPEED_MAX             1000
#define PWM_MAX               999

void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);

    HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);

    g_pid_left.kp  = 6.75f;
    g_pid_left.ki  = 17.01f;
    g_pid_left.kd  = 0.0f;

    g_pid_right.kp = 5.75f;
    g_pid_right.ki = 19.01f;
    g_pid_right.kd = 0.0f;

    g_motor_cmd.last_cmd_time = HAL_GetTick();
}

// Called by FreeRTOS task when a new command arrives from ROS
void Motor_SetTargetSpeed(int16_t left_speed_mmps, int16_t right_speed_mmps) {
    if (left_speed_mmps  >  SPEED_MAX) left_speed_mmps  =  SPEED_MAX;
    if (left_speed_mmps  < -SPEED_MAX) left_speed_mmps  = -SPEED_MAX;
    if (right_speed_mmps >  SPEED_MAX) right_speed_mmps =  SPEED_MAX;
    if (right_speed_mmps < -SPEED_MAX) right_speed_mmps = -SPEED_MAX;

    g_pid_left.target_speed  = (float)left_speed_mmps;
    g_pid_right.target_speed = (float)right_speed_mmps;

    // Reset integral when direction changes to prevent windup
    if ((left_speed_mmps  >= 0) != (g_motor_cmd.left_speed  >= 0)) g_pid_left.integral  = 0;
    if ((right_speed_mmps >= 0) != (g_motor_cmd.right_speed >= 0)) g_pid_right.integral = 0;

    g_motor_cmd.left_speed  = left_speed_mmps;
    g_motor_cmd.right_speed = right_speed_mmps;
    g_motor_cmd.last_cmd_time = HAL_GetTick();
}

void Motor_PID_Update(void) {

    // ================================================================
    // STEP 1: READ ACTUAL SPEED FROM ENCODER DELTAS
    // ================================================================
    int32_t left_delta  = g_encoder_left_delta;
    int32_t right_delta = g_encoder_right_delta;
    g_encoder_left_delta  = 0;
    g_encoder_right_delta = 0;

    // Convert ticks per 10ms to mm/s
    // actual speed sign follows encoder count direction
    float left_actual  =  (left_delta  * MM_PER_TICK) / PID_DT;
    float right_actual =  (right_delta * MM_PER_TICK) / PID_DT;

    // ================================================================
    // STEP 2: PID WORKS ON MAGNITUDE (ABSOLUTE SPEED)
    // Direction is handled separately by target sign, just like
    // your original Motor_SetSpeed that worked perfectly
    // ================================================================

    // --- LEFT MOTOR PID ---
    float left_target_magnitude  = fabsf(g_pid_left.target_speed);
    float left_actual_magnitude  = fabsf(left_actual);

    g_pid_left.error     = left_target_magnitude - left_actual_magnitude;
    g_pid_left.integral += g_pid_left.error * PID_DT;
    if (g_pid_left.integral >  2000.0f) g_pid_left.integral =  2000.0f;
    if (g_pid_left.integral < -2000.0f) g_pid_left.integral = -2000.0f;

    float left_derivative   = (g_pid_left.error - g_pid_left.prev_error) / PID_DT;
    float left_output_mmps  = (g_pid_left.kp * g_pid_left.error)
                            + (g_pid_left.ki * g_pid_left.integral)
                            + (g_pid_left.kd * left_derivative);
    g_pid_left.prev_error = g_pid_left.error;

    // Clamp output magnitude
    if (left_output_mmps  < 0) left_output_mmps  = 0;
    if (left_output_mmps  > SPEED_MAX) left_output_mmps  = SPEED_MAX;

    // --- RIGHT MOTOR PID ---
    float right_target_magnitude = fabsf(g_pid_right.target_speed);
    float right_actual_magnitude = fabsf(right_actual);

    g_pid_right.error     = right_target_magnitude - right_actual_magnitude;
    g_pid_right.integral += g_pid_right.error * PID_DT;
    if (g_pid_right.integral >  2000.0f) g_pid_right.integral =  2000.0f;
    if (g_pid_right.integral < -2000.0f) g_pid_right.integral = -2000.0f;

    float right_derivative  = (g_pid_right.error - g_pid_right.prev_error) / PID_DT;
    float right_output_mmps = (g_pid_right.kp * g_pid_right.error)
                            + (g_pid_right.ki * g_pid_right.integral)
                            + (g_pid_right.kd * right_derivative);
    g_pid_right.prev_error = g_pid_right.error;

    if (right_output_mmps < 0) right_output_mmps = 0;
    if (right_output_mmps > SPEED_MAX) right_output_mmps = SPEED_MAX;

    // ================================================================
    // STEP 3: CONVERT MAGNITUDE TO PWM
    // ================================================================
    uint16_t left_pwm  = (uint16_t)((left_output_mmps  * PWM_MAX) / SPEED_MAX);
    uint16_t right_pwm = (uint16_t)((right_output_mmps * PWM_MAX) / SPEED_MAX);

    // ================================================================
    // STEP 4: STICTION COMPENSATION
    // If there is a non-zero target but PWM is too low to move motor,
    // boost it to minimum moving threshold
    // ================================================================
    if (g_pid_left.target_speed  != 0 && left_pwm  < PWM_MIN_MOVING_LEFT)
        left_pwm  = PWM_MIN_MOVING_LEFT;
    if (g_pid_right.target_speed != 0 && right_pwm < PWM_MIN_MOVING_RIGHT)
        right_pwm = PWM_MIN_MOVING_RIGHT;

    // If target is exactly zero, hard stop
    if (g_pid_left.target_speed  == 0) left_pwm  = 0;
    if (g_pid_right.target_speed == 0) right_pwm = 0;

    // ================================================================
    // STEP 5: SET DIRECTION PINS BASED ON TARGET SIGN
    // This is IDENTICAL to your original Motor_SetSpeed that worked
    // ================================================================

    // LEFT MOTOR
    if (g_pid_left.target_speed > 0) {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
    } else if (g_pid_left.target_speed < 0) {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
    }

    // RIGHT MOTOR
    if (g_pid_right.target_speed > 0) {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);
    } else if (g_pid_right.target_speed < 0) {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);
    }

    // ================================================================
    // STEP 6: APPLY PWM
    // ================================================================
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, right_pwm);
}

void Motor_Emergency_Stop(void) {
    g_pid_left.target_speed  = 0;
    g_pid_right.target_speed = 0;
    g_pid_left.integral  = 0;
    g_pid_right.integral = 0;
    Motor_SetTargetSpeed(0, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
}

uint8_t Motor_CheckTimeout(uint32_t current_time, uint32_t timeout_ms) {
    return ((current_time - g_motor_cmd.last_cmd_time) > timeout_ms);
}
