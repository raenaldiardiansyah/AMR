#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"

// Motor command structure
typedef struct {
    int16_t left_speed;   // -1000 to +1000 (mm/s)
    int16_t right_speed;  // -1000 to +1000 (mm/s)
    uint32_t last_cmd_time; // Timestamp of last valid command
} MotorCommand_t;

// Function prototypes
void Motor_Init(void);
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);
void Motor_Emergency_Stop(void);
uint8_t Motor_CheckTimeout(uint32_t current_time, uint32_t timeout_ms);

#endif /* MOTOR_CONTROL_H */
