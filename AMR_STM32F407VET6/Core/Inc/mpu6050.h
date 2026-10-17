#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

// MPU6050 I2C Address (AD0 pin connected to GND)
#define MPU6050_ADDR        (0x68 << 1)  // Shifted for HAL I2C functions

// MPU6050 Register Addresses
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_PIN_CFG  0x37
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_INT_STATUS   0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_TEMP_OUT_H   0x41

// Expected WHO_AM_I value
#define MPU6050_WHO_AM_I_VAL 0x70

// Interrupt enable bits
#define MPU6050_INT_DATA_RDY_EN  0x01  // Data Ready interrupt enable

// Scale factors for ±2g and ±250°/s
#define ACCEL_SCALE_FACTOR  16384.0f  // LSB/g for ±2g
#define GYRO_SCALE_FACTOR   131.0f    // LSB/(°/s) for ±250°/s
#define GYRO_TO_RAD         0.01745329f  // Convert °/s to rad/s (π/180)
#define GRAVITY             9.81f     // m/s²

/**
 * @brief MPU6050 data structure
 */
typedef struct {
    // Raw sensor data (16-bit)
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    int16_t temp_raw;

    // Processed data (SI units)
    float accel_x;  // m/s²
    float accel_y;  // m/s²
    float accel_z;  // m/s²

    float gyro_x;   // rad/s
    float gyro_y;   // rad/s
    float gyro_z;   // rad/s (YAW RATE - most important for robot!)

    float temperature;  // °C
} MPU6050_Data;

/**
 * @brief Initialize MPU6050 sensor with interrupt enabled
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Read all sensor data from MPU6050
 * @param hi2c Pointer to I2C handle
 * @param data Pointer to MPU6050_Data structure to store results
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);

/**
 * @brief Process raw data into physical units
 * @param data Pointer to MPU6050_Data structure
 */
void MPU6050_ProcessData(MPU6050_Data *data);

/**
 * @brief Read and clear interrupt status register
 * @param hi2c Pointer to I2C handle
 * @retval Interrupt status byte (bit 0 = data ready)
 */
uint8_t MPU6050_ReadIntStatus(I2C_HandleTypeDef *hi2c);

#endif /* INC_MPU6050_H_ */
