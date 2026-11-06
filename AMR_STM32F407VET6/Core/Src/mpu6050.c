#include "mpu6050.h"

/**
 * @brief Initialize MPU6050 sensor with DATA_READY interrupt enabled
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;
    uint8_t check;
    uint8_t data;

    // Step 1: Verify device is present (WHO_AM_I should return expected value)
    // AMANDEMEN: Timeout 100ms
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I,
                              1, &check, 1, 100);

    if (status != HAL_OK || check != MPU6050_WHO_AM_I_VAL) {
        return HAL_ERROR;  // Device not found or wrong device
    }

    // Step 2: Wake up MPU6050 (exit sleep mode)
    // AMANDEMEN: Timeout 100ms
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                               1, &data, 1, 100);
    if (status != HAL_OK) return status;

    HAL_Delay(200);  // Wait for sensor to stabilize

    // Step 3: Set sample rate divider
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // Setting SMPLRT_DIV = 9 gives us 100Hz sample rate
    data = 0x09;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV,
                               1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // Step 4: Configure Digital Low Pass Filter (DLPF)
    data = 0x03;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_CONFIG,
                               1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // Step 5: Configure Gyroscope range (±250°/s)
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG,
                               1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // Step 6: Configure Accelerometer range (±2g)
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG,
                               1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // Step 7: Configure Interrupt Pin (INT pin behavior)
    data = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_INT_PIN_CFG,
                               1, &data, 1, 100);
    if (status != HAL_OK) return status;

    // Step 8: Enable DATA_READY interrupt
    data = MPU6050_INT_DATA_RDY_EN;
    status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_INT_ENABLE,
                               1, &data, 1, 100);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief Read all sensor data from MPU6050
 */
HAL_StatusTypeDef MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t raw_data[14];
    HAL_StatusTypeDef status;

    // AMANDEMEN ABSOLUT: Pembacaan dinamis di dalam RTOS TIDAK BOLEH melebihi 10ms.
    // Jika bus I2C macet, fungsi ini akan langsung menyerah dan membiarkan RTOS terus berjalan.
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                              1, raw_data, 14, 10);

    if (status != HAL_OK) {
        return status;
    }

    // Parse raw data (big-endian format: MSB first, then LSB)
    data->accel_x_raw = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    data->accel_y_raw = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    data->accel_z_raw = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    data->temp_raw = (int16_t)(raw_data[6] << 8 | raw_data[7]);

    data->gyro_x_raw = (int16_t)(raw_data[8] << 8 | raw_data[9]);
    data->gyro_y_raw = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    data->gyro_z_raw = (int16_t)(raw_data[12] << 8 | raw_data[13]);

    // Process raw values into physical units
    MPU6050_ProcessData(data);

    return HAL_OK;
}

/**
 * @brief Convert raw sensor values to physical units with axis corrections
 */
void MPU6050_ProcessData(MPU6050_Data *data) {
    // Convert accelerometer from LSB to m/s²
    // Axes inverted to match ROS REP-103 based on current mounting
    data->accel_x = -(data->accel_x_raw / ACCEL_SCALE_FACTOR) * GRAVITY;
    data->accel_y = -(data->accel_y_raw / ACCEL_SCALE_FACTOR) * GRAVITY;
    data->accel_z = (data->accel_z_raw / ACCEL_SCALE_FACTOR) * GRAVITY;

    // Convert gyroscope from LSB to rad/s
    // All axes correct as-is for current mounting
    data->gyro_x = (data->gyro_x_raw / GYRO_SCALE_FACTOR) * GYRO_TO_RAD;
    data->gyro_y = (data->gyro_y_raw / GYRO_SCALE_FACTOR) * GYRO_TO_RAD;
    data->gyro_z = (data->gyro_z_raw / GYRO_SCALE_FACTOR) * GYRO_TO_RAD;

    // Convert temperature to Celsius
    data->temperature = (data->temp_raw / 340.0f) + 36.53f;
}

/**
 * @brief Read and clear interrupt status register
 */
uint8_t MPU6050_ReadIntStatus(I2C_HandleTypeDef *hi2c) {
    uint8_t int_status = 0;

    // AMANDEMEN: Timeout 10ms
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_INT_STATUS,
                     1, &int_status, 1, 10);

    return int_status;  // Bit 0 = DATA_RDY_INT
}
