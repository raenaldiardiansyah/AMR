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
// Ultrasonic Declaration
float us_distance_cm = 0.0f;

// Saklar sensor ultrasonik: 0 = OFF (sensor tidak terpasang), 1 = ON
// Saat OFF, pembacaan noise di pin Echo tidak akan memblokir/menghentikan motor.
#define USE_ULTRASONIC 0
extern TIM_HandleTypeDef htim7;

// UART RX Buffer for receiving velocity commands
#define CMD_BUFFER_SIZE 64
uint8_t uart_rx_buffer[CMD_BUFFER_SIZE];
uint8_t uart_rx_index = 0;

// Safety timeout: Stop motors if no command received
#define CMD_TIMEOUT_MS 2000

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

  // 3. IMU Initialization with Aggressive Retries
    osDelay(500);

    char debug_msg[80];
    sprintf(debug_msg, "[DEBUG] Starting MPU6050 init...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    // Langsung masuk ke loop inisialisasi dengan mekanisme retries
    for (int attempt = 0; attempt < 5; attempt++) {
        HAL_StatusTypeDef init_result = MPU6050_Init(&hi2c1);

        if (init_result == HAL_OK) {
            imu_initialized = 1;
            char init_msg[] = "[IMU:OK:POLLING_MODE]\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)init_msg, strlen(init_msg), 100);
            break;
        }
        // Jika I2C macet saat inisialisasi, paksa reset hardware I2C
        __HAL_I2C_DISABLE(&hi2c1);
        osDelay(10);
        __HAL_I2C_ENABLE(&hi2c1);

        osDelay(200);
    }

    if (!imu_initialized) {
        char init_msg[] = "[IMU:FAIL]\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)init_msg, strlen(init_msg), 100);
    }

  // 4. Start UART RX Interrupt untuk menerima perintah motor dari Raspberry
  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);

  char tx_buffer[128];
  int16_t left_ticks = 0;
  int16_t right_ticks = 0;

  // LOOP UTAMA - BERJALAN TEPAT DI 100Hz (Setiap 10ms)
  for(;;)
  {
      // A. BACA ENCODER RODA SECARA KONTINU
      left_ticks = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
      right_ticks = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

      // B. BACA IMU SECARA PAKSA (POLLING) - TANPA PEDULI INTERUPSI
      if (imu_initialized) {
          // Hanya satu transaksi I2C tunggal untuk mencegah kemacetan bus
          if (MPU6050_ReadData(&hi2c1, &imu_data) == HAL_OK) {
              imu_read_count++;
          } else {
              imu_error_count++;
              // Mekanisme Pemulihan Diri: Jika macet, reset I2C secara brutal
              if (imu_error_count > 10) {
                  __HAL_I2C_DISABLE(&hi2c1);
                  osDelay(2);
                  __HAL_I2C_ENABLE(&hi2c1);
                  imu_error_count = 0;
              }
          }
      }

      // C. TELEMETRI KE RASPBERRY PI (Dikirim setiap 5 siklus = 20Hz)
      static uint8_t telemetry_counter = 0;
      telemetry_counter++;

      if (telemetry_counter >= 5) {
          telemetry_counter = 0;
          int len;

          if (imu_initialized) {
              // Konversi ke bilangan bulat agar komunikasi Serial lebih ringan
              int16_t gz_mrad = (int16_t)(imu_data.gyro_z * 1000);
              int16_t ax_cm = (int16_t)(imu_data.accel_x * 100);
              int16_t ay_cm = (int16_t)(imu_data.accel_y * 100);
              int16_t az_cm = (int16_t)(imu_data.accel_z * 100);

              // Perhatikan penambahan \"us\":%.1f di akhir format JSON
				len = sprintf(tx_buffer,
					"{\"l\":%d,\"r\":%d,\"gz\":%d,\"ax\":%d,\"ay\":%d,\"az\":%d,\"rd\":%lu,\"err\":%lu,\"us\":%.1f}\r\n",
					left_ticks,
					right_ticks,
					gz_mrad,
					ax_cm,
					ay_cm,
					az_cm,
					imu_read_count,
					imu_error_count,
					us_distance_cm
				);
          } else {
              len = sprintf(tx_buffer, "{\"l\":%d,\"r\":%d,\"us\":%.1f}\r\n", left_ticks, right_ticks, us_distance_cm);
          }

          if (len > 0) {
              HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 10);
          }
      }

      // E. PENEMBAKAN SENSOR ULTRASONIK (Berjalan pada 20Hz / setiap 50ms)
		static uint8_t us_trigger_counter = 0;
		us_trigger_counter++;

		if (us_trigger_counter >= 5) {
			us_trigger_counter = 0;

			// Kirim pulsa HIGH selama ~10 mikrodetik ke pin Trigger
			HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_SET);

			// Penundaan kotor (delay) ~10us untuk arsitektur 168MHz
			// Tidak perlu menggunakan timer untuk trigger sederhana ini
			for(volatile int i=0; i<300; i++) {}

			HAL_GPIO_WritePin(US_Trig_GPIO_Port, US_Trig_Pin, GPIO_PIN_RESET);
		}

      if (Motor_CheckTimeout(HAL_GetTick(), CMD_TIMEOUT_MS)) {
          Motor_Emergency_Stop();
      }

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

				// --- FILTER KEAMANAN (DIRECTIONAL FAILSAFE) ---
				// Jika rintangan terdeteksi di bawah 30cm dan di atas batas noise 5cm
				if (USE_ULTRASONIC && us_distance_cm > 5.0f && us_distance_cm < 30.0f) {
					// Jika perintah dari ROS 2 adalah MAJU (v_cmd positif)
					if (v_cmd > 0) {
						v_cmd = 0; // Tolak dan netralkan kecepatan maju
					}
					// Catatan: Jika v_cmd negatif (mundur) atau w_cmd (rotasi),
					// biarkan lewat agar robot bisa bermanuver mundur/menghindar.
				}

				int16_t left_speed = v_cmd - (w_cmd / 2);
				int16_t right_speed = v_cmd + (w_cmd / 2);
				Motor_SetSpeed(left_speed, right_speed);
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
    if (GPIO_Pin == US_Echo_Pin) {
        if (HAL_GPIO_ReadPin(US_Echo_GPIO_Port, US_Echo_Pin) == GPIO_PIN_SET) {
            __HAL_TIM_SET_COUNTER(&htim7, 0);
            HAL_TIM_Base_Start(&htim7);
        } else {
            HAL_TIM_Base_Stop(&htim7);
            uint32_t time_us = __HAL_TIM_GET_COUNTER(&htim7);
            us_distance_cm = (time_us * 0.0343f) / 2.0f;

            if (USE_ULTRASONIC && us_distance_cm > 2.0f && us_distance_cm < 30.0f) {
                Motor_Emergency_Stop();
            }
        }
    }
}
/* USER CODE END Application */

