/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CEVA_SH2/sh2.h"
#include "CEVA_SH2/sh2_err.h"
#include "CEVA_SH2/sh2_SensorValue.h"
#include "CEVA_SH2/sh2_hal.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    BNO_STATUS_OK = 0,
    BNO_STATUS_DISCONNECTED = 1
} BNO_Status_t;

typedef struct {
    struct {
        float roll;
        float pitch;
        float yaw;
    } euler;
    struct {
        float w;
        float x;
        float y;
        float z;
    } quat;
    uint8_t status;
} BNO086_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
extern sh2_Hal_t my_sh2_hal;

// Global sensor values
volatile sh2_SensorValue_t live_sensor_data;
volatile BNO_Status_t bno_connection_status = BNO_STATUS_DISCONNECTED;
volatile uint32_t last_bno_rx_tick = 0;
volatile uint32_t last_reconnect_attempt_tick = 0;

// Speed & packet latency tracking
volatile uint32_t bno_last_packet_time_us = 0;
volatile uint32_t bno_packet_dt_us = 0;
volatile float    bno_actual_hz = 0.0f;

// Robot orientation & acceleration
volatile float robot_qx = 0.0f, robot_qy = 0.0f, robot_qz = 0.0f, robot_qw = 1.0f;
volatile float robot_accel_x = 0.0f, robot_accel_y = 0.0f, robot_accel_z = 0.0f;

float robot_roll_deg  = 0.0f;
float robot_pitch_deg = 0.0f;
float robot_yaw_deg   = 0.0f;

struct {
    float roll;
    float pitch;
    float yaw;
} euler;

BNO086_Data_t bno_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent);
void sensorHandler(void *cookie, sh2_SensorEvent_t *event);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        // Sensor underwent reset
    }
}

void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) return;

    // Pelacakan latency & refresh rate (Hz)
    uint32_t current_time_us = (HAL_GetTick() * 1000U);
    bno_packet_dt_us = current_time_us - bno_last_packet_time_us;
    bno_last_packet_time_us = current_time_us;

    if (bno_packet_dt_us > 0) {
        bno_actual_hz = 1000000.0f / (float)bno_packet_dt_us;
    }

    last_bno_rx_tick = HAL_GetTick();
    bno_connection_status = BNO_STATUS_OK;
    live_sensor_data = value;

    switch (value.sensorId) {
    case SH2_ROTATION_VECTOR:
    {
        float r = value.un.rotationVector.real; // qw
        float i = value.un.rotationVector.i;    // qx
        float j = value.un.rotationVector.j;    // qy
        float k = value.un.rotationVector.k;    // qz

        robot_qw = r;
        robot_qx = i;
        robot_qy = j;
        robot_qz = k;

        bno_data.status = value.status;
        bno_data.quat.w = r;
        bno_data.quat.x = i;
        bno_data.quat.y = j;
        bno_data.quat.z = k;

        // 1. Roll (X-axis) [-180 s/d +180 deg]
        float sinr_cosp = 2.0f * (r * i + j * k);
        float cosr_cosp = 1.0f - 2.0f * (i * i + j * j);
        euler.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
        robot_roll_deg = euler.roll;
        bno_data.euler.roll = euler.roll;

        // 2. Pitch (Y-axis) [-90 s/d +90 deg]
        float sinp = 2.0f * (r * j - k * i);
        if (fabsf(sinp) >= 1.0f) {
            euler.pitch = copysignf(90.0f, sinp);
        } else {
            euler.pitch = asinf(sinp) * RAD_TO_DEG;
        }
        robot_pitch_deg = euler.pitch;
        bno_data.euler.pitch = euler.pitch;

        // 3. Yaw (Z-axis) [-180 s/d +180 deg]
        float siny_cosp = 2.0f * (r * k + i * j);
        float cosy_cosp = 1.0f - 2.0f * (j * j + k * k);
        euler.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

        // Normalisasi Yaw agar tetap dalam rentang [-180, +180] deg
        if (euler.yaw > 180.0f)  euler.yaw -= 360.0f;
        if (euler.yaw < -180.0f) euler.yaw += 360.0f;
        robot_yaw_deg = euler.yaw;
        bno_data.euler.yaw = euler.yaw;

        break;
    }
    case SH2_LINEAR_ACCELERATION:
    {
        robot_accel_x = value.un.linearAcceleration.x;
        robot_accel_y = value.un.linearAcceleration.y;
        robot_accel_z = value.un.linearAcceleration.z;
        break;
    }
    default:
        break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
    // 1. Inisialisasi awal BNO086 via HAL SPI
    if (sh2_open(&my_sh2_hal, eventHandler, NULL) == SH2_OK) {
        bno_connection_status = BNO_STATUS_OK;
        last_bno_rx_tick = HAL_GetTick();

        sh2_setSensorCallback(sensorHandler, NULL);

        // 2. Aktifkan laporan rotasi (10ms = 10000 us / 100 Hz)
        sh2_SensorConfig_t config;
        config.changeSensitivityEnabled = false;
        config.wakeupEnabled = false;
        config.changeSensitivityRelative = false;
        config.alwaysOnEnabled = false;
        config.changeSensitivity = 0;
        config.reportInterval_us = 10000; // 100 Hz
        config.batchInterval_us = 0;

        sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);

        HAL_Delay(10);
        // Aktifkan juga Linear Acceleration
        sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);
    } else {
        bno_connection_status = BNO_STATUS_DISCONNECTED;
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (bno_connection_status == BNO_STATUS_DISCONNECTED) {
            // Auto-reconnect watchdog setiap 500 ms jika koneksi terputus
            if ((HAL_GetTick() - last_reconnect_attempt_tick) > 500) {
                last_reconnect_attempt_tick = HAL_GetTick();

                // 1. Posisikan pin kontrol ke status idle
                HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BNO_WAK_GPIO_Port, BNO_WAK_Pin, GPIO_PIN_SET);

                // 2. Tutup session SHTP & reset hardware
                sh2_close();
                HAL_Delay(10);

                // 3. Reset periferal SPI Mikrokontroler
                HAL_SPI_Abort(&hspi1);
                HAL_SPI_DeInit(&hspi1);
                MX_SPI1_Init();

                // 4. Lepaskan reset BNO086
                HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);

                // 5. Tunggu BNO086 menyelesaikan internal boot sequence
                HAL_Delay(150);

                // 6. Bersihkan phantom interrupt
                __HAL_GPIO_EXTI_CLEAR_IT(BNO_INT_Pin);
                bno_int_triggered = 0;

                // 7. Cek level fisik pin INT
                if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_RESET) {
                    bno_int_triggered = 1;
                }

                // 8. Buka kembali sesi SH2
                if (sh2_open(&my_sh2_hal, eventHandler, NULL) == SH2_OK) {
                    sh2_setSensorCallback(sensorHandler, NULL);

                    sh2_SensorConfig_t config;
                    config.changeSensitivityEnabled = false;
                    config.wakeupEnabled = false;
                    config.changeSensitivityRelative = false;
                    config.alwaysOnEnabled = false;
                    config.changeSensitivity = 0;
                    config.reportInterval_us = 10000; // 100 Hz
                    config.batchInterval_us = 0;

                    if (sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config) == SH2_OK) {
                        bno_connection_status = BNO_STATUS_OK;
                        last_bno_rx_tick = HAL_GetTick();
                    }

                    HAL_Delay(10);
                    sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);
                }
            }
        } else {
            // Jalankan polling SHTP untuk memproses pesan sensor
            sh2_service();

            // Watchdog: jika tidak ada data diterima > 300 ms, tandai terputus
            if ((HAL_GetTick() - last_bno_rx_tick) > 300) {
                bno_connection_status = BNO_STATUS_DISCONNECTED;
            }
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BNO_WAK_Pin|BNO_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BNO_CS_Pin */
  GPIO_InitStruct.Pin = BNO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BNO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BNO_WAK_Pin BNO_RST_Pin */
  GPIO_InitStruct.Pin = BNO_WAK_Pin|BNO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BNO_INT_Pin */
  GPIO_InitStruct.Pin = BNO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
