/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "lsm6dsr_reg.h" // LSM6DSR driver header file
#include "MadgwickAHRS.h" // Madgwick AHRS algorithm header file
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEG2RAD 0.017453292519943295f // Pi / 180
#define RAD2DEG 57.29577951308232f    // 180 / Pi

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint8_t fusion_tick = 0;

// --- Madgwick globals exposed by the library ---
extern volatile float q0, q1, q2, q3;      // quaternion (from Madgwick)
extern volatile float sampleFreq;          // Madgwick internal sample rate
static float gyro_bias_dps[3] = {0};       // boot-time gyro bias estimate

// There are 3 axes of data for both the accelerometer and gyroscope, each a 16 bit value
int16_t accel_raw[3] = {0}, gyro_raw[3] = {0};
float accel_g[3] = {0}, gyro_dps[3] = {0};

// Making an instance of the ctx_t struct to use in accessing the lsm6dsr
stmdev_ctx_t lsm6dsr_ctx;

// status register to see if new data is available
lsm6dsr_status_reg_t status;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
================================
PLATFORM COMMUNICATION FUNCTIONS
================================
*/

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    HAL_GPIO_WritePin(Chip_Select_GPIO_Port, Chip_Select_Pin, GPIO_PIN_RESET);

    uint8_t tx_buf[1] = { reg & 0x7F }; // Write operation
    HAL_SPI_Transmit(handle, tx_buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(handle, (uint8_t*)bufp, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(Chip_Select_GPIO_Port, Chip_Select_Pin, GPIO_PIN_SET);
    return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    HAL_GPIO_WritePin(Chip_Select_GPIO_Port, Chip_Select_Pin, GPIO_PIN_RESET);

    uint8_t tx_buf[1] = { reg | 0x80 }; // Read operation
    HAL_SPI_Transmit(handle, tx_buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(Chip_Select_GPIO_Port, Chip_Select_Pin, GPIO_PIN_SET);
    return 0;
}

/*
===============
SERVO FUNCTIONS
===============
*/
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle)
{
    // angle range: -90 to +90 degrees
    float pulse_length_ms = ((angle + 90.0f) / 180.0f) * 1.0f + 1.0f; // maps [-90,+90] to [1ms,2ms]

    // Convert pulse length in ms to timer counts (0.2us resolution)
    uint32_t pulse_counts = (uint32_t)(pulse_length_ms * 5000.0f);

    __HAL_TIM_SET_COMPARE(htim, Channel, pulse_counts);
}

void Servo_Sweep_Demo(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    const int delay_ms = 10;  // Adjust this for speed of sweep
    float angle;

    // 0° to -90°
    for (angle = 0; angle >= -90; angle -= 1.0f) {
        Servo_SetAngle(htim, Channel, angle);
        HAL_Delay(delay_ms);
    }

    HAL_Delay(500);

    // -90° to +90°
    for (angle = -90; angle <= 90; angle += 1.0f) {
        Servo_SetAngle(htim, Channel, angle);
        HAL_Delay(delay_ms);
    }

    HAL_Delay(500);

    // +90° back to 0°
    for (angle = 90; angle >= 0; angle -= 1.0f) {
        Servo_SetAngle(htim, Channel, angle);
        HAL_Delay(delay_ms);
    }
}

// To redirect the printf to output to the UART instead so I can see it in putty
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
      fusion_tick = 1;              // 500 Hz “call Madgwick now”
  }
}

static void IMU_CalibrateGyro(stmdev_ctx_t *ctx, float bias_out_dps[3]) {
    // Assumes the board is held still for ~0.5 s
    const int N = 200;
    int32_t sx = 0, sy = 0, sz = 0;
    int16_t g[3];
    for (int i = 0; i < N; i++) {
        lsm6dsr_angular_rate_raw_get(ctx, g);
        sx += g[0]; sy += g[1]; sz += g[2];
        HAL_Delay(2); // ~500 ms total
    }
    float gx = (float)(sx / N);
    float gy = (float)(sy / N);
    float gz = (float)(sz / N);
    bias_out_dps[0] = lsm6dsr_from_fs500dps_to_mdps((int16_t)gx) / 1000.0f;
    bias_out_dps[1] = lsm6dsr_from_fs500dps_to_mdps((int16_t)gy) / 1000.0f;
    bias_out_dps[2] = lsm6dsr_from_fs500dps_to_mdps((int16_t)gz) / 1000.0f;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);      // start periodic update IRQ

  // Setup lsm6dsr_ctx correctly for thios device setup
  lsm6dsr_ctx.handle = &hspi1;
  lsm6dsr_ctx.mdelay = HAL_Delay;
  lsm6dsr_ctx.write_reg = platform_write;
  lsm6dsr_ctx.read_reg = platform_read;

  // Perform self tests on the accelerometer and gyroscope
  lsm6dsr_xl_self_test_set(&lsm6dsr_ctx, LSM6DSR_XL_ST_POSITIVE);
  lsm6dsr_gy_self_test_set(&lsm6dsr_ctx, LSM6DSR_GY_ST_POSITIVE);
  HAL_Delay(500); // Wait for the self test to complete
  lsm6dsr_xl_self_test_set(&lsm6dsr_ctx, LSM6DSR_XL_ST_DISABLE);
  lsm6dsr_gy_self_test_set(&lsm6dsr_ctx, LSM6DSR_GY_ST_DISABLE);

  /*----------Device Reset-----------*/
  lsm6dsr_reset_set(&lsm6dsr_ctx, PROPERTY_ENABLE);
  uint8_t rst;
  do {
    lsm6dsr_reset_get(&lsm6dsr_ctx, &rst);
  } while (rst);

  // Match Madgwick rate to IMU ODR (you set 104 Hz)
  sampleFreq = 104.0f;

  // Quick gyro bias while stationary
  IMU_CalibrateGyro(&lsm6dsr_ctx, gyro_bias_dps);


  /*---------------Run Time Settings--------------*/
  lsm6dsr_block_data_update_set(&lsm6dsr_ctx, PROPERTY_ENABLE);

  // Enable high performance mode
  lsm6dsr_xl_power_mode_set(&lsm6dsr_ctx, LSM6DSR_HIGH_PERFORMANCE_MD);
  lsm6dsr_gy_power_mode_set(&lsm6dsr_ctx, LSM6DSR_GY_HIGH_PERFORMANCE);

  lsm6dsr_xl_data_rate_set(&lsm6dsr_ctx, LSM6DSR_XL_ODR_104Hz);
  lsm6dsr_gy_data_rate_set(&lsm6dsr_ctx, LSM6DSR_GY_ODR_104Hz);
  lsm6dsr_xl_full_scale_set(&lsm6dsr_ctx, LSM6DSR_8g);
  lsm6dsr_gy_full_scale_set(&lsm6dsr_ctx, LSM6DSR_500dps);

  // Enable Low Pass Filter 1 on the accelerometer and gyroscope
  lsm6dsr_xl_filter_lp2_set(&lsm6dsr_ctx, PROPERTY_ENABLE);
  lsm6dsr_gy_filter_lp1_set(&lsm6dsr_ctx, PROPERTY_ENABLE);
  lsm6dsr_gy_hp_path_internal_set(&lsm6dsr_ctx, LSM6DSR_HP_FILTER_16mHz);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    // Servo_Sweep_Demo(&htim2, TIM_CHANNEL_2);

    lsm6dsr_status_reg_get(&lsm6dsr_ctx, &status);
    
    if (status.xlda && status.gda) {
      // If both accelerometer and gyroscope data are ready, retrieve the data
      lsm6dsr_acceleration_raw_get(&lsm6dsr_ctx, (int16_t*)accel_raw);
      lsm6dsr_angular_rate_raw_get(&lsm6dsr_ctx, (int16_t*)gyro_raw);

      accel_g[0] = (lsm6dsr_from_fs8g_to_mg(accel_raw[0])) / 1000.0f;
      accel_g[1] = (lsm6dsr_from_fs8g_to_mg(accel_raw[1])) / 1000.0f;
      accel_g[2] = (lsm6dsr_from_fs8g_to_mg(accel_raw[2])) / 1000.0f;

      gyro_dps[0] = (lsm6dsr_from_fs500dps_to_mdps(gyro_raw[0])) / 1000.0f;
      gyro_dps[1] = (lsm6dsr_from_fs500dps_to_mdps(gyro_raw[1])) / 1000.0f;
      gyro_dps[2] = (lsm6dsr_from_fs500dps_to_mdps(gyro_raw[2])) / 1000.0f;

      // Normalize accel in-place (required by Madgwick)
      float inv = 1.0f / sqrtf(accel_g[0]*accel_g[0] + accel_g[1]*accel_g[1] + accel_g[2]*accel_g[2]);
      accel_g[0] *= inv;  accel_g[1] *= inv;  accel_g[2] *= inv;

      // Run fusion (IMU variant: gyro in rad/s, accel in g, normalized)
      MadgwickAHRSupdateIMU(gyro_dps[0]*DEG2RAD, gyro_dps[1]*DEG2RAD, gyro_dps[2]*DEG2RAD,
                            accel_g[0],         accel_g[1],         accel_g[2]);

      // Stream quaternion + sensors (CSV line that your Python can parse)
      printf("IMU,%lu,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\r\n",
              (unsigned long)HAL_GetTick(), q0, q1, q2, q3,
              gyro_dps[0], gyro_dps[1], gyro_dps[2],
              accel_g[0],  accel_g[1],  accel_g[2]);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 79;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Chip_Select_GPIO_Port, Chip_Select_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Chip_Select_Pin */
  GPIO_InitStruct.Pin = Chip_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Chip_Select_GPIO_Port, &GPIO_InitStruct);

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
