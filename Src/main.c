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
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app/fusion/fusion.h"
#include "app/sensors/bmp280.h"
#include "app/sensors/mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_TASK_PERIOD_MS (10U)
#define TEMP_TASK_PERIOD_MS (100U)
#define FUSION_TASK_PERIOD_MS (10U)
#define I2C1_PROBE_TIMEOUT_MS (10U)
#define I2C1_PROBE_RETRIES (5U)
#define MPU6050_ADDR_0 (0x68U)
#define MPU6050_ADDR_1 (0x69U)
#define BMP280_ADDR_0 (0x76U)
#define BMP280_ADDR_1 (0x77U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for blinkTask */
osThreadId_t blinkTaskHandle;
const osThreadAttr_t blinkTask_attributes = {
    .name = "blinkTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
    .name = "imuTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for tempTask */
osThreadId_t tempTaskHandle;
const osThreadAttr_t tempTask_attributes = {
    .name = "tempTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for fusionTask */
osThreadId_t fusionTaskHandle;
const osThreadAttr_t fusionTask_attributes = {
    .name = "fusionTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
osMutexId_t appDataMutexHandle;
const osMutexAttr_t appDataMutex_attributes = {
    .name = "appDataMutex",
};

ImuSample g_latest_imu = {0};
Bmp280Sample g_latest_bmp280 = {0};
FusionState g_fusion_state = {0};
bool g_has_imu_sample = false;
bool g_has_bmp280_sample = false;
uint32_t g_app_status_flags = APP_STATUS_FLAG_PLACEHOLDER_DATA |
                              APP_STATUS_FLAG_MPU6050_WIP |
                              APP_STATUS_FLAG_BMP280_WIP |
                              APP_STATUS_FLAG_FUSION_WIP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartBlinkTask(void *argument);
void StartImuTask(void *argument);
void StartTempTask(void *argument);
void StartFusionTask(void *argument);

/* USER CODE BEGIN PFP */
static bool I2C1_Probe7bitAddress(uint8_t address_7bit);
static bool I2C1_ProbeKnownAddresses(const uint8_t *addresses, uint32_t count);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  osKernelInitialize();

  appDataMutexHandle = osMutexNew(&appDataMutex_attributes);
  if (appDataMutexHandle == NULL)
  {
    Error_Handler();
  }

  if (!I2C1_ProbeKnownAddresses((const uint8_t[]){MPU6050_ADDR_0, MPU6050_ADDR_1},
                                2U))
  {
    Error_Handler();
  }

  if (!I2C1_ProbeKnownAddresses((const uint8_t[]){BMP280_ADDR_0, BMP280_ADDR_1},
                                2U))
  {
    Error_Handler();
  }

  if (!MPU6050_Init())
  {
    Error_Handler();
  }

  if (!BMP280_Init())
  {
    Error_Handler();
  }

  if (!Fusion_Init(&g_fusion_state))
  {
    Error_Handler();
  }

  blinkTaskHandle = osThreadNew(StartBlinkTask, NULL, &blinkTask_attributes);
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);
  tempTaskHandle = osThreadNew(StartTempTask, NULL, &tempTask_attributes);
  fusionTaskHandle = osThreadNew(StartFusionTask, NULL, &fusionTask_attributes);

  if ((blinkTaskHandle == NULL) || (imuTaskHandle == NULL) ||
      (tempTaskHandle == NULL) || (fusionTaskHandle == NULL))
  {
    Error_Handler();
  }

  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  while (1)
  {
  }
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* PB8 = I2C1_SCL, PB9 = I2C1_SDA */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  I2C1->CR1 = 0U;
  I2C1->CR1 |= I2C_CR1_SWRST;
  I2C1->CR1 &= ~I2C_CR1_SWRST;

  /* APB1 is 42 MHz -> standard mode 100 kHz. */
  I2C1->CR2 = 42U;
  I2C1->CCR = 210U;
  I2C1->TRISE = 43U;
  I2C1->CR1 = I2C_CR1_PE;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static bool I2C1_Probe7bitAddress(uint8_t address_7bit)
{
  uint32_t start_tick = HAL_GetTick();

  while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U)
  {
    if ((HAL_GetTick() - start_tick) >= I2C1_PROBE_TIMEOUT_MS)
    {
      return false;
    }
  }

  I2C1->CR1 |= I2C_CR1_START;
  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_SB) == 0U)
  {
    if ((HAL_GetTick() - start_tick) >= I2C1_PROBE_TIMEOUT_MS)
    {
      I2C1->CR1 |= I2C_CR1_STOP;
      return false;
    }
  }

  I2C1->DR = (uint8_t)(address_7bit << 1);
  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0U)
  {
    if ((HAL_GetTick() - start_tick) >= I2C1_PROBE_TIMEOUT_MS)
    {
      I2C1->CR1 |= I2C_CR1_STOP;
      return false;
    }
  }

  if ((I2C1->SR1 & I2C_SR1_AF) != 0U)
  {
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  (void)I2C1->SR1;
  (void)I2C1->SR2;
  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}

static bool I2C1_ProbeKnownAddresses(const uint8_t *addresses, uint32_t count)
{
  uint32_t idx = 0U;
  uint32_t retry = 0U;

  if ((addresses == NULL) || (count == 0U))
  {
    return false;
  }

  for (idx = 0U; idx < count; idx++)
  {
    for (retry = 0U; retry < I2C1_PROBE_RETRIES; retry++)
    {
      if (I2C1_Probe7bitAddress(addresses[idx]))
      {
        return true;
      }
      HAL_Delay(2U);
    }
  }

  return false;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinkTask */
/**
 * @brief  Function implementing the blinkTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
 * @brief Function implementing the imuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  ImuSample sample = {0};
  (void)argument;

  for (;;)
  {
    if (MPU6050_ReadGyroAccel(&sample))
    {
      if (osMutexAcquire(appDataMutexHandle, osWaitForever) == osOK)
      {
        g_latest_imu = sample;
        g_has_imu_sample = true;
        osMutexRelease(appDataMutexHandle);
      }
    }
    osDelay(IMU_TASK_PERIOD_MS);
  }
}

/* USER CODE BEGIN Header_StartTempTask */
/**
 * @brief Function implementing the tempTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTempTask */
void StartTempTask(void *argument)
{
  Bmp280Sample sample = {0};
  (void)argument;

  for (;;)
  {
    if (BMP280_ReadTemperaturePressure(&sample))
    {
      if (osMutexAcquire(appDataMutexHandle, osWaitForever) == osOK)
      {
        g_latest_bmp280 = sample;
        g_has_bmp280_sample = true;
        osMutexRelease(appDataMutexHandle);
      }
    }
    osDelay(TEMP_TASK_PERIOD_MS);
  }
}

/* USER CODE BEGIN Header_StartFusionTask */
/**
 * @brief Function implementing the fusionTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFusionTask */
void StartFusionTask(void *argument)
{
  ImuSample imu_sample = {0};
  Bmp280Sample bmp280_sample = {0};
  FusionState fusion_state = {0};
  bool have_samples = false;
  (void)argument;

  for (;;)
  {
    have_samples = false;

    if (osMutexAcquire(appDataMutexHandle, osWaitForever) == osOK)
    {
      if (g_has_imu_sample && g_has_bmp280_sample)
      {
        imu_sample = g_latest_imu;
        bmp280_sample = g_latest_bmp280;
        fusion_state = g_fusion_state;
        have_samples = true;
      }
      osMutexRelease(appDataMutexHandle);
    }

    if (have_samples && Fusion_Update(&imu_sample, &bmp280_sample, &fusion_state))
    {
      if (osMutexAcquire(appDataMutexHandle, osWaitForever) == osOK)
      {
        g_fusion_state = fusion_state;
        osMutexRelease(appDataMutexHandle);
      }
    }

    osDelay(FUSION_TASK_PERIOD_MS);
  }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
