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
#include "bmi088.h"
#include "i2c_recovery.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_INIT_ATTEMPTS (3U)    /* Init tries before giving up (engineering choice) */
#define GYRO_RETRY_PERIOD_MS  (1000U)  /* Re-init interval while not NORMAL       */
#define PRINT_PERIOD_MS       (100U)   /* Debug print to PuTTY: 10 lines/second   */
#define UART_TX_TIMEOUT_MS    (20U)    /* ~90 chars at 115200 baud take ~8 ms     */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static bmi088_gyro_t gyro;    /* BMI088 gyro state; zero (UNINIT) at start-up */
static volatile uint32_t gyro_sample_count;    /* Completed reads; ~100 per second */
static const i2c_recovery_pins_t i2c2_pins = {GPIOF, GPIO_PIN_1, GPIOF, GPIO_PIN_0};    /* SCL PF1, SDA PF0 */
static uint32_t i2c_reset_count;       /* Peripheral resets, for telemetry      */
static uint32_t i2c_bus_clear_count;   /* Bus clears, for telemetry             */
static i2c_recovery_status_t i2c_recovery_status;    /* Last recovery result    */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Print one line of gyro data to the ST-LINK virtual COM port
  *         (USART1, 115200 8N1), e.g. for PuTTY.
  * @note   The gyro fields are copied with interrupts briefly disabled, so
  *         the line never mixes two samples (TIM6 / I2C interrupts update
  *         them). Main-loop use only: HAL_UART_Transmit() blocks ~8 ms.
  * @param  init_status  Last result of bmi088_gyro_init().
  */
static void print_gyro(bmi088_status_t init_status)
{
  char line[128];
  int len;
  uint32_t primask;
  uint32_t count;
  bmi088_dev_state_t state;
  uint8_t faults;
  int16_t raw[BMI088_AXES];
  bmi088_float32_t rate[BMI088_AXES];
  uint8_t axis;

  primask = __get_PRIMASK();
  __disable_irq();
  count = gyro_sample_count;
  state = gyro.state;
  faults = gyro.fault_count;
  for (axis = 0U; axis < BMI088_AXES; axis++)
  {
    raw[axis] = gyro.sample[axis];
    rate[axis] = gyro.angular_velocity[axis];
  }
  __set_PRIMASK(primask);

  len = snprintf(line, sizeof(line),
                 "n=%lu st=%u fc=%u init=%u  X=%+.6f Y=%+.6f Z=%+.6f rad/s  raw=%d %d %d\r\n",
                 (unsigned long)count, (unsigned int)state, (unsigned int)faults,
                 (unsigned int)init_status,
                 (double)rate[0], (double)rate[1], (double)rate[2],
                 (int)raw[0], (int)raw[1], (int)raw[2]);

  if (len > 0)
  {
    if ((size_t)len >= sizeof(line))
    {
      len = (int)sizeof(line) - 1;    /* truncated: send what fits */
    }
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)line, (uint16_t)len,
                            UART_TX_TIMEOUT_MS);
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
  bmi088_status_t gyro_status = BMI088_ERR_STATE;    /* not initialised yet */
  uint8_t gyro_attempt = 0U;
  uint32_t now;
  uint32_t gyro_retry_tick;
  uint32_t print_tick;
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
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  while ((gyro_status != BMI088_OK) && (gyro_attempt < GYRO_INIT_ATTEMPTS))
  {
    gyro_status = bmi088_gyro_init(&gyro, &hi2c2);
    gyro_attempt++;
  }

  if (gyro_status != BMI088_OK)
  {
    /* Gyro unavailable after all attempts: gyro.state is
     * BMI088_STATE_ERROR, so no gyro reads are started.
     * The rest of the system keeps running; the main loop retries. */
  }

  gyro_retry_tick = HAL_GetTick();
  print_tick = gyro_retry_tick;

  /* Start the 100 Hz timer: each TIM6 interrupt starts one gyro read.
   * Started even if init failed; reads begin once a re-init succeeds. */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Unsigned subtraction stays correct when HAL_GetTick() wraps (~49.7 days). */
    now = HAL_GetTick();

    /* Gyro reads are started by the TIM6 interrupt, not here.
     * Recovery: while the gyro is not NORMAL, one re-init attempt per second.
     * No read is started during re-init: start_read requires NORMAL, and
     * init sets NORMAL only after its last register access. */
    if ((gyro.state != BMI088_STATE_NORMAL) &&
        ((now - gyro_retry_tick) >= GYRO_RETRY_PERIOD_MS))
    {
      gyro_retry_tick = now;

      /* Recovery ladder: reset the I2C peripheral, clear the bus if a line is
       * still low, then re-initialise the sensor. No read can start meanwhile:
       * start_read requires NORMAL. */
      i2c_recovery_status = i2c_recovery_reset_peripheral(&hi2c2);
      i2c_reset_count++;

      if ((i2c_recovery_status == I2C_RECOVERY_OK) &&
          (i2c_recovery_lines_high(&i2c2_pins) == 0U))
      {
        i2c_recovery_status = i2c_recovery_clear_bus(&hi2c2, &i2c2_pins);
        i2c_bus_clear_count++;
      }

      gyro_status = bmi088_gyro_init(&gyro, &hi2c2);
    }

    /* Debug output only (not part of the data path): 10 lines per second. */
    if ((now - print_tick) >= PRINT_PERIOD_MS)
    {
      print_tick = now;
      print_gyro(gyro_status);
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM6 update interrupt, every 10 ms: start one gyro read.
  * @param  htim Timer that overflowed.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    (void)bmi088_gyro_start_read(&gyro);
  }
}

/**
  * @brief  I2C memory read finished: process the gyro sample.
  * @param  hi2c I2C bus that finished.
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2)
  {
    (void)bmi088_gyro_read_complete(&gyro);
    gyro_sample_count++;
  }
}

/**
  * @brief  I2C transfer failed: count the gyro fault.
  * @param  hi2c I2C bus that reported the error.
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2)
  {
    (void)bmi088_gyro_read_error(&gyro);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
