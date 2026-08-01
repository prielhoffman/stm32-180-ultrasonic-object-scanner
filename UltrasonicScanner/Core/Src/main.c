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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
    SYSTEM_STATE_STARTUP = 0,
    SYSTEM_STATE_SCANNING,
    SYSTEM_STATE_DETECTED,
    SYSTEM_STATE_ERROR
} SystemState_t;

typedef struct{
	SystemState_t state;
    uint16_t angleDeg;
    uint16_t distanceCm;
    uint8_t measurementValid;
} ScanMessage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HC_SR04_TIMEOUT_US           30000U

#define OBJECT_DETECT_DISTANCE_CM    25U
#define OBJECT_RELEASE_DISTANCE_CM   30U

#define DETECT_CONFIRM_SAMPLES        2U
#define RELEASE_CONFIRM_SAMPLES       3U

#define BUZZER_TASK_PERIOD_TICKS      10U
#define BUZZER_ON_TIME_TICKS          60U

#define BUZZER_GAP_NEAR_TICKS        100U
#define BUZZER_GAP_MEDIUM_TICKS      300U
#define BUZZER_GAP_FAR_TICKS         700U

#define BUZZER_NEAR_DISTANCE_CM       10U
#define BUZZER_MEDIUM_DISTANCE_CM     17U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for ScannerTask */
osThreadId_t ScannerTaskHandle;
const osThreadAttr_t ScannerTask_attributes = {
  .name = "ScannerTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for AlertTask */
osThreadId_t AlertTaskHandle;
const osThreadAttr_t AlertTask_attributes = {
  .name = "AlertTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for ScanDataQueue */
osMessageQueueId_t ScanDataQueueHandle;
const osMessageQueueAttr_t ScanDataQueue_attributes = {
  .name = "ScanDataQueue"
};
/* Definitions for AlertDataQueue */
osMessageQueueId_t AlertDataQueueHandle;
const osMessageQueueAttr_t AlertDataQueue_attributes = {
  .name = "AlertDataQueue"
};
/* USER CODE BEGIN PV */
LCD_I2C_HandleTypeDef lcd;

static int16_t scanAngle = 90;
static int8_t scanDirection = 1;

static SystemState_t systemState = SYSTEM_STATE_SCANNING;

static uint8_t detectConfirmCount = 0U;
static uint8_t releaseConfirmCount = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void StartScannerTask(void *argument);
void StartDisplayTask(void *argument);
void StartAlertTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DelayUs(uint16_t delayUs){
    /* Start counting again from zero */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);

    /* TIM2 advances once every microsecond, stay here until the requested time has elapsed */
    while (__HAL_TIM_GET_COUNTER(&htim2) < delayUs){
        /* Busy wait */
    }
}

static uint32_t HC_SR04_ReadEchoPulseUs(void){
    /* Ensure TRIG starts LOW */
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    DelayUs(2U);

    /* Send a 10 us trigger pulse */
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);

    DelayUs(10U);

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    /* Wait for ECHO to rise */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);

    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET){
        if (__HAL_TIM_GET_COUNTER(&htim2) >= HC_SR04_TIMEOUT_US){
            return 0U;
        }
    }

    /* ECHO is now HIGH, start measuring its pulse width */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);

    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET){
        if (__HAL_TIM_GET_COUNTER(&htim2) >= HC_SR04_TIMEOUT_US){
            return 0U;
        }
    }

    /* TIM2 counts once per microsecond */
    return __HAL_TIM_GET_COUNTER(&htim2);
}


static void Servo_SetAngle(uint8_t angle){
    uint32_t pulseUs;

    /* Protect the servo from receiving an angle outside the supported 0-180 degree range */
    if (angle > 180U){
        angle = 180U;
    }

    /*
     * Map:
     *   0 degrees   -> 1000 us
     *   90 degrees  -> 1500 us
     *   180 degrees -> 2000 us
     */
    pulseUs = 1000U + (((uint32_t)angle * 1000U) / 180U);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulseUs);
}

static void Scanner_AdvanceAngle(void){
    if (scanDirection > 0){
        if (scanAngle >= 170){
            scanDirection = -1;
            scanAngle--;
        }
        else{
            scanAngle++;
        }
    }
    else{
        if (scanAngle <= 10){
            scanDirection = 1;
            scanAngle++;
        }
        else{
            scanAngle--;
        }
    }
}

static uint32_t Alert_GetBuzzerGapTicks(uint16_t distanceCm){
    if (distanceCm <= BUZZER_NEAR_DISTANCE_CM){
        return BUZZER_GAP_NEAR_TICKS;
    }

    if (distanceCm <= BUZZER_MEDIUM_DISTANCE_CM){
        return BUZZER_GAP_MEDIUM_TICKS;
    }

    return BUZZER_GAP_FAR_TICKS;
}

static void Application_RtosStartupError(void){
    /*
     * Keep the system in a visible and safe state
     * The scheduler has not started yet, so no task can change these outputs
     */
    Servo_SetAngle(90U);

    HAL_GPIO_WritePin(LED_GREEN_EXT_GPIO_Port, LED_GREEN_EXT_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED_RED_EXT_GPIO_Port, LED_RED_EXT_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(BUZZER_CTRL_GPIO_Port, BUZZER_CTRL_Pin, GPIO_PIN_RESET);

    LCD_I2C_Clear(&lcd);

    LCD_I2C_SetCursor(&lcd, 0U, 0U);
    LCD_I2C_Print(&lcd, "SYSTEM ERROR    ");

    LCD_I2C_SetCursor(&lcd, 1U, 0U);
    LCD_I2C_Print(&lcd, "RTOS STARTUP    ");

    /* Stop here instead of starting a partially initialized system */
    for (;;){
        HAL_Delay(1000U);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  Servo_SetAngle(90U);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htim2);

  HAL_Delay(100U);

  LCD_I2C_Init(&lcd, &hi2c1, 0x27U);
  LCD_I2C_Clear(&lcd);

  LCD_I2C_SetCursor(&lcd, 0U, 0U);
  LCD_I2C_Print(&lcd, "SYSTEM STARTING ");

  LCD_I2C_SetCursor(&lcd, 1U, 0U);
  LCD_I2C_Print(&lcd, "PLEASE WAIT...  ");

  HAL_Delay(1000U);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ScanDataQueue */
  ScanDataQueueHandle = osMessageQueueNew (4, sizeof(ScanMessage_t), &ScanDataQueue_attributes);

  /* creation of AlertDataQueue */
  AlertDataQueueHandle = osMessageQueueNew (4, sizeof(ScanMessage_t), &AlertDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  if ((ScanDataQueueHandle == NULL) || (AlertDataQueueHandle == NULL)){
      Application_RtosStartupError();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ScannerTask */
  ScannerTaskHandle = osThreadNew(StartScannerTask, NULL, &ScannerTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of AlertTask */
  AlertTaskHandle = osThreadNew(StartAlertTask, NULL, &AlertTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if ((ScannerTaskHandle == NULL) || (DisplayTaskHandle == NULL) || (AlertTaskHandle == NULL)){
      Application_RtosStartupError();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|TRIG_Pin|BUZZER_CTRL_Pin|LED_GREEN_EXT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_EXT_GPIO_Port, LED_RED_EXT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin BUZZER_CTRL_Pin LED_GREEN_EXT_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|BUZZER_CTRL_Pin|LED_GREEN_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_EXT_Pin */
  GPIO_InitStruct.Pin = LED_RED_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_EXT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartScannerTask */
/**
  * @brief  Function implementing the ScannerTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartScannerTask */
void StartScannerTask(void *argument){
  /* USER CODE BEGIN 5 */
  for (;;){
      ScanMessage_t message;
      uint32_t echoPulseUs;

      /* SCANNING: Move smoothly and measure every three degrees */
      if (systemState == SYSTEM_STATE_SCANNING){
          Servo_SetAngle((uint8_t)scanAngle);
          osDelay(20U);

          /* At angles that are not measurement points, simply continue the sweep */
          if ((scanAngle % 3) != 0){
              Scanner_AdvanceAngle();
              continue;
          }

          echoPulseUs = HC_SR04_ReadEchoPulseUs();

          message.angleDeg = (uint16_t)scanAngle;

          if (echoPulseUs == 0U){
              message.distanceCm = 0U;
              message.measurementValid = 0U;

              /* A timeout cannot confirm an object */
              detectConfirmCount = 0U;
          }
          else{
              message.distanceCm = (uint16_t)(echoPulseUs / 58U);

              message.measurementValid = 1U;

              if (message.distanceCm <= OBJECT_DETECT_DISTANCE_CM){
                  detectConfirmCount++;

                  /* Stay at this angle while waiting for the confirming measurement */
                  if (detectConfirmCount >= DETECT_CONFIRM_SAMPLES){
                      systemState = SYSTEM_STATE_DETECTED;

                      detectConfirmCount = 0U;
                      releaseConfirmCount = 0U;
                  }
              }
              else{
                  detectConfirmCount = 0U;
              }
          }

          message.state = systemState;

          (void)osMessageQueuePut(ScanDataQueueHandle, &message, 0U, 0U);

          (void)osMessageQueuePut(AlertDataQueueHandle, &message, 0U, 0U);

          /* Continue moving only when no possible object is waiting for confirmation */
          if ((systemState == SYSTEM_STATE_SCANNING) && (detectConfirmCount == 0U)){
              Scanner_AdvanceAngle();
          }
      }

      /* DETECTED: Keep commanding the same servo angle and continue measuring the object */
      else if (systemState == SYSTEM_STATE_DETECTED){
          Servo_SetAngle((uint8_t)scanAngle);

          /* Measurements while holding the object do not need to run every 20 ms */
          osDelay(100U);

          echoPulseUs = HC_SR04_ReadEchoPulseUs();

          message.angleDeg = (uint16_t)scanAngle;

          if (echoPulseUs == 0U){
              message.distanceCm = 0U;
              message.measurementValid = 0U;

              /* A timeout may mean that the object is no longer in front of the sensor */
              releaseConfirmCount++;
          }
          else{
              message.distanceCm = (uint16_t)(echoPulseUs / 58U);

              message.measurementValid = 1U;

              if (message.distanceCm >= OBJECT_RELEASE_DISTANCE_CM){
                  releaseConfirmCount++;
              }
              else{
                  releaseConfirmCount = 0U;
              }
          }

          if (releaseConfirmCount >= RELEASE_CONFIRM_SAMPLES){
              systemState = SYSTEM_STATE_SCANNING;

              releaseConfirmCount = 0U;
              detectConfirmCount = 0U;
          }

          message.state = systemState;

          (void)osMessageQueuePut(ScanDataQueueHandle, &message, 0U, 0U);

          (void)osMessageQueuePut(AlertDataQueueHandle, &message, 0U, 0U);
      }

      /* ERROR is not implemented yet */
      else{
          osDelay(100U);
      }
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument){
  /* USER CODE BEGIN StartDisplayTask */

  ScanMessage_t message;

  /* 16 visible characters plus the terminating '\0' */
  char line1[17];
  char line2[17];

  LCD_I2C_Clear(&lcd);

  for (;;){
      /* Sleep until ScannerTask sends a new system snapshot */
      if (osMessageQueueGet(ScanDataQueueHandle, &message, NULL, osWaitForever) == osOK){
          if (message.state == SYSTEM_STATE_SCANNING){
              /* Exactly 16 visible characters */
        	  (void)snprintf(line1, sizeof(line1), "AREA CLEAR      ");
        	  (void)snprintf(line2, sizeof(line2), "ANGLE:%3u deg   ", (unsigned int)message.angleDeg);
          }
          else if (message.state == SYSTEM_STATE_DETECTED){
              (void)snprintf(line1, sizeof(line1), "FOREIGN BODY    ");

              if (message.measurementValid != 0U){
                  (void)snprintf(line2, sizeof(line2), "A:%3u D:%3ucm   ", (unsigned int)message.angleDeg, (unsigned int)message.distanceCm);
              }
              else{
                  (void)snprintf(line2, sizeof(line2), "A:%3u D:---cm   ", (unsigned int)message.angleDeg);
              }
          }
          else{
              (void)snprintf(line1, sizeof(line1), "SYSTEM ERROR    ");

              (void)snprintf(line2, sizeof(line2), "CHECK HARDWARE  ");
          }

          /*
           * Overwrite both complete rows
           * No Clear command is needed on every update, so the display should not flicker
           */
          LCD_I2C_SetCursor(&lcd, 0U, 0U);

          LCD_I2C_Print(&lcd, line1);

          LCD_I2C_SetCursor(&lcd, 1U, 0U);

          LCD_I2C_Print(&lcd, line2);
      }
  }

  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartAlertTask */
/**
* @brief Function implementing the AlertTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlertTask */
void StartAlertTask(void *argument)
{
  /* USER CODE BEGIN StartAlertTask */

  ScanMessage_t message;

  SystemState_t currentState = SYSTEM_STATE_STARTUP;

  uint16_t currentDistanceCm = 0U;
  uint8_t currentMeasurementValid = 0U;

  uint8_t buzzerIsOn = 0U;
  uint32_t nextBuzzerChangeTick = 0U;

  /*
   * Begin with every alert output disabled.
   */
  HAL_GPIO_WritePin(
      LED_GREEN_EXT_GPIO_Port,
      LED_GREEN_EXT_Pin,
      GPIO_PIN_RESET
  );

  HAL_GPIO_WritePin(
      LED_RED_EXT_GPIO_Port,
      LED_RED_EXT_Pin,
      GPIO_PIN_RESET
  );

  HAL_GPIO_WritePin(
      BUZZER_CTRL_GPIO_Port,
      BUZZER_CTRL_Pin,
      GPIO_PIN_RESET
  );

  for (;;)
  {
      osStatus_t queueStatus;
      uint32_t currentTick;

      /*
       * Wait for a message, but only for 10 ticks.
       *
       * The finite timeout allows this task to wake up periodically
       * and update the buzzer pattern even when no new queue message arrives.
       */
      queueStatus = osMessageQueueGet(AlertDataQueueHandle, &message, NULL, BUZZER_TASK_PERIOD_TICKS);

      currentTick = osKernelGetTickCount();

      if (queueStatus == osOK){
          uint8_t enteredDetectedState;

          enteredDetectedState = ((currentState != SYSTEM_STATE_DETECTED) && (message.state == SYSTEM_STATE_DETECTED)) ? 1U : 0U;

          currentState = message.state;
          currentDistanceCm = message.distanceCm;
          currentMeasurementValid = message.measurementValid;

          if (currentState == SYSTEM_STATE_SCANNING){
              HAL_GPIO_WritePin(LED_GREEN_EXT_GPIO_Port, LED_GREEN_EXT_Pin, GPIO_PIN_SET);

              HAL_GPIO_WritePin(LED_RED_EXT_GPIO_Port, LED_RED_EXT_Pin, GPIO_PIN_RESET);
          }
          else if (currentState == SYSTEM_STATE_DETECTED){
              HAL_GPIO_WritePin(LED_GREEN_EXT_GPIO_Port, LED_GREEN_EXT_Pin, GPIO_PIN_RESET);

              HAL_GPIO_WritePin(LED_RED_EXT_GPIO_Port, LED_RED_EXT_Pin, GPIO_PIN_SET);
          }
          else{
              HAL_GPIO_WritePin(LED_GREEN_EXT_GPIO_Port, LED_GREEN_EXT_Pin, GPIO_PIN_RESET);

              HAL_GPIO_WritePin(LED_RED_EXT_GPIO_Port, LED_RED_EXT_Pin, GPIO_PIN_RESET);
          }

          /*
           * The buzzer is allowed to operate only when:
           * 1. The system is in DETECTED
           * 2. The latest measurement is valid
           */
          if ((currentState != SYSTEM_STATE_DETECTED) || (currentMeasurementValid == 0U)){
              HAL_GPIO_WritePin(BUZZER_CTRL_GPIO_Port, BUZZER_CTRL_Pin, GPIO_PIN_RESET);

              buzzerIsOn = 0U;
              nextBuzzerChangeTick = currentTick;
          }
          else if (enteredDetectedState != 0U){
              /* Schedule the first beep immediately after entering DETECTED */
              nextBuzzerChangeTick = currentTick;
          }
      }

      /* Generate the buzzer pattern independently of the rate at which messages arrive */
      if ((currentState == SYSTEM_STATE_DETECTED) && (currentMeasurementValid != 0U)){
          /*
           * Casting the subtraction to int32_t makes the comparison safe
           * even when the RTOS tick counter eventually wraps around
           */
          if ((int32_t)(currentTick - nextBuzzerChangeTick) >= 0){
              if (buzzerIsOn != 0U){
                  uint32_t gapTicks;

                  HAL_GPIO_WritePin(BUZZER_CTRL_GPIO_Port, BUZZER_CTRL_Pin, GPIO_PIN_RESET);

                  buzzerIsOn = 0U;

                  gapTicks = Alert_GetBuzzerGapTicks(currentDistanceCm);

                  nextBuzzerChangeTick = currentTick + gapTicks;
              }
              else{
                  HAL_GPIO_WritePin(BUZZER_CTRL_GPIO_Port, BUZZER_CTRL_Pin, GPIO_PIN_SET);

                  buzzerIsOn = 1U;

                  nextBuzzerChangeTick = currentTick + BUZZER_ON_TIME_TICKS;
              }
          }
      }
  }

  /* USER CODE END StartAlertTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
