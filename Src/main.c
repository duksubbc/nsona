/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "buffer.h"
#include "ringbuffer.h"
#include "xprintf.h"
#include "soft_timer.h"
#include "keyscan.h"

#include "task1.h"
#include "task2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
             
//#define USEC100  (100)
//#define SEC      (1000*10)
                      
                          
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CONFIG_T gConfig;
uint8_t  UARTRxBuffIdx;
uint8_t  UARTRxBuff[128];
// char     Received[256];
// BUFFER_t USART_Buffer;
// uint8_t  USARTBuffer[RX_RING_SIZE];

uint16_t tim3_ch1_duty;

uint8_t  console_debug = 1;



 uint8_t  RF_Start = 0;
 uint16_t RF_PSC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


int32_t setTBDAndDuty(int32_t tbd,int32_t duty);
int32_t setTiming(int32_t SD,int32_t ISI,int32_t *pTD);
int32_t calcurateSD(int32_t sd,int32_t prp);

void InituserTask03(CONFIG_T *sysconf);
void userTask03(CONFIG_T *sysconf);


extern void dig_port(uint16_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CONFIG_MODE(uint32_t timeOut)
{
  gConfig.state = 1;
  gConfig.timeout = timeOut; 
}

void GPIO_HV_Enable(uint8_t enable)
{
  //Single HV EN
  if(enable)
    GPIOC->BSRR  = EN_HV_Pin;
  else
    GPIOC->BSRR = (uint32_t)EN_HV_Pin << 16U;
  
  //Multy HV EN
  if(enable)
    GPIOB->BSRR  = BATT_V_Pin;
  else
    GPIOB->BSRR = (uint32_t)BATT_V_Pin << 16U;
}


void MD1213_OE_Enable(uint8_t enable)
{
  if(enable)
    MD1213_OE_GPIO_Port->BSRR  = MD1213_OE_Pin;
  else
    MD1213_OE_GPIO_Port->BSRR = (uint32_t)MD1213_OE_Pin << 16U;
}

void STHV748_THSD_Enable(uint8_t enable)
{
  if(enable)
    STHV748_THSD_GPIO_Port->BSRR  = STHV748_THSD_Pin;
  else
    STHV748_THSD_GPIO_Port->BSRR = (uint32_t)STHV748_THSD_Pin << 16U;
}

void LED1_ON(uint8_t on)
{
  if(!on)
    LED1_GPIO_Port->BSRR  = LED1_Pin;
  else
    LED1_GPIO_Port->BSRR = (uint32_t)LED1_Pin << 16U;
}

void calcuarateEnv(CONFIG_T *env, Timer_T *t)
{

}
void Load_Env(CONFIG_T *env)
{
  if(env != NULL)
  {

    // NvCheckAndReset();
    // NvReadAll(env);

     env->setFrequency = 250000;
     env->setOutputVoltage = 50;

     // 1 = 4us
     env->setTBD  =  from100uSEC(1);   // range 0.1 ~ 50ms  (100 ~ 50000)
     env->setDuty =  50;   //  range   1 ~ 100 %
     env->setPRP  = setTBDAndDuty(env->setTBD,env->setDuty);

     env->setSD  =  env->setPRP*5;      // range 1 ~ 500ms  (1000 ~ 500000)
     env->setISI =  env->setSD;       // s
     env->setBI  = env->setSD + env->setISI;    // s 
     env->setTD  = 10*SEC;     // s 

     env->setImpedance = 50;
    
    
    env->setDelay[RF_CH0] = 0*USEC100;
    env->setDelay[RF_CH1] = 0*USEC100;
    env->setDelay[RF_CH2] = 10*USEC100;
    env->setDelay[RF_CH3] = 20*USEC100;
    env->setDelay[RF_CH4] = 30*USEC100;
    env->setDelay[RF_CH5] = 40*USEC100;
    env->setDelay[RF_CH6] = 50*USEC100;
    

    env->setAbnormalStopMode = 0;
    env->setAbnormalStopMaxV = 50;
    env->setAbnormalStopMaxI = 50;
    env->setAbnormalStopMinV = 50;
    env->setAbnormalStopMinI = 50;

    env->sonication = 0;

  }

}

void display_version(void)
{
  DBG_INFO("\r\nVersion %s(%s %s)\r\n",_VERSION_,__DATE__,__TIME__);
}

void indro_message(void)
{
#if defined(CONFIG_MULTI)
  DBG_INFO("\r\n\r\nNEUROSONA NS-US300 Multi  RF ES B'rd");
#else
  DBG_INFO("\r\n\r\nNEUROSONA NS-US300 Single RF ES B'rd");
#endif
  DBG_INFO("\r\nBuild Date  %s %s",__DATE__,__TIME__);
  DBG_INFO("\r\nFW  Version %s\r\n",_VERSION_);
  DBG_INFO_MAG("NEUROSONA Co., Ltd. ");
  DBG_INFO_CYN("www.neurosona.com\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  uint8_t resis = 0;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  InitSysTimeOut();
#if defined(DEBUG_UART3)
  DEBUG_LL_USARTInit(&huart1,&huart3);
  BUFFER_Init(&USART_Buffer, RX_RING_SIZE, USARTBuffer);
#elif defined(DEBUG_UART2)
  DEBUG_LL_USARTInit(&huart1,&huart2);
  BUFFER_Init(&USART_Buffer, RX_RING_SIZE, USARTBuffer);
#else
  DEBUG_LL_USARTInit(&huart1,NULL);
  BUFFER_Init(&USART_Buffer, RX_RING_SIZE, USARTBuffer);
#endif
  indro_message();
  
  AUDIO_RESET_LOW();
  HAL_Delay(100);
  AUDIO_RESET_HI();
  #if defined(CONFIG_MULTI)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, BATT_V_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : BATT_V_Pin  */
    GPIO_InitStruct.Pin = BATT_V_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  #endif
  GPIO_HV_Enable(DISABLE);
  
#if defined(CONFIG_MULTI)  
  STHV748_THSD_Enable(DISABLE);
#else
  MD1213_OE_Enable(DISABLE);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  UARTRxBuffIdx = 0;
  
#if defined(DEBUG_UART3)
  HAL_UART_Receive_IT(&huart3,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);
  HAL_UART_Receive_IT(&huart1,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);  
#elif defined(DEBUG_UART2)
  HAL_UART_Receive_IT(&huart2,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1); 
  HAL_UART_Receive_IT(&huart1,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);  
#else
  HAL_UART_Receive_IT(&huart1,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);  
#endif
  
  Load_Env(&gConfig);
  
  InituserTask01(&gConfig);
  InituserTask02(&gConfig);
  InituserTask03(&gConfig);
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    userTask01(&gConfig);
    userTask02(&gConfig);
    userTask03(&gConfig);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 120-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim3.Init.Prescaler = 59;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 120;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim5, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim5, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  htim6.Init.Prescaler = 25000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 240-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 59;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 120-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_HV_Pin|STHV748_A_IN3_Pin|STHV748_B_IN3_Pin|STHV748_D_IN3_Pin 
                          |STHV748_IN4_Pin|STHV748_THSD_Pin|LED1_Pin|LED2_Pin 
                          |AUDIO_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLE_RESET_Pin|STHV748_C_IN3_Pin|MD1213_OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AUDIO_CS_GPIO_Port, AUDIO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : EN_HV_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = EN_HV_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STHV748_A_IN3_Pin STHV748_B_IN3_Pin STHV748_D_IN3_Pin STHV748_IN4_Pin 
                           STHV748_THSD_Pin */
  GPIO_InitStruct.Pin = STHV748_A_IN3_Pin|STHV748_B_IN3_Pin|STHV748_D_IN3_Pin|STHV748_IN4_Pin 
                          |STHV748_THSD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_RESET_Pin STHV748_C_IN3_Pin */
  GPIO_InitStruct.Pin = BLE_RESET_Pin|STHV748_C_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AUDIO_CS_Pin */
  GPIO_InitStruct.Pin = AUDIO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AUDIO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AUDIO_RESET_Pin */
  GPIO_InitStruct.Pin = AUDIO_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUDIO_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AUDIO_BUSY_Pin */
  GPIO_InitStruct.Pin = AUDIO_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MD1213_OE_Pin */
  GPIO_InitStruct.Pin = MD1213_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MD1213_OE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void setPWMDuty(uint16_t duty)
{
  uint32_t period = 1000;
  uint32_t pulse;
  double x,y;
  
  y = duty;
  
  x = (y/100)*period;
  
  pulse = (uint32_t)x;

#if 0 
  TIM3->CCR1 = pulse;
#else
  
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  
  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);

  /* USER CODE BEGIN TIM3_Init 1 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
#endif
  return ;
  
}

int32_t gTBD[MAX_RF_CH];
int32_t gPRP[MAX_RF_CH];
int32_t gSD[MAX_RF_CH];
int32_t gISI[MAX_RF_CH];
int32_t gBI[MAX_RF_CH];
int32_t gTD[MAX_RF_CH];
int32_t gDelay[MAX_RF_CH];

static void init_timing(void)
{
  int i;
     
  for(i = RF_CH0 ; i < MAX_RF_CH ; i++) {
    gTBD[i] = -1;
    gPRP[i] = -1;
    gSD[i]  = -1;
    gISI[i] = -1;
    gBI[i]  = -1;
    gTD[i]  = -1;
  }
  
  gDelay[RF_CH0] = gConfig.setDelay[RF_CH0];
  gDelay[RF_CH1] = gConfig.setDelay[RF_CH1];
  gDelay[RF_CH2] = gConfig.setDelay[RF_CH2];
  gDelay[RF_CH3] = gConfig.setDelay[RF_CH3];
  gDelay[RF_CH4] = gConfig.setDelay[RF_CH4];
  gDelay[RF_CH5] = gConfig.setDelay[RF_CH5];
  gDelay[RF_CH6] = gConfig.setDelay[RF_CH6];
}


static void set_timing(void)
{
  int i;
      
//    gTBD = gConfig.setTBD;
//    gPRP = gConfig.setPRP;
//    gSD  = gConfig.setSD;
//    gISI = gConfig.setISI;
//    gBI  = gConfig.setBI;
//    gTD  = gConfig.setTD;
  for(i = RF_CH0 ; i < MAX_RF_CH ; i++) {
    gTBD[i] = gConfig.setTBD;
    gPRP[i] = gConfig.setPRP;
    gSD[i]  = gConfig.setSD;
    gISI[i] = gConfig.setISI;
    gBI[i]  = gConfig.setBI;
    gTD[i]  = gConfig.setTD;
  }
  
  gDelay[RF_CH0] = gConfig.setDelay[RF_CH0];
  gDelay[RF_CH1] = gConfig.setDelay[RF_CH1];
  gDelay[RF_CH2] = gConfig.setDelay[RF_CH2];
  gDelay[RF_CH3] = gConfig.setDelay[RF_CH3];
  gDelay[RF_CH4] = gConfig.setDelay[RF_CH4];
  gDelay[RF_CH5] = gConfig.setDelay[RF_CH5];
  gDelay[RF_CH6] = gConfig.setDelay[RF_CH6];
  
  INFO("\r\ncycle (TD %d/%d)\r\n",(gTD[RF_CH0]/SEC),(gConfig.setTD/SEC));
}

#if !defined(CONFIG_MULTI)
static void start_single_timer(void)
{
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  
}

static void stop_single_timer(void)
{
  HAL_TIM_Base_Stop(&htim6);
  HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
}
#endif

#if defined(CONFIG_MULTI)
static void start_multi_timer(uint8_t ch)
{
  if(ch == RF_CH0) {
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  } else if(ch == RF_CH1) {
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  } else if(ch == RF_CH2) {
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  } else if(ch == RF_CH3) {
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  } else if(ch == RF_CH4) {
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  }
}

static void stop_multi_timer(void)
{
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  
  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
}
#endif

void setSonication(int32_t mode)
{
  if(mode  == 0) {
    init_timing();
    
    GPIO_HV_Enable(0);
#if defined(CONFIG_MULTI)
    STHV748_THSD_Enable(DISABLE);
    stop_multi_timer();
    HAL_TIM_Base_Stop(&htim6);
    TIM6->PSC = RF_PSC;
    RF_Start = 0;
    //HAL_TIM_Base_Start(&htim3);
    //HAL_TIM_Base_Start(&htim7);
#else
    MD1213_OE_Enable(0);
    stop_single_timer();
#endif
  } else {
    set_timing();
#if defined(CONFIG_MULTI)
    start_multi_timer(RF_CH0);
    GPIO_HV_Enable(1);
    // STHV748_THSD_Enable(DISABLE);
    // STHV748_THSD_Enable(ENABLE);
    HAL_TIM_Base_Start_IT(&htim6);
#else    
    MD1213_OE_Enable(0);
    start_single_timer();
    GPIO_HV_Enable(1);
#endif
  }
}


#define TIMER_BASC_CLK 60000000
int8_t setFrequncy(int32_t frq)
{
  uint32_t base_clk = TIMER_BASC_CLK/2;
  uint16_t scaler;
  
  scaler = base_clk/frq;

  if(scaler < 0xFFFF) {
#if defined(CONFIG_MULTI)
    TIM1->PSC = scaler-1;
    TIM8->PSC = scaler-1;
#else     
    TIM5->PSC = scaler-1;
#endif
    return 1;
  }

  return 0;
}


int32_t calcurateSD(int32_t sd,int32_t prp)
{
  int32_t new;
  if(prp == 0) return 0;
  new = (sd/prp)*prp;
  return new;
}

int32_t setTiming(int32_t SD,int32_t ISI,int32_t *pTD)
{
  int32_t bi ,td;
  td = *pTD;
  bi = SD + ISI;

  if(td < bi) {
    *pTD = bi; 
  }
  return bi;
}

int32_t setTBDAndDuty(int32_t tbd,int32_t duty)
{
  char str[64];
  int32_t PRP;

  // duty?�� 1 ~ 100 ?�� 값을 �??�?? ?�� ?��?��
  // tbd ?�� 0 ?�� 값을 �??질수 ?��?��

  //case 1 : tbd == 0 ?��경우 rpr ?�� 0 ?���?? ?��?��
  //case 2 : duty�?? 0 ?�� 경우 tbd �?? 0 �?? ?��?�� ?���?? 처리
  if(tbd == 0) {
    PRP = 0;
    return (PRP);
  }

  //case 3:  duty�?? 100 ?�� rpr ?�� 값�? ??

  // rpr = (tbd/duty)*100;
  // PRP = (int32_t)(tbd*(float)(100/duty));
  PRP = (int32_t)(tbd*(100/(float)duty));
  sprintf(str,"\r\nTBD %d, Duty %d, PRP %d(%0.1f)\r\n",tbd,duty, PRP, tbd*(100/(float)duty));
  INFO(str);
  return PRP;
}

void InituserTask03(CONFIG_T *sysconf)
{
  setSysTimeOut(UPDATE_TIMER,1);
}

void userTask03(CONFIG_T *sysconf)
{
  if(getSysTimeOut(UPDATE_TIMER) == 0)
  {
    if(sysconf->updateEnv != 0)
    {
      if(sysconf->updateEnv & (1 << CMD_setFrequency)) {
        sysconf->updateEnv &= ~(1 << CMD_setFrequency);
        sysconf->setFrequency = sysconf->n_setFrequency;

        if(setFrequncy(sysconf->setFrequency)) {

          nv_write_setFrequency(sysconf->setFrequency);
          DBG_INFO("\r\nUpdate Frequency\r\n");
          displayEnv(sysconf,CMD_getFrequency);
        }
      }

      if(sysconf->updateEnv & (1 << CMD_setOutputVoltage)) {
        sysconf->updateEnv &= ~(1 << CMD_setOutputVoltage);
        sysconf->setOutputVoltage = sysconf->n_setOutputVoltage;
        nv_write_setOutputVoltage(sysconf->setOutputVoltage);
        DBG_INFO("\r\nUpdate setOutputVoltage\r\n");
        displayEnv(sysconf,CMD_getOutputVoltage);
      }

      if(sysconf->updateEnv & (1 << CMD_setTBDAndDuty)) {
        sysconf->updateEnv &= ~(1 << CMD_setTBDAndDuty);
        sysconf->setTBD  = from100uSEC(sysconf->n_setTBD);
        sysconf->setDuty = sysconf->n_setDuty;

        sysconf->setPRP = setTBDAndDuty(sysconf->setTBD,sysconf->setDuty);

        nv_write_setTBD(sysconf->setTBD);
        nv_write_setDuty(sysconf->setDuty);
        nv_write_setPRP(sysconf->setPRP);

        DBG_INFO("\r\nUpdate setTBDAndDuty\r\n");
        displayEnv(sysconf,CMD_getTBDAndDuty);
      }

      if(sysconf->updateEnv & (1 << CMD_setTiming)) {
        int32_t a,b,c;
        sysconf->updateEnv &= ~(1 << CMD_setTiming);

        sysconf->setSD  = from1mSEC(sysconf->n_setSD);
        sysconf->setISI = from1sSEC(sysconf->n_setISI);
        sysconf->setTD  = from1sSEC(sysconf->n_setTD);

        sysconf->setBI = setTiming(sysconf->setSD,sysconf->setISI,&sysconf->setTD);
  
        a = calcurateSD(sysconf->setSD,sysconf->setPRP);

        b = sysconf->setSD - a;

        sysconf->setISI += b;
        sysconf->setSD   = a; 


        nv_write_setSD(sysconf->setSD);
        nv_write_setISI(sysconf->setISI);
        nv_write_setBI(sysconf->setBI);
        nv_write_setTD(sysconf->setTD);

        DBG_INFO("\r\nUpdate setTiming\r\n");
        displayEnv(sysconf,CMD_getTiming);

      }

      if(sysconf->updateEnv & (1 << CMD_setImpedance)) {
        sysconf->updateEnv &= ~(1 << CMD_setImpedance);

        sysconf->setImpedance = sysconf->n_setImpedance;
        nv_write_setImpedance(sysconf->setImpedance);

        DBG_INFO("\r\nUpdate setImpedance\r\n");
        displayEnv(sysconf,CMD_getImpedance);
      }

      if(sysconf->updateEnv & (1 << CMD_setAbnormalStopMode)) {
        sysconf->updateEnv &= ~(1 << CMD_setAbnormalStopMode);

        sysconf->setAbnormalStopMode = sysconf->n_setAbnormalStopMode;
        sysconf->setAbnormalStopMaxV = sysconf->n_setAbnormalStopMaxV;
        sysconf->setAbnormalStopMaxI = sysconf->n_setAbnormalStopMaxI;
        sysconf->setAbnormalStopMinV = sysconf->n_setAbnormalStopMinV;
        sysconf->setAbnormalStopMinI = sysconf->n_setAbnormalStopMinI;

        nv_write_setAbnormalStopMode(sysconf->setAbnormalStopMode);

        DBG_INFO("\r\nUpdate setAbnormalStopMode\r\n");
        displayEnv(sysconf,CMD_getAbnormalStopMode);
      }

      if(sysconf->updateEnv & (1 << CMD_sonication)) {
        sysconf->updateEnv &= ~(1 << CMD_sonication);
        sysconf->sonication = sysconf->n_sonication;
        DBG_INFO("\r\nUpdate sonication\r\n");
        INFO("\r\nsonication [%s]\r\n",sysconf->sonication == 1? "start":"stop");
        setSonication(sysconf->sonication);
      }
      
      if(sysconf->updateEnv & (1 << CMD_setDigipot)) {
        uint8_t val = 0;
        HAL_StatusTypeDef ret;
        sysconf->updateEnv &= ~(1 << CMD_setDigipot);
        
        ret = setDigpo(sysconf->digipotCH,sysconf->setDigipot,&val);
        
        if(ret == HAL_OK)
        {
          DBG_INFO("\r\nUpdate Digital Potentiometer \r\n");
          INFO("\r\nsetDigpo %d[%d]\r\n",sysconf->setDigipot,val);
        }
        else if(ret == HAL_ERROR)
        {
          DBG_ERROR("\r\nsetDigpo Error [ERROR]\r\n");
        }
        else if(ret == HAL_BUSY) {
          DBG_ERROR("\r\nsetDigpo Error [BUSY] \r\n");
        }
        else if(ret == HAL_TIMEOUT) {
          DBG_ERROR("\r\nsetDigpo Error [TIMEOUT]\r\n");
        }
      }
    }
    
    setSysTimeOut(UPDATE_TIMER,10);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
#if defined(DEBUG_UART3)
  if(huart->Instance == USART3 || huart->Instance == USART1)
#elif defined(DEBUG_UART2)
  if(huart->Instance == USART2 || huart->Instance == USART1)
#else
  if(huart->Instance == USART1)
#endif 
  {
    char  ch = (char)UARTRxBuff[UARTRxBuffIdx];
    switch (ch)
    {
      case 0x0A: //'\n'
       break;
      case 0x0D: //'\r'
      //case 0x0A: //'\n'
       CONFIG_MODE(3000);
       if(UARTRxBuffIdx != 0) {
         BUFFER_Write(&USART_Buffer,UARTRxBuff,UARTRxBuffIdx+1); 
         UARTRxBuffIdx = 0;
       } else {
         BUFFER_Write(&USART_Buffer,(uint8_t *)&ch,1); 
       }
       break;
      case 0x08: // '\b'    
      case 0x7F:  // DEL
        if (UARTRxBuffIdx > 0)
        {
            xputs("\033[1D");
            xputs("\033[K");
            UARTRxBuffIdx--;
        }
        else
            xputc(0x07);
        break;
//    case 0x1B:  // ESC
//    case 0x09:  // HT (horizontal tab)
//    case 0x0B:  // VT (vertical tab)
//    case 0x0C:  // FF (NP form feed, new page)
//        break;
     default:
      xputc(ch);
      UARTRxBuffIdx++;
      break;
    }
    
#if defined(DEBUG_UART3)
  HAL_UART_Receive_IT(&huart1,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);
  HAL_UART_Receive_IT(&huart3,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);
#elif defined(DEBUG_UART2)
  HAL_UART_Receive_IT(&huart1,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);
  HAL_UART_Receive_IT(&huart2,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);  
#else
  HAL_UART_Receive_IT(&huart1,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);  
#endif    
  }
}

void HAL_SYSTICK_Callback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SYSTICK_Callback could be implemented in the user file
   */
  static uint16_t tick = 0;

  for(int i = 0; i < MAX_TIMER; i++) {
    if(systemTimer[i] > 0)
      systemTimer[i]--;
  }

  if(gConfig.timeout > 0) {
    gConfig.timeout--;
    if(gConfig.timeout == 0)
      gConfig.state = 0;
  }


  if(tick++ > 500)
  {
    tick = 0;
    HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
    
  }
}

#if !defined(CONFIG_MULTI)
static void single_RF_generate(void)
{
  static int32_t  preTD = 0;
  
  if(gTD[RF_CH0]--) {
      //BI cycle
      if((gBI[RF_CH0]--) > 0) {
        if((gSD[RF_CH0]--) > 0) { 
          if((gPRP[RF_CH0]--) > 0) 
          { 
            if((gTBD[RF_CH0]--) > 0)
              MD1213_OE_GPIO_Port->BSRR  = MD1213_OE_Pin;
              //MD1213_OE_Enable(1); 
            else  
              MD1213_OE_GPIO_Port->BSRR = (uint32_t)MD1213_OE_Pin << 16U;
              //MD1213_OE_Enable(0);

            if(gPRP[RF_CH0] == 0) {
               gTBD[RF_CH0] = gConfig.setTBD; 
               gPRP[RF_CH0] = gConfig.setPRP;
             }  
          } 
        } 
        else 
        { 
          MD1213_OE_GPIO_Port->BSRR = (uint32_t)MD1213_OE_Pin << 16U;
          //MD1213_OE_Enable(0);
        }

        if(gBI[RF_CH0] == 0) {
          gSD[RF_CH0] = gConfig.setSD;
          gBI[RF_CH0] = gConfig.setBI;
          gTBD[RF_CH0] = gConfig.setTBD;
          gPRP[RF_CH0] = gConfig.setPRP;
        }
      }
    } else {
      gTBD[RF_CH0] = -1;
      gPRP[RF_CH0] = -1;
      gSD[RF_CH0]  = -1;
      gISI[RF_CH0] = -1;
      gBI[RF_CH0]  = -1;
      gTD[RF_CH0]  = -1;

      if(gConfig.sonication) {
        gConfig.n_sonication = 0;
        gConfig.updateEnv |= (1 << CMD_sonication);
      }
      MD1213_OE_Enable(0);
      HAL_TIM_Base_Stop(&htim6);
    }
}
#endif

#if defined(CONFIG_MULTI)
static void multi_RF_generate(void)
{
  static int32_t  preTD = 0;

   if(gTD[RF_CH1]--) {
      //BI cycle
      if((gBI[RF_CH1]--) > 0) {
        if((gSD[RF_CH1]--) > 0) { 
          if((gPRP[RF_CH1]--) > 0) {
             if((gTBD[RF_CH1]--) > 0) {
              STHV748_THSD_GPIO_Port->BSRR  = STHV748_THSD_Pin;
              //start_multi_timer(RF_CH0);
             } else {
              STHV748_THSD_GPIO_Port->BSRR = (uint32_t)STHV748_THSD_Pin << 16U;
              //stop_multi_timer();
             }

             //  
             if(gPRP[RF_CH1] == 0) {
               gTBD[RF_CH1] = gConfig.setTBD; 
               gPRP[RF_CH1] = gConfig.setPRP;
             }
          } 
        } 
        else 
        {
          STHV748_THSD_GPIO_Port->BSRR = (uint32_t)STHV748_THSD_Pin << 16U;
          //stop_multi_timer();
        }

        if(gBI[RF_CH1] == 0) {
          gSD[RF_CH1] = gConfig.setSD;
          gBI[RF_CH1] = gConfig.setBI;
          gTBD[RF_CH1] = gConfig.setTBD;
          gPRP[RF_CH1] = gConfig.setPRP;
        }
      }
    } else {
      gTBD[RF_CH1] = -1;
      gPRP[RF_CH1] = -1;
      gSD [RF_CH1] = -1;
      gISI[RF_CH1] = -1;
      gBI [RF_CH1] = -1;
      gTD [RF_CH1] = -1;

      if(gConfig.sonication) {
        gConfig.n_sonication = 0;
        gConfig.updateEnv |= (1 << CMD_sonication);
      }
      
      //stop_multi_timer();
      
      HAL_TIM_Base_Stop(&htim6);
    }
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  
  if(htim->Instance==TIM6)
  {
    if(RF_Start == 0) {
      RF_Start = 1;
      RF_PSC = TIM6->PSC;
      TIM6->PSC = 0;
    } else {
#if defined(CONFIG_MULTI)
    multi_RF_generate();
#else
    single_RF_generate();
#endif
    }
  }

  if(htim->Instance == TIM7) {
    DEBUG_EvtHandler();
    KeyScan();
    // Repeat_key_Scan();
    
  }

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
