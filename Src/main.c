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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "buffer.h"
#include "ringbuffer.h"
#include "xprintf.h"
#include "soft_timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define DBG_PRINT(...)    xprintf(__VA_ARGS__)
#define DBG_INFO(...)     xprintf("\x1B[32m"__VA_ARGS__"\x1B[0m")
#define DBG_WARRING(...)  xprintf("\033[33m"__VA_ARGS__"\033[0m")
#define DBG_ERROR(...)    xprintf("\033[31m"__VA_ARGS__"\033[0m")

#define DBG_INFO_MAG(...)    xprintf("\033[35m"__VA_ARGS__"\033[0m")
#define DBG_INFO_CYN(...)    xprintf("\033[36m"__VA_ARGS__"\033[0m")


#define INFO(fmt, ...)    xprintf("\033[33m");\
                          xprintf((fmt),##__VA_ARGS__);\
                          xprintf("\033[0m");
                          
//#define INFO_STR(...)     xprintf("\033[33m"__VA_ARGS__"\033[0m")
//#define INFO(fmt,...)     xprintf(fmt,"\033[33m"__VA_ARGS__"\033[0m")
#define INFO_PRINT(fmt,arg)     xprintf(fmt,##arg)

#define USEC100  (1000*10)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CONFIG_T gConfig;
uint8_t  UARTRxBuffIdx;
uint8_t  UARTRxBuff[128];
char     Received[256];
BUFFER_t USART1_Buffer;
uint8_t  USART1Buffer[RX_RING_SIZE];

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
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

void InituserTask02(void);
void userTask02(void);

void InituserTask03(void);
void userTask03(void);

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
  if(enable)
    HAL_GPIO_WritePin(GPIOC, EN_HV_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOC, EN_HV_Pin, GPIO_PIN_RESET);
  
  
  if(enable)
    HAL_GPIO_WritePin(GPIOB, BATT_V_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, BATT_V_Pin, GPIO_PIN_RESET);
  
}


void MD1213_OE_Enable(uint8_t enable)
{
  if(enable)
    HAL_GPIO_WritePin(GPIOB, MD1213_OE_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, MD1213_OE_Pin, GPIO_PIN_RESET);

}


void AD5170_WriteValue(uint8_t data)
{
  uint8_t buf[2];
  
  buf[0] = 0x00;
  buf[1] = data;
  HAL_I2C_Master_Transmit(&hi2c1,0x58,buf,2,100);
  //HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
}


void calcuarateEnv(CONFIG_T *env, Timer_T *t)
{

}
void Load_Env(CONFIG_T *env)
{
  if(env != NULL)
  {
    env->setFrequency = 250000;
    env->setOutputVoltage = 50;

    env->setTBD  =  500; // 25.0ms  range 0.1 ~ 50ms
    env->setDuty =  50;   // 50 %    range 1 ~ 100 %
    env->setPRP  = (int32_t)(env->setTBD/env->setDuty)*100;

    env->setSD  =  env->setPRP*3;      // ms
    env->setISI =  env->setSD*4;  // s
    env->setBI  = env->setSD + env->setISI;    // s 
    env->setTD  = 50*1000*10;    // s 

    env->setImpedance = 50;

    env->setAbnormalStopMode = 0;
    env->setAbnormalStopMaxV = 50;
    env->setAbnormalStopMaxI = 50;
    env->setAbnormalStopMinV = 50;
    env->setAbnormalStopMinI = 50;

    env->sonication = 0;

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
  uint8_t resis = 0;

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
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  InitSysTimeOut();
  DEBUG_LL_USARTInit(&huart1);
  BUFFER_Init(&USART1_Buffer, RX_RING_SIZE, USART1Buffer);
  
  
  //HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  //HAL_TIM_Base_Start_IT(&htim6);
  MD1213_OE_Enable(0);
  GPIO_HV_Enable(0);
  //GPIO_HV_Enable(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  DBG_INFO("\r\n\r\nNEUROSONA NS-US3000 Single RF ES B'rd");
  DBG_INFO("\r\nBuild Date  %s %s",__DATE__,__TIME__);
  DBG_INFO("\r\nFW  Version  0.1a\r\n");
  DBG_INFO_MAG("NEUROSONA Co., Ltd. ");
  DBG_INFO_CYN("www.neurosona.com\r\n");
  DBG_PRINT("\r\nNS-US3000 $");
  
  UARTRxBuffIdx = 0;
  HAL_UART_Receive_IT(&huart1,&UARTRxBuff[UARTRxBuffIdx], 1);
  
  Load_Env(&gConfig);
  InituserTask02();
  InituserTask03();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    userTask02();
    userTask03();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 240;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim5.Init.Prescaler = 120-1;
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
  htim6.Init.Prescaler = 59;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
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
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_HV_Pin|STH748_A_IN3_Pin|STH748_B_IN3_Pin|STH748_D_IN3_Pin 
                          |STH748_IN4_Pin|LED1_Pin|LED2_Pin|AUDIO_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLE_RESET_Pin|STH748_C_IN3_Pin|MD1213_OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : EN_HV_Pin */
  GPIO_InitStruct.Pin = EN_HV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_HV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STH748_A_IN3_Pin STH748_B_IN3_Pin STH748_D_IN3_Pin STH748_IN4_Pin 
                           LED1_Pin LED2_Pin AUDIO_RESET_Pin */
  GPIO_InitStruct.Pin = STH748_A_IN3_Pin|STH748_B_IN3_Pin|STH748_D_IN3_Pin|STH748_IN4_Pin 
                          |LED1_Pin|LED2_Pin|AUDIO_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_RESET_Pin STH748_C_IN3_Pin MD1213_OE_Pin */
  GPIO_InitStruct.Pin = BLE_RESET_Pin|STH748_C_IN3_Pin|MD1213_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : T_Pin */
  GPIO_InitStruct.Pin = T_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AUDIO_BUSY_Pin */
  GPIO_InitStruct.Pin = AUDIO_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_BUSY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#define CHARISNUM(x)    ((x) >= '0' && (x) <= '9')
#define CHAR2NUM(x)     ((x) - '0')

static int32_t ParseNumber(char* ptr, uint8_t* cnt) {
	uint8_t minus = 0;
	int32_t sum = 0;
	uint8_t i = 0;
	
	/* Check for minus character */
	if (*ptr == '-') {
		minus = 1;
		ptr++;
		i++;
	}
	
	/* Parse number */
	while (CHARISNUM(*ptr)) {
		sum = 10 * sum + CHAR2NUM(*ptr);
		ptr++;
		i++;
	}
	
	/* Save number of characters used for number */
	if (cnt != NULL) {
		*cnt = i;
	}
	
	/* Minus detected */
	if (minus) {
		return 0 - sum;
	}
	
	/* Return number */
	return sum;
}
static void checkCmdArgs(COMMAMD_ID id, uint8_t num)
{
  switch(id) {
   case CMD_setFrequency:
    if(num == 3) {
      gConfig.updateEnv |= (1 << CMD_setFrequency) ;
      xprintf("\r\nsetFrequency OK");
    } else {
      gConfig.n_setFrequency = gConfig.setFrequency;
      if(num > 3)
        DBG_ERROR("\r\nsetFrequency too many arguments");
      else
        DBG_ERROR("\r\nsetFrequency too few arguments");
    }
    break;
   case CMD_setOutputVoltage:
    if(num == 3) {
      gConfig.updateEnv |= (1 << CMD_setOutputVoltage) ;
      xprintf("\r\nsetOutputVoltage OK");
    } else {
      gConfig.n_setOutputVoltage = gConfig.setOutputVoltage;
      if(num > 3)
        DBG_ERROR("\r\nsetOutputVoltage too many arguments");
      else
        DBG_ERROR("\r\nsetOutputVoltage too few arguments");
    }
    break;
   case CMD_setTBDAndDuty:
    if(num == 4) {
      gConfig.updateEnv |= (1 << CMD_setTBDAndDuty) ;
      xprintf("\r\nsetTBDAndDuty OK");
    } else {
      gConfig.n_setTBD  =  gConfig.setTBD;
      gConfig.n_setDuty = gConfig.setDuty;
      if(num > 4)
        DBG_ERROR("\r\nsetTBDAndDuty too many arguments");
      else
        DBG_ERROR("\r\nsetTBDAndDuty too few arguments");
    }
    break;
   case CMD_setTiming:
    if(num == 5) {
      gConfig.updateEnv |= (1 << CMD_setTiming) ;
      xprintf("\r\nsetTiming OK");
    } else {
      gConfig.n_setSD  = gConfig.setSD;
      gConfig.n_setISI = gConfig.setISI;
      gConfig.n_setTD  = gConfig.setTD;
      if(num > 5)
        DBG_ERROR("\r\nsetTiming too many arguments");
      else
        DBG_ERROR("\r\nsetTiming too few arguments");
    }
    break;
   case CMD_setImpedance:
    if(num == 3) {
      gConfig.updateEnv |= (1 << CMD_setImpedance) ;
      xprintf("\r\nsetImpedance OK");
    } else {
      gConfig.n_setImpedance = gConfig.setImpedance;
      if(num > 3)
        DBG_ERROR("\r\nsetImpedance too many arguments");
      else
        DBG_ERROR("\r\nsetImpedance too few arguments");
    }
    break;
   case CMD_setAbnormalStopMode:
    if(num == 7) {
      gConfig.updateEnv |= (1 << CMD_setAbnormalStopMode) ;
      xprintf("\r\nsetAbnormalStopMode OK");
    } else {
      gConfig.n_setAbnormalStopMode = gConfig.setAbnormalStopMode;
      gConfig.n_setAbnormalStopMaxI = gConfig.setAbnormalStopMaxI;
      gConfig.n_setAbnormalStopMaxV = gConfig.setAbnormalStopMaxV;
      gConfig.n_setAbnormalStopMinV = gConfig.setAbnormalStopMinV;
      gConfig.n_setAbnormalStopMinI = gConfig.setAbnormalStopMinI;

      if(num > 7)
        DBG_ERROR("\r\nsetAbnormalStopMode too many arguments");
      else
        DBG_ERROR("\r\nsetAbnormalStopMode too few arguments");
    }
    break;
   case CMD_sonication:
    if(num == 2) {
      INFO("\r\nsonication [%s]\r\n",gConfig.sonication == 1? "start":"stop");
    } else if(num == 3) {
      gConfig.updateEnv |= (1 << CMD_sonication) ;
      xprintf("\r\nsonication OK");
      //xprintf("\r\n %s sonication",gConfig.n_sonication == 1? "start":"stop");
    } else {
      gConfig.n_sonication = gConfig.sonication;
      DBG_ERROR("\r\nsonication too many arguments");
    }
    break;
   case CMD_clearError:
      xprintf("\r\nclearError OK");
    break;
  }
}


static void displayEnv(COMMAMD_ID id)
{
  switch(id) {
   case CMD_VERSION:
      INFO("\r\nVersion 0.1\r\n");
      break; 
   case CMD_getFrequency:
      INFO("\r\nFrequency = %d Hz\r\n",gConfig.setFrequency);
      break;
   case CMD_getOutputVoltage:
      INFO("\r\nOutputVoltage = %d V\r\n",gConfig.setOutputVoltage);
      INFO("\r\n");
      break;
   case CMD_getTBDAndDuty:
      INFO("\r\nTBD  = %d ms ",    gConfig.setTBD/10);
      INFO("\r\nDuty = %d %% \r\n",gConfig.setDuty);
      INFO("\r\n");
      break;
   case CMD_getTiming:
      INFO("\r\nSD  = %d ms ",gConfig.setSD/10);
      INFO("\r\nISI = %d s",  gConfig.setISI/USEC100);
      INFO("\r\nBI  = %d s",  gConfig.setBI/USEC100);
      INFO("\r\nTD  = %d s\r\n",gConfig.setTD/USEC100);
      break;
   case CMD_getImpedance:
      INFO("\r\nImpedance  = %d \r\n",gConfig.setImpedance);
      break;
   case CMD_getAbnormalStopMode:
      INFO("\r\nAbnormal[%s]",gConfig.setAbnormalStopMode == 0? "Off":"On");
      INFO("\r\nMax Volt  = %d ",gConfig.setAbnormalStopMaxV);
      INFO("\r\nmin Volt   = %d ",gConfig.setAbnormalStopMinV);
      INFO("\r\nMax Current  = %d ",gConfig.setAbnormalStopMaxI);
      INFO("\r\nmin Current  = %d \r\n",gConfig.setAbnormalStopMinI);
      break;
  }
}

void InituserTask02(void)
{
  setSysTimeOut(DEBUG_TIMER,1);
  HAL_TIM_Base_Start_IT(&htim7);
}

void userTask02(void)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  if(getSysTimeOut(DEBUG_TIMER) == 0)
  {
    uint16_t res ;
    COMMAMD_ID id;
    const char *text = "\r\nNS-US3000 $ ";
  
    if(gConfig.state != 0)
    {
      char* ptr;
      uint16_t num = 0;
      res = BUFFER_ReadString(&USART1_Buffer, Received, sizeof(Received));
      if(res > 0 ) {
        if(strstr(Received,"nsbt") != NULL) {

          /* Get token */
          ptr = strtok(Received," ");
          
          /* Do it until token != NULL */
          while (ptr != NULL) {
            /* Get positions */
            switch (num++) {
              case 0: 
                id = CMD_unknown;
                //xprintf("");
                break;
              case 1:
                /* Ignore first and last " */
                if(strstr (ptr,"version") != NULL) {
                  id = CMD_VERSION;
                  
                } else if(strstr(ptr,"setFrequency") != NULL)     {
                  id = CMD_setFrequency;
                } else if(strstr(ptr,"setOutputVoltage") != NULL) {
                  id = CMD_setOutputVoltage;
                } else if(strstr(ptr,"setTBDAndDuty")!= NULL)    {
                  id = CMD_setTBDAndDuty;
                } else if(strstr(ptr,"setTiming")!= NULL)         {
                  id = CMD_setTiming;
                } else if(strstr(ptr,"setImpedance")!= NULL)      {
                  id = CMD_setImpedance;
                } else if(strstr(ptr,"setAbnormalStopMode")!= NULL) {
                  id = CMD_setAbnormalStopMode;
                } else if(strstr(ptr,"sonication")!= NULL) {
                  id = CMD_sonication;
                } else if(strstr(ptr,"clearError")!= NULL) {
                  id = CMD_clearError;
                } else if(strstr(ptr,"getFrequency")!= NULL)     {
                  id = CMD_getFrequency;
                } else if(strstr(ptr,"getOutputVoltage")!= NULL) {
                  id = CMD_getOutputVoltage;
                } else if(strstr(ptr,"getTBDAndDuty")!= NULL)    {
                  id = CMD_getTBDAndDuty;
                } else if(strstr(ptr,"getTiming")!= NULL)         {
                  id = CMD_getTiming;
                } else if(strstr(ptr,"getImpedance")!= NULL)      {
                  id = CMD_getImpedance;
                } else if(strstr(ptr,"getAbnormalStopMode")!= NULL) {
                  id = CMD_getAbnormalStopMode;
                }
                break;
              case 2: 
                if(id == CMD_setFrequency) {
                  gConfig.n_setFrequency =  ParseNumber(ptr,NULL);
                } 
                else if(id == CMD_setOutputVoltage) {
                  gConfig.n_setOutputVoltage =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setTBDAndDuty) {
                  gConfig.n_setTBD =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setTiming) {
                  gConfig.n_setSD =  ParseNumber(ptr,NULL);
                  gConfig.n_setTD *= 10;
                }
                else if(id == CMD_setImpedance) {
                  gConfig.n_setImpedance =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setAbnormalStopMode) {
                  gConfig.n_setAbnormalStopMode =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_sonication) {
                  if(strstr(ptr,"start") != NULL) {
                    gConfig.n_sonication = 1;

                  } else if(strstr(ptr,"stop") != NULL) {
                    gConfig.n_sonication = 0;
                  }
                }
                break;
              case 3:
                if(id == CMD_setTBDAndDuty) {
                  gConfig.n_setDuty =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setTiming) {
                  gConfig.n_setISI =  ParseNumber(ptr,NULL);
                  gConfig.n_setISI *= USEC100;
                }
                else if(id == CMD_setAbnormalStopMode) {
                  gConfig.n_setAbnormalStopMaxV =  ParseNumber(ptr,NULL);
                }
                break;
              case 4: 
                if(id == CMD_setTiming) {
                  gConfig.n_setTD =  ParseNumber(ptr,NULL);
                  gConfig.n_setTD *= USEC100;
                }
                else if(id == CMD_setAbnormalStopMode) {
                  gConfig.n_setAbnormalStopMaxI =  ParseNumber(ptr,NULL);
                }
                break;
              case 5: 
                if(id == CMD_setAbnormalStopMode) {
                  gConfig.n_setAbnormalStopMinV =  ParseNumber(ptr,NULL);
                }
                break;
              case 6: 
                if(id == CMD_setAbnormalStopMode) {
                  gConfig.n_setAbnormalStopMinI =  ParseNumber(ptr,NULL);
                }
                break;
              default: break;
            }

            /* Get new token */
            ptr = strtok(NULL, " ");
          }
          //
          checkCmdArgs(id,num);
          displayEnv(id);
        }
        
        xprintf(text);
      }
    }

    setSysTimeOut(DEBUG_TIMER,1);
  }
  /* USER CODE END StartTask02 */
}

int32_t gTBD;
int32_t gRPR;
int32_t gSD;
int32_t gISI;
int32_t gBI;
int32_t gTD;

void setSonication(int32_t mode)
{
  if(mode  == 0) {
    gTBD = -1;
    gRPR = -1;
    gSD  = -1;
    gISI = -1;
    gBI  = -1;
    gTD  = -1;
    GPIO_HV_Enable(0);
    MD1213_OE_Enable(0);
    HAL_TIM_Base_Stop(&htim6);
    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
  } else {
    gTBD = gConfig.setTBD;
    gRPR = gConfig.setPRP;
    gSD  = gConfig.setSD;
    gISI = gConfig.setISI;
    gBI  = gConfig.setBI;
    gTD  = gConfig.setTD;

    MD1213_OE_Enable(0);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
    GPIO_HV_Enable(1);
    INFO("\r\ncycle (TD %d/%d)\r\n",(gTD/USEC100),(gConfig.setTD/USEC100));
  }


}

#define TIM5_BASC_CLK 60000000
int8_t setFrequncy(int32_t frq)
{
  uint32_t base_clk = TIM5_BASC_CLK/2;
  uint16_t scaler; 

  scaler = base_clk/frq;

  if(scaler < 0xFFFF) {
    TIM5->PSC = scaler-1;
    return 1;
  } 

  return 0;
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
  int32_t rpr;

  rpr = (tbd/duty)*100;

  return rpr;
}

void InituserTask03(void)
{
  setSysTimeOut(UPDATE_TIMER,1);

}

void userTask03(void)
{
  if(getSysTimeOut(UPDATE_TIMER) == 0)
  {
    if(gConfig.updateEnv != 0)
    {
      if(gConfig.updateEnv & (1 << CMD_setFrequency)) {
        gConfig.updateEnv &= ~(1 << CMD_setFrequency);
        gConfig.setFrequency = gConfig.n_setFrequency;

        if(setFrequncy(gConfig.setFrequency)) {
          DBG_INFO("\r\nUpdate Frequency\r\n");
          displayEnv(CMD_getFrequency);

        }
      }

      if(gConfig.updateEnv & (1 << CMD_setOutputVoltage)) {
        gConfig.updateEnv &= ~(1 << CMD_setOutputVoltage);
        gConfig.setOutputVoltage = gConfig.n_setOutputVoltage;
        DBG_INFO("\r\nUpdate setOutputVoltage\r\n");
        displayEnv(CMD_getOutputVoltage);
      }

      if(gConfig.updateEnv & (1 << CMD_setTBDAndDuty)) {
        gConfig.updateEnv &= ~(1 << CMD_setTBDAndDuty);
        gConfig.setTBD = gConfig.n_setTBD;
        gConfig.setDuty = gConfig.n_setDuty;

        gConfig.setPRP = setTBDAndDuty(gConfig.setTBD,gConfig.setDuty);

        DBG_INFO("\r\nUpdate setTBDAndDuty\r\n");
        displayEnv(CMD_getTBDAndDuty);
      }

      if(gConfig.updateEnv & (1 << CMD_setTiming)) {
        gConfig.updateEnv &= ~(1 << CMD_setTiming);

        gConfig.setSD = gConfig.n_setSD;
        gConfig.setISI = gConfig.n_setISI;
        gConfig.setTD = gConfig.n_setTD;

        gConfig.setBI = setTiming(gConfig.setSD,gConfig.setISI,&gConfig.setTD);

        DBG_INFO("\r\nUpdate setTiming\r\n");
        displayEnv(CMD_getTiming);

      }

      if(gConfig.updateEnv & (1 << CMD_setImpedance)) {
        gConfig.updateEnv &= ~(1 << CMD_setImpedance);

        gConfig.setImpedance = gConfig.n_setImpedance;

        DBG_INFO("\r\nUpdate setImpedance\r\n");
        displayEnv(CMD_getImpedance);
      }

      if(gConfig.updateEnv & (1 << CMD_setAbnormalStopMode)) {
        gConfig.updateEnv &= ~(1 << CMD_setAbnormalStopMode);

        gConfig.setAbnormalStopMode = gConfig.n_setAbnormalStopMode;
        gConfig.setAbnormalStopMaxV = gConfig.n_setAbnormalStopMaxV;
        gConfig.setAbnormalStopMaxI = gConfig.n_setAbnormalStopMaxI;
        gConfig.setAbnormalStopMinV = gConfig.n_setAbnormalStopMinV;
        gConfig.setAbnormalStopMinI = gConfig.n_setAbnormalStopMinI;

        DBG_INFO("\r\nUpdate setAbnormalStopMode\r\n");
        displayEnv(CMD_getAbnormalStopMode);
      }

      if(gConfig.updateEnv & (1 << CMD_sonication)) {
        gConfig.updateEnv &= ~(1 << CMD_sonication);
        gConfig.sonication = gConfig.n_sonication;
        DBG_INFO("\r\nUpdate sonication\r\n");
        INFO("\r\nsonication [%s]\r\n",gConfig.sonication == 1? "start":"stop");
        setSonication(gConfig.sonication);
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
  if(huart->Instance == USART1)
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
         BUFFER_Write(&USART1_Buffer,UARTRxBuff,UARTRxBuffIdx+1); 
         UARTRxBuffIdx = 0;
       } else {
         BUFFER_Write(&USART1_Buffer,&ch,1); 
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
    HAL_UART_Receive_IT(&huart1,(uint8_t*)&UARTRxBuff[UARTRxBuffIdx],1);  
  }
}

void HAL_SYSTICK_Callback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SYSTICK_Callback could be implemented in the user file
   */
  for(int i = 0; i < MAX_TIMER; i++)
  {
    if(systemTimer[i] > 0)
      systemTimer[i]--;
  }

  if(gConfig.timeout > 0) {
    gConfig.timeout--;
    if(gConfig.timeout == 0) {
      gConfig.state = 0;
    }
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  
  if(htim->Instance==TIM6)
  {
    if(gTD--) {
      if((gBI--) > 0) {
        if((gSD--) > 0) {
          
          if((gRPR--) > 0) {
            if((gTBD--) > 0) {
              MD1213_OE_Enable(1);
            } else {
              MD1213_OE_Enable(0);
            }
          } else {
            gTBD = gConfig.setTBD;
            gRPR = gConfig.setPRP;
          }
          // gSD = 100;
          // gBI = 100;
          // gTD = 100;
          
          
        }
      } else {
        gSD = gConfig.setSD;
        gBI = gConfig.setBI;
        INFO("1 cycle done (TD %d/%d)\r\n",(gTD/USEC100),(gConfig.setTD/USEC100));
      }
    } else {
      gTBD = -1;
      gRPR = -1;
      gSD  = -1;
      gISI = -1;
      gBI  = -1;
      gTD  = -1;

      if(gConfig.sonication) {
        gConfig.n_sonication = 0;
        gConfig.updateEnv |= (1 << CMD_sonication);
      }
      MD1213_OE_Enable(0);
      HAL_TIM_Base_Stop(&htim6);
    }
  }

  if(htim->Instance == TIM7) {
    
    DEBUG_EvtHandler();
  
    // KeyScan();
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
