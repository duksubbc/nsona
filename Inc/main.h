/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define MAX_RF_CH       7

typedef struct _config_t {
  uint8_t    state;
  uint8_t    flag;
  int32_t    timeout;
  int32_t    version;
  uint8_t    configCH;
  uint8_t    ActiveCH;
  uint32_t   updateEnv;
  int32_t    n_setFrequency;
  int32_t    n_setOutputVoltage;
  int32_t    n_setTBD;
  int32_t    n_setDuty;
  int32_t    n_setSD;
  int32_t    n_setISI;
  int32_t    n_setTD;
  int32_t    n_setImpedance;
  int32_t    n_setAbnormalStopMode;
  int32_t    n_setAbnormalStopMaxV;
  int32_t    n_setAbnormalStopMaxI;
  int32_t    n_setAbnormalStopMinV;
  int32_t    n_setAbnormalStopMinI;
  int32_t    n_setDelay[MAX_RF_CH];
  int32_t    n_sonication;
  int32_t    setFrequency;
  int32_t    setOutputVoltage;
  int32_t    setTBD;
  int32_t    setDuty;
  int32_t    setPRP;
  int32_t    setSD;
  int32_t    setISI;
  int32_t    setBI;
  int32_t    setTD;
  int32_t    setImpedance;
  int32_t    setAbnormalStopMode;
  int32_t    setAbnormalStopMaxV;
  int32_t    setAbnormalStopMaxI;
  int32_t    setAbnormalStopMinV;
  int32_t    setAbnormalStopMinI;
  int32_t    setDelay[MAX_RF_CH];
  int32_t    sonication;
  uint8_t    digipotCH;
  uint8_t    setDigipot;
} CONFIG_T;

typedef struct _timer_t {
  int32_t    sonication;
} Timer_T;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define AUDIO_CS_LOW()          (AUDIO_CS_GPIO_Port->BSRR = AUDIO_CS_Pin << 16)
#define AUDIO_CS_HI()           (AUDIO_CS_GPIO_Port->BSRR = AUDIO_CS_Pin)

#define AUDIO_RESET_LOW()       (AUDIO_RESET_GPIO_Port->BSRR = AUDIO_RESET_Pin << 16)
#define AUDIO_RESET_HI()        (AUDIO_RESET_GPIO_Port->BSRR = AUDIO_RESET_Pin)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern HAL_StatusTypeDef setDigpo(uint8_t ch , uint8_t val, uint8_t *reg);
extern void LED1_ON(uint8_t on);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_HV_Pin GPIO_PIN_13
#define EN_HV_GPIO_Port GPIOC
#define EX_RF_1A_Pin GPIO_PIN_0
#define EX_RF_1A_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_1
#define SW1_GPIO_Port GPIOC
#define EX_RF_3A_Pin GPIO_PIN_2
#define EX_RF_3A_GPIO_Port GPIOC
#define EX_RF_4A_Pin GPIO_PIN_3
#define EX_RF_4A_GPIO_Port GPIOC
#define MD1213_PULSE_A_Pin GPIO_PIN_0
#define MD1213_PULSE_A_GPIO_Port GPIOA
#define MD1213_PULSE_B_Pin GPIO_PIN_1
#define MD1213_PULSE_B_GPIO_Port GPIOA
#define RF_V_Pin GPIO_PIN_4
#define RF_V_GPIO_Port GPIOA
#define STHV748_D_IN2_Pin GPIO_PIN_5
#define STHV748_D_IN2_GPIO_Port GPIOA
#define STHV748_A_IN2_Pin GPIO_PIN_7
#define STHV748_A_IN2_GPIO_Port GPIOA
#define STHV748_A_IN3_Pin GPIO_PIN_4
#define STHV748_A_IN3_GPIO_Port GPIOC
#define STHV748_B_IN3_Pin GPIO_PIN_5
#define STHV748_B_IN3_GPIO_Port GPIOC
#define STHV748_B_IN2_Pin GPIO_PIN_0
#define STHV748_B_IN2_GPIO_Port GPIOB
#define BATT_V_Pin GPIO_PIN_1
#define BATT_V_GPIO_Port GPIOB
#define BLE_RESET_Pin GPIO_PIN_2
#define BLE_RESET_GPIO_Port GPIOB
#define BLE_TX_Pin GPIO_PIN_10
#define BLE_TX_GPIO_Port GPIOB
#define BLE_RX_Pin GPIO_PIN_11
#define BLE_RX_GPIO_Port GPIOB
#define STHV748_C_IN3_Pin GPIO_PIN_12
#define STHV748_C_IN3_GPIO_Port GPIOB
#define BLE_CTX_Pin GPIO_PIN_13
#define BLE_CTX_GPIO_Port GPIOB
#define BLE_RTS_Pin GPIO_PIN_14
#define BLE_RTS_GPIO_Port GPIOB
#define STHV748_C_IN2_Pin GPIO_PIN_15
#define STHV748_C_IN2_GPIO_Port GPIOB
#define STHV748_D_IN1_Pin GPIO_PIN_6
#define STHV748_D_IN1_GPIO_Port GPIOC
#define STHV748_D_IN3_Pin GPIO_PIN_7
#define STHV748_D_IN3_GPIO_Port GPIOC
#define STHV748_IN4_Pin GPIO_PIN_8
#define STHV748_IN4_GPIO_Port GPIOC
#define STHV748_THSD_Pin GPIO_PIN_9
#define STHV748_THSD_GPIO_Port GPIOC
#define STHV748_A_IN1_Pin GPIO_PIN_8
#define STHV748_A_IN1_GPIO_Port GPIOA
#define STHV748_B_IN1_Pin GPIO_PIN_9
#define STHV748_B_IN1_GPIO_Port GPIOA
#define STHV748_C_IN1_Pin GPIO_PIN_10
#define STHV748_C_IN1_GPIO_Port GPIOA
#define AUDIO_CS_Pin GPIO_PIN_15
#define AUDIO_CS_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC
#define AUDIO_RESET_Pin GPIO_PIN_12
#define AUDIO_RESET_GPIO_Port GPIOC
#define AUDIO_BUSY_Pin GPIO_PIN_2
#define AUDIO_BUSY_GPIO_Port GPIOD
#define SPI_SCK_Pin GPIO_PIN_3
#define SPI_SCK_GPIO_Port GPIOB
#define MD1213_OE_Pin GPIO_PIN_4
#define MD1213_OE_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_5
#define SPI_MOSI_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define _VERSION_     "0.2a"

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define DBG_PRINT(...)      xprintf(__VA_ARGS__)
#define DBG_INFO(...)       xprintf("\x1B[32m"__VA_ARGS__"\x1B[0m")
#define DBG_WARRING(...)    xprintf("\033[33m"__VA_ARGS__"\033[0m")
#define DBG_ERROR(...)      xprintf("\033[31m"__VA_ARGS__"\033[0m")

#define DBG_INFO_MAG(...)   xprintf("\033[35m"__VA_ARGS__"\033[0m")
#define DBG_INFO_CYN(...)   xprintf("\033[36m"__VA_ARGS__"\033[0m")


#define INFO(fmt, ...)      xprintf("\033[32m");\
                            xprintf((fmt),##__VA_ARGS__);\
                            xprintf("\033[0m");

#define WARRING(fmt, ...)   xprintf("\033[33m");\
                            xprintf((fmt),##__VA_ARGS__);\
                            xprintf("\033[0m");

#define ERROR(fmt, ...)     xprintf("\033[31m");\
                            xprintf((fmt),##__VA_ARGS__);\
                            xprintf("\033[0m");
                            
//#define INFO_STR(...)     xprintf("\033[33m"__VA_ARGS__"\033[0m")
//#define INFO(fmt,...)     xprintf(fmt,"\033[33m"__VA_ARGS__"\033[0m")
#define INFO_PRINT(fmt,arg) xprintf(fmt,##arg)



                            
#define USEC100  (1)
#define SEC      (250000)
//#define SEC      (1000000)       
//#define SEC      (100000)

#define to1mSEC(x)      (x/250)
#define to100uSEC(x)    (x/25)
#define to1sSEC(x)      (x/250000)


#define from1mSEC(x)      (x*250)
#define from100uSEC(x)    (x*25)
#define from1sSEC(x)      (x*250000)


#define RF_CH0          0  
#define RF_CH1          1
#define RF_CH2          2
#define RF_CH3          3
#define RF_CH4          4
#define RF_CH5          5
#define RF_CH6          6

#define RF_DEF_FREQUENCY    250000
#define RF_DEF_OUTVOLT      10
#define RF_DEF_TBD          from100uSEC(1)
#define RF_DEF_DUTY         50
#define RF_DEF_RPR          from100uSEC(2)

#define RF_DEF_SD           from1mSEC(1)
#define RF_DEF_ISI          from1sSEC(1)
#define RF_DEF_BI           from1sSEC(1)
#define RF_DEF_TD           from1sSEC(10)

#define RF_DEF_IMPEDANCE    50
#define RF_DEF_ABNORMAL_MODE    0

#define RF_DEF_ABNORMAL_MAXV    0
#define RF_DEF_ABNORMAL_MINV    0
#define RF_DEF_ABNORMAL_MAXI    0
#define RF_DEF_ABNORMAL_MINI    0

#define RF_DEF_DELAY_CH0        from100uSEC(1)
#define RF_DEF_DELAY_CH1        from100uSEC(1)
#define RF_DEF_DELAY_CH2        from100uSEC(1)
#define RF_DEF_DELAY_CH3        from100uSEC(1)
#define RF_DEF_DELAY_CH4        from100uSEC(1)
#define RF_DEF_DELAY_CH5        from100uSEC(1)
#define RF_DEF_DELAY_CH6        from100uSEC(1)

extern uint8_t  console_debug ;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
