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
#define RF_CH0          0  
#define RF_CH1          1
#define RF_CH2          2
#define RF_CH3          3
#define RF_CH4          4
#define RF_CH5          5
#define RF_CH6          6

  
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
  int32_t    n_setDelay[MAX_RF_CH];
  int32_t    n_setAbnormalStopMode;

  int32_t    n_setAbnormalStopMaxV;
  int32_t    n_setAbnormalStopMaxI;
  int32_t    n_setAbnormalStopMinV;
  int32_t    n_setAbnormalStopMinI;

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
  int32_t    setDelay[MAX_RF_CH];
  int32_t    setAbnormalStopMode;

  int32_t    setAbnormalStopMaxV;
  int32_t    setAbnormalStopMaxI;
  int32_t    setAbnormalStopMinV;
  int32_t    setAbnormalStopMinI;

  int32_t    sonication;
  
  uint8_t    digipotCH;
  uint8_t    setDigipot;

} CONFIG_T;

typedef struct _timer_t {
  int32_t    sonication;
} Timer_T;

#if 1
typedef enum _COMMAMD_ID {
    
    CMD_setFrequency = 0,
    CMD_setOutputVoltage,
    CMD_setTBDAndDuty, 
    CMD_setTiming,
    CMD_setImpedance, 
    CMD_setAbnormalStopMode,
    CMD_setDelay,
    CMD_sonication,
    CMD_setDigipot,
    CMD_clearError,
    CMD_getFrequency,
    CMD_getOutputVoltage,
    CMD_getTBDAndDuty, 
    CMD_getTiming,
    CMD_getImpedance, 
    CMD_getAbnormalStopMode,
    CMD_getDelay,
    CMD_VERSION,
    CMD_HELP,
    CMD_unknown
} COMMAMD_ID;
#else
typedef enum _COMMAMD_ID {
    
    CMD_setFrequency = 0,
    CMD_setOutputVoltage = 1,
    CMD_setTBDAndDuty = 2, 
    CMD_setTiming = 3,
    CMD_setImpedance = 4, 
    CMD_setAbnormalStopMode = 5,
    
    
    CMD_setActiveCH  = 6,
    CMD_setCurrentCH = 7,
    
    CMD_sonication = 8,
    
    
    
    CMD_setDigipot = 9,
    
    
    
    CMD_clearError = 8,
    CMD_getFrequency = 9,
    CMD_getOutputVoltage = 10,
    CMD_getTBDAndDuty = 11, 
    CMD_getTiming = 12,
    CMD_getImpedance = 13, 
    CMD_getAbnormalStopMode = 14,
    
    
    
    CMD_VERSION = 15,
    CMD_HELP    = 16,
    CMD_unknown = 16
} COMMAMD_ID;
#endif
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
