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
typedef struct _config_t {
  uint8_t    state;
  uint8_t    flag;
  int32_t    timeout;
  int32_t    version;

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

  int32_t    n_sonication;

  int32_t    setFrequency;
  int32_t    setOutputVoltage;
  int32_t    setTBD;
  int32_t    setDuty;
  int32_t    setSD;
  int32_t    setISI;
  int32_t    setTD;

  int32_t    setImpedance;
  int32_t    setAbnormalStopMode;

  int32_t    setAbnormalStopMaxV;
  int32_t    setAbnormalStopMaxI;
  int32_t    setAbnormalStopMinV;
  int32_t    setAbnormalStopMinI;

  int32_t    sonication;

} CONFIG_T;

typedef enum _COMMAMD_ID {
    CMD_VERSION = 0,
    CMD_setFrequency = 1,
    CMD_setOutputVoltage = 2,
    CMD_setTBDAndDuty = 3, 
    CMD_setTiming = 4,
    CMD_setImpedance = 5, 
    CMD_setAbnormalStopMode = 6,
    CMD_sonication = 7,
    CMD_clearError = 8,
    CMD_getFrequency = 9,
    CMD_getOutputVoltage = 10,
    CMD_getTBDAndDuty = 11, 
    CMD_getTiming = 12,
    CMD_getImpedance = 13, 
    CMD_getAbnormalStopMode = 14,
    CMD_unknown = 15
} COMMAMD_ID;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_HV_Pin GPIO_PIN_13
#define EN_HV_GPIO_Port GPIOC
#define RTC_XTALI_Pin GPIO_PIN_14
#define RTC_XTALI_GPIO_Port GPIOC
#define RTC_XTALO_Pin GPIO_PIN_15
#define RTC_XTALO_GPIO_Port GPIOC
#define CPU_XTALI_Pin GPIO_PIN_0
#define CPU_XTALI_GPIO_Port GPIOH
#define CPU_XTALO_Pin GPIO_PIN_1
#define CPU_XTALO_GPIO_Port GPIOH
#define EX_RF_1A_Pin GPIO_PIN_0
#define EX_RF_1A_GPIO_Port GPIOC
#define EX_RF_2A_Pin GPIO_PIN_1
#define EX_RF_2A_GPIO_Port GPIOC
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
#define RF_A_Pin GPIO_PIN_6
#define RF_A_GPIO_Port GPIOA
#define STHV748_A_IN2_Pin GPIO_PIN_7
#define STHV748_A_IN2_GPIO_Port GPIOA
#define STH748_A_IN3_Pin GPIO_PIN_4
#define STH748_A_IN3_GPIO_Port GPIOC
#define STH748_B_IN3_Pin GPIO_PIN_5
#define STH748_B_IN3_GPIO_Port GPIOC
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
#define STH748_C_IN3_Pin GPIO_PIN_12
#define STH748_C_IN3_GPIO_Port GPIOB
#define BLE_CTX_Pin GPIO_PIN_13
#define BLE_CTX_GPIO_Port GPIOB
#define BLE_RTS_Pin GPIO_PIN_14
#define BLE_RTS_GPIO_Port GPIOB
#define STHV748_C_IN2_Pin GPIO_PIN_15
#define STHV748_C_IN2_GPIO_Port GPIOB
#define STHV748_D_IN1_Pin GPIO_PIN_6
#define STHV748_D_IN1_GPIO_Port GPIOC
#define STH748_D_IN3_Pin GPIO_PIN_7
#define STH748_D_IN3_GPIO_Port GPIOC
#define STH748_IN4_Pin GPIO_PIN_8
#define STH748_IN4_GPIO_Port GPIOC
#define T_Pin GPIO_PIN_9
#define T_GPIO_Port GPIOC
#define STHV748_A_IN1_Pin GPIO_PIN_8
#define STHV748_A_IN1_GPIO_Port GPIOA
#define STHV748_B_IN1_Pin GPIO_PIN_9
#define STHV748_B_IN1_GPIO_Port GPIOA
#define STHV748_C_IN1_Pin GPIO_PIN_10
#define STHV748_C_IN1_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
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
