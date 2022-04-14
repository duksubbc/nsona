
#include <stdio.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "debug.h"
#include "buffer.h"
#include "ringbuffer.h"
#include "xprintf.h"
#include "soft_timer.h"
#include "keyscan.h"


extern I2C_HandleTypeDef hi2c1;

#define   SPOSITIVE_AD5170       0x58
#define   SNEGATIVE_AD5170       0x5C
#define   MPOSITIVE_AD5170       0x5A
#define   MNEGATIVE_AD5170       0x5E

char str[128];

void dig_port(uint8_t val)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[2];
  uint8_t rcv[2];
  
  buf[0] = 0x00;
  buf[1] = val;
  ret = HAL_I2C_Master_Transmit(&hi2c1,SNEGATIVE_AD5170,buf,2,100);
  if(ret != HAL_OK)
  {
    xprintf(" I2C Read Error %d \r\n",ret);
  }
  
  ret = HAL_I2C_Master_Receive(&hi2c1,SNEGATIVE_AD5170,rcv,2,100);
  if(ret != HAL_OK)
  {
    xprintf(" I2C Read Error %d \r\n",ret);
  } else {
    sprintf(str,"AD5170 Read 0x%x%x\r\n",rcv[0],rcv[1]);
    xprintf("%s\r\n",str);
  }
  //HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
}


HAL_StatusTypeDef setDigpo(uint8_t ch , uint8_t val, uint8_t *reg)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint16_t DevAddress;
  uint8_t buf[2];
  uint8_t rcv[2];
  
  
  if(ch == 1)
    DevAddress = SPOSITIVE_AD5170;
  else if(ch == 2)
    DevAddress = SNEGATIVE_AD5170;
  else if(ch == 3)
    DevAddress = MPOSITIVE_AD5170;
  else if(ch == 4)
    DevAddress = MNEGATIVE_AD5170;
  else
    return HAL_ERROR;
  
  buf[0] = 0x00;
  buf[1] = val;
  ret = HAL_I2C_Master_Transmit(&hi2c1,DevAddress,buf,2,100);
  if(ret != HAL_OK)
  {
    return ret;//xprintf(" I2C Read Error %d \r\n",ret);
  }
  
  ret = HAL_I2C_Master_Receive(&hi2c1,DevAddress,rcv,2,100);
  if(ret != HAL_OK)
  {
    //xprintf(" I2C Read Error %d \r\n",ret);
  }
  
  if(reg) *reg = rcv[0];
  
  return ret;
}