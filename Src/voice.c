
#include "main.h"


extern SPI_HandleTypeDef hspi1;

//******************************* Volume Level ******************************//
#define VOL_MUTE    0x0000
#define VOL_MIN     0x4407
#define VOL_NORMAL  0x4404
#define VOL_MAX     0x4400



uint16_t         g_SoundVolume;
//VoiceMode 	 g_VoiceMode;
uint32_t         g_msTickCount;

typedef enum _VOICE_ID {
    VID_POWER_ON = 0,
    VID_POWER_OFF = 1,
    VID_PUT_ON = 2,
} VOICE_ID;

/* Private macro -------------------------------------------------------------*/

//#define AUDIO_CS_LOW()          (AUDIO_CS_GPIO_Port->BSRR = AUDIO_CS_Pin << 16)
//#define AUDIO_CS_HI()           (AUDIO_CS_GPIO_Port->BSRR = AUDIO_CS_Pin)
//
//#define AUDIO_RESET_LOW()       (AUDIO_RESET_GPIO_Port->BSRR = AUDIO_RESET_Pin << 16)
//#define AUDIO_RESET_HI()        (AUDIO_RESET_GPIO_Port->BSRR = AUDIO_RESET_Pin)


#define AUDIO_CS_LOW()	        do{HAL_GPIO_WritePin(GPIOA, AUDIO_CS_Pin,RESET);  HAL_Delay(10);}  while(0)
#define AUDIO_CS_HI()	        do{HAL_GPIO_WritePin(GPIOA, AUDIO_CS_Pin,SET); HAL_Delay(10);} while(0)

#define AUDIO_RESET_LOW()       do{HAL_GPIO_WritePin(GPIOC, AUDIO_RESET_Pin,RESET);  HAL_Delay(1);}  while(0)
#define AUDIO_RESET_HI()        do{HAL_GPIO_WritePin(GPIOC, AUDIO_RESET_Pin,SET); HAL_Delay(1);} while(0)

#define IO_O_VSCL_SetLow()	do{HAL_GPIO_WritePin(GPIOB, SPI_SCK_Pin,RESET);  HAL_Delay(1);}  while(0)
#define IO_O_VSCL_SetHigh()	do{HAL_GPIO_WritePin(GPIOB, SPI_SCK_Pin,SET); HAL_Delay(1);} while(0)
#define IO_O_VSDA_SetLow()	do{HAL_GPIO_WritePin(GPIOB, SPI_MOSI_Pin,RESET); HAL_Delay(1);}  while(0)
#define IO_O_VSDA_SetHigh()	do{HAL_GPIO_WritePin(GPIOB, SPI_MOSI_Pin,SET);HAL_Delay(1);} while(0)

//static void I2C_data(uint16_t cmd);
//static uint8_t is_sound_chip_busy(void);
//static void sound_power_up(void);
//static void sound_power_down(void);
//static void sound_volume(uint16_t cmd);
//static void sound_output(VOICE_ID vid);

static void I2C_data(uint32_t cmd)
{
  uint32_t mask = 0;
  for(mask = 0x800000; mask > 0; mask >>= 1)
  {
    IO_O_VSCL_SetLow();
    if(cmd & mask)
      IO_O_VSDA_SetHigh();
    else
      IO_O_VSDA_SetLow();

    IO_O_VSCL_SetHigh();
  }
  IO_O_VSCL_SetLow();
}


static uint8_t is_sound_chip_busy(void)
{
  if (HAL_GPIO_ReadPin(GPIOD, AUDIO_BUSY_Pin) == SET)
    return 1;
  else
    return 0;
}

void sound_power_up(void)
{
  uint16_t cmd = 0;
  cmd = 0xA800;
  IO_O_VSCL_SetHigh();
  IO_O_VSDA_SetHigh();
  IO_O_VSDA_SetLow();
  I2C_data(cmd);
  IO_O_VSDA_SetLow();
  IO_O_VSCL_SetHigh();
  IO_O_VSDA_SetHigh();
  while(is_sound_chip_busy());
}

void sound_power_down(void)
{
  uint16_t cmd = 0;
  cmd = 0xB800;
    
  while(is_sound_chip_busy());
    
  IO_O_VSCL_SetHigh();
  IO_O_VSDA_SetHigh();
  IO_O_VSDA_SetLow();
  I2C_data(cmd);
  IO_O_VSDA_SetLow();
  IO_O_VSCL_SetHigh();
  IO_O_VSDA_SetHigh();

  while(is_sound_chip_busy());
}


void sound_volume(uint16_t cmd)
{
  IO_O_VSCL_SetHigh();
  IO_O_VSDA_SetHigh();
  IO_O_VSDA_SetLow();
  I2C_data(cmd);
  IO_O_VSDA_SetLow();
  IO_O_VSCL_SetHigh();
  IO_O_VSDA_SetHigh();
}


void sound_output(VOICE_ID vid)
{
  //u8 cnt=0;
  uint32_t old_time, temp;

  uint16_t cmd = 0;

  old_time = g_msTickCount;

  cmd = 0x9800 + vid;
  if (g_SoundVolume != VOL_MUTE)
  {
          IO_O_VSCL_SetHigh();
          IO_O_VSDA_SetHigh();
          IO_O_VSDA_SetLow();
          I2C_data(cmd);
          IO_O_VSDA_SetLow();
          IO_O_VSCL_SetHigh();
          IO_O_VSDA_SetHigh();
  }
}

void sound_Paly(uint8_t vid)
{
  uint16_t data = 0;
  data |= 0xE0 << 8;
  data |= vid;
  AUDIO_CS_LOW();
  HAL_SPI_Transmit(&hspi1,(uint8_t *)&data, 1,100);
  AUDIO_CS_HI();

}

uint16_t gdata = 0;
void sound_enable(void)
{
  int i;
  uint16_t data = 0;
  
  AUDIO_RESET_HI();
  AUDIO_CS_HI();
  
  HAL_Delay(10);

  for(i =0 ; i < 4;i++)
  {
    data |= (0xE0+i) << 8;
    if(i == 1)
      data |= 0x08 ;
    else
      data |= 00 ;
    
    AUDIO_CS_LOW();
    HAL_SPI_Transmit(&hspi1,(uint8_t *)&data, 1,100);
    AUDIO_CS_HI();
    data = 0;
  }
}