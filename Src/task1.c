#define TASK1_GLOBAL


#include "main.h"
#include "keyscan.h"
#include "soft_timer.h"


#include "task1.h"
#include "task2.h"

#ifdef CONFIG_PWM_TIM3
extern TIM_HandleTypeDef htim3;
#endif
extern CONFIG_T gConfig;

void InituserTask01(CONFIG_T *sysconf)
{
  setSysTimeOut(KEY_TIMER,250);
  #ifdef CONFIG_PWM_TIM3
  tim3_ch1_duty = 0;
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  #endif
  InitKey();
}

void userTask01(CONFIG_T *sysconf)
{
//  static uint8_t reg = 0;
  uint8_t press , release;
  uint8_t key_value = KEY_NONE;
  
  /* Infinite loop */
  if(getSysTimeOut(KEY_TIMER) == 0)
  {
    setSysTimeOut(KEY_TIMER,250);
  }
  
  press = 0;
  release = 0;

  if(PullKey !=KEY_NONE) {
    key_value = key_table[PullKey];
    PullKey = KEY_NONE;
    release = 1;
  } else if(PushKey != KEY_NONE) {
    key_value = key_table[PushKey];
    PushKey = KEY_NONE;
    press = 1;
  }
  
  if(key_value != KEY_NONE) 
  {
    switch(key_value)
    {
    case KEY_SW1:
      if(press) {
//        tim3_ch1_duty += 10;
//        if(tim3_ch1_duty > 100) tim3_ch1_duty = 0;
//        
//        setPWMDuty(tim3_ch1_duty);
//        
//        reg += 25;
//        dig_port(reg);
        
        LED1_ON(1);
        
        if(sysconf->sonication == 0) {
          sysconf->n_sonication = 1;
          sysconf->updateEnv |= (1 << CMD_sonication) ;
        } else {
          sysconf->n_sonication = 0;
          sysconf->updateEnv |= (1 << CMD_sonication) ;
        }
      } else if(release) {
        LED1_ON(0);
      }
      break;

     default:
      break;
    }
  }
}


