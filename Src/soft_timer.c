
#include "main.h"
#include "soft_timer.h"
#define MAX_TIMER               8
#define SYS_TIMER               0
#define NET_TIMER               1
#define LCD_TIMER               2
#define LOWPWR_TIMER            3
#define DELAY_TIMER             4
#define USER_TIMER		5
#define KEY_TIMER               6
#define MAIN_TIMER		7

int32_t systemTimer[MAX_TIMER] = {-1,-1,-1,-1,-1,-1,-1,-1};

void InitSysTimeOut(void)
{
  for(int i = 0; i < MAX_TIMER;i++)
    systemTimer[i] = -1;
}

void setSysTimeOut(uint8_t ch , int32_t time)
{
  if(ch >= MAX_TIMER)
    return;
  
  systemTimer[ch] = time;
  
  return;
}

int32_t getSysTimeOut(uint8_t ch)
{
  if(ch >= MAX_TIMER)
    return -2;
  return systemTimer[ch];
}