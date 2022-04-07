
#include "main.h"
#include "soft_timer.h"


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