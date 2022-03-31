#ifndef _SOFT_TIMER_H_
#define _SOFT_TIMER_H_

#ifdef SOFT_TIMER_GLOBAL
  #define SOFT_TIMER_EXT
#else
  #define SOFT_TIMER_EXT extern
#endif

#define MAX_TIMER               8
#define SENSOR_TIMER            0
#define DEBUG_TIMER            1
#define UPDATE_TIMER               2
#define LOWPWR_TIMER            3
#define DELAY_TIMER             4
#define USER_TIMER		5
#define KEY_TIMER               6
#define MAIN_TIMER		7

SOFT_TIMER_EXT int32_t  systemTimer[];

SOFT_TIMER_EXT void    setSysTimeOut(uint8_t ch , int32_t time);
SOFT_TIMER_EXT int32_t getSysTimeOut(uint8_t ch);
SOFT_TIMER_EXT void    InitSysTimeOut(void);


#endif