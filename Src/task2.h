#ifndef _TASK2_H_
#define _TASK2_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "buffer.h"


#ifdef TASK2_GLOBAL
  #define TASK2_EXT
#else
  #define TASK2_EXT extern
#endif


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


TASK2_EXT BUFFER_t USART_Buffer;
TASK2_EXT uint8_t  USARTBuffer[];

TASK2_EXT void displayEnv(COMMAMD_ID id);

TASK2_EXT void InituserTask02(CONFIG_T *sysconf);
TASK2_EXT void userTask02(CONFIG_T *sysconf);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif //_TASK2_H_