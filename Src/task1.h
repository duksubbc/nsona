#ifndef _TASK_1_H_
#define _TASK_1_H_  100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef TASK1_GLOBAL
  #define TASK1_EXT
#else
  #define TASK1_EXT extern
#endif



TASK1_EXT void InituserTask01(CONFIG_T *sysconf);
TASK1_EXT void userTask01(CONFIG_T *sysconf);




/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif //_TASK_1_H_