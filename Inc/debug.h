/*-----------------------------------------------------------------------/
/  Low level debug communction port interface include file               /
/-----------------------------------------------------------------------*/

#ifndef _DEBUG_DEFINED
#define _DEBUG_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

uint8_t DEBUG_LL_USARTInit(void *handle);
uint8_t DEBUG_LL_USARTSend(uint8_t* data, uint16_t count);
uint8_t DEBUG_EvtHandler(void);

#ifdef __cplusplus
}
#endif

#endif
