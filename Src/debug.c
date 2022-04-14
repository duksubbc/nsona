
#include "main.h"
#include "debug.h"
#include "ringbuffer.h"
#include "xprintf.h"

#define  USE_XPRINT      1
#define  SEND_BUF_SIZE   32
#define  RCV_BUF_SIZE    32

static UART_HandleTypeDef  *phuart1;
static UART_HandleTypeDef  *phuart2;
static uint8_t   debugTxBuffer1[TX_RING_SIZE];
static uint8_t   debugTxBuffer2[TX_RING_SIZE];

//static uint8_t   debugRxBuffer[RX_RING_SIZE];
static struct    ringbuffer   gRingTxBuffer1;
static struct    ringbuffer   gRingTxBuffer2;

static uint8_t   sndBuf1[SEND_BUF_SIZE];
static uint8_t   sndBuf2[SEND_BUF_SIZE];
//static uint8_t   rcvBuf[2][RCV_BUF_SIZE];

static void Putch(unsigned char data) 
{
  uint8_t d = data;
  DEBUG_LL_USARTSend(&d,1);
}

uint8_t DEBUG_LL_USARTInit(void *handle1,void *handle2)
{
  phuart1 = NULL;
  phuart2 = NULL;
    
  if(handle1) {  
    phuart1 = (UART_HandleTypeDef *)handle1;
    rbInitialize(&gRingTxBuffer1, debugTxBuffer1,TX_RING_SIZE);
  }

  if(handle2) {
    phuart2 = (UART_HandleTypeDef *)handle2;
    rbInitialize(&gRingTxBuffer2, debugTxBuffer2,TX_RING_SIZE);
  }
  
  #if (USE_XPRINT == 1)
      xfunc_out=Putch; // Uart1 Debuging 115200bps
  #endif
  
  return 0;
}


uint8_t DEBUG_LL_USARTSend(uint8_t* data, uint16_t count) 
{
#if 1
  if(phuart1) 
  { 
    if(rbGetWriteAvailable(&gRingTxBuffer1) > count)
      rbWrite(&gRingTxBuffer1,(uint8_t *)data,count);
  }
  
  if(phuart2) { 
    if(rbGetWriteAvailable(&gRingTxBuffer2) > count)
      rbWrite(&gRingTxBuffer2,(uint8_t *)data,count);
  }
  /* Return 0 = Successful */
#else
  HAL_UART_StateTypeDef res;
  do {
    res = HAL_UART_Transmit(phuart,(uint8_t*)data,count,100);
  } while(res != HAL_OK);
#endif
  return 0;
}


uint8_t DEBUG_EvtHandler(void) 
{
  /* Send data via USART */
    int32_t  res;
  
    if(phuart1) {
      //TX
      if(phuart1->gState == HAL_UART_STATE_READY) {
        res = rbGetReadAvailable(&gRingTxBuffer1);
        if(res > SEND_BUF_SIZE ) res = SEND_BUF_SIZE;
        rbRead(&gRingTxBuffer1,sndBuf1,res);
      
        HAL_UART_Transmit_IT(phuart1,(uint8_t*)sndBuf1,res);
      }
    }
    
    if(phuart2) {
      //TX
      if(phuart2->gState == HAL_UART_STATE_READY) {
        res = rbGetReadAvailable(&gRingTxBuffer2);
        if(res > SEND_BUF_SIZE ) res = SEND_BUF_SIZE;
        rbRead(&gRingTxBuffer2,sndBuf2,res);
        HAL_UART_Transmit_IT(phuart2,(uint8_t*)sndBuf2,res);
      }
    }

  /* Return 0 = Successful */
  return 0;
}