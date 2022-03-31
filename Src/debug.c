
#include "main.h"
#include "debug.h"
#include "ringbuffer.h"
#include "xprintf.h"

#define  USE_XPRINT      1
#define  SEND_BUF_SIZE   32
#define  RCV_BUF_SIZE    32

static UART_HandleTypeDef  *phuart;
static uint8_t   debugTxBuffer[TX_RING_SIZE];
//static uint8_t   debugRxBuffer[RX_RING_SIZE];
static struct    ringbuffer   gRingTxBuffer,gRingRxBuffer;
static uint8_t   sndBuf[SEND_BUF_SIZE];
//static uint8_t   rcvBuf[2][RCV_BUF_SIZE];

static void Putch(unsigned char data) 
{
  uint8_t d = data;
  DEBUG_LL_USARTSend(&d,1);
}

uint8_t DEBUG_LL_USARTInit(void *handle)
{
  if(handle) {  
    phuart = (UART_HandleTypeDef *)handle;

    rbInitialize(&gRingTxBuffer, debugTxBuffer,TX_RING_SIZE);
    
#if (USE_XPRINT == 1)
    xfunc_out=Putch; // Uart1 Debuging 115200bps
#endif
    //HAL_UART_Receive_DMA(phuart,(uint8_t*)rcvBuf[pinpong],RCV_BUF_SIZE);
    //HAL_UART_Receive_IT(phuart,(uint8_t*)rcvBuf[0],1);

  }
  
  return 0;
}


uint8_t DEBUG_LL_USARTSend(uint8_t* data, uint16_t count) 
{
#if 1
  if(rbGetWriteAvailable(&gRingTxBuffer) > count)
    rbWrite(&gRingTxBuffer,(uint8_t *)data,count);
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

  if(phuart) {
    //TX
    if(phuart->gState == HAL_UART_STATE_READY) {
      res = rbGetReadAvailable(&gRingTxBuffer);
      if(res > SEND_BUF_SIZE ) res = SEND_BUF_SIZE;
      if(res ) {
        rbRead(&gRingTxBuffer,sndBuf,res);
        HAL_UART_Transmit_IT(phuart,(uint8_t*)sndBuf,res);
      }
    }
  }
  
  /* Return 0 = Successful */
  return 0;
}