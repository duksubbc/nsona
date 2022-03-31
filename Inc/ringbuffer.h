#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#define rbOK					1	
#define rbINVALID_PARAMETER			2
#define rbINTERNAL_ERROR			3

#define RX_RING_SIZE                            512
#define TX_RING_SIZE                            640

struct ringbuffer
{
	uint8_t * buffer;
	volatile int32_t write_index;
	volatile int32_t read_index;
	int32_t size;
};


// 

// 
extern struct ringbuffer gRingBuffer;
extern char   rxBuffer[];

extern struct ringbuffer *pRingBuffer;


uint16_t rbInitialize(struct ringbuffer * prb, void * buffer, int32_t size);
uint16_t rbClose(struct ringbuffer *prb);

int32_t rbWrite(struct ringbuffer *prb, uint8_t *buffer, int32_t size);
int32_t rbGetWriteAvailable(struct ringbuffer *prb);
int32_t rbRead(struct ringbuffer *prb, uint8_t *buffer, int32_t size);
int32_t rbGetReadAvailable(struct ringbuffer *prb);
uint16_t rbClear (struct ringbuffer *prb);

#endif


