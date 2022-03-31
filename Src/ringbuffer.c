// Circular Buffer Implementation

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "ringbuffer.h"


struct ringbuffer *pRingBuffer;

uint16_t rbInitialize(struct ringbuffer * prb, void * buffer, int32_t size)
{
    if (prb == NULL || size < 1)
        return rbINVALID_PARAMETER;

    memset(prb, 0, sizeof(struct ringbuffer));

    prb->size = size;

    prb->read_index = 0;
    prb->write_index = 0;
    prb->buffer = (uint8_t *)buffer;
    pRingBuffer = prb;

    return rbOK;
}

uint16_t rbClose(struct ringbuffer *prb)
{
	return rbOK;
}

int32_t rbWrite(struct ringbuffer *prb, uint8_t *buffer, int32_t size)
{
    int32_t remain_length, length_to_write, written_length;
    int32_t write_loc;
	int32_t rb_size;

    remain_length = rbGetWriteAvailable(prb);
    if(remain_length < size)
        length_to_write = remain_length;
    else
        length_to_write = size;
    
    written_length = length_to_write;
    rb_size = prb->size;
    
    write_loc = prb->write_index;
    if((write_loc + length_to_write) > rb_size)
    {
        memcpy(prb->buffer + write_loc, buffer, rb_size - write_loc);
        buffer += rb_size - write_loc;
        length_to_write -= rb_size - write_loc;
        write_loc = 0;
    }
    memcpy(prb->buffer + write_loc, buffer, length_to_write);
    prb->write_index = write_loc + length_to_write;
	if( prb->write_index >= rb_size )
		prb->write_index = 0;
    
    return written_length;
}

int32_t rbGetWriteAvailable(struct ringbuffer *prb)
{
    int32_t available;
    int32_t read_loc, write_loc;
    int32_t rb_size;

    read_loc = prb->read_index;
    write_loc = prb->write_index;

    if( read_loc > write_loc ) {
       available = read_loc - write_loc;
    }
    else {
      rb_size = prb->size;
      available = (rb_size - write_loc) + read_loc;
    }

    return available;
}

int32_t rbRead(struct ringbuffer *prb, uint8_t *buffer, int32_t size)
{
    int32_t available_length, length_to_read, read_length;
    int32_t read_loc;
    int32_t rb_size;

    available_length = rbGetReadAvailable(prb);
    if(size > available_length)
        length_to_read = available_length;
    else
        length_to_read = size;
        
    read_length = length_to_read;
    rb_size = prb->size;
    read_loc = prb->read_index;
    
    if (read_loc + length_to_read > rb_size)
    {
        memcpy(buffer, prb->buffer + read_loc, rb_size - read_loc);
        buffer += rb_size - read_loc;
        length_to_read -= rb_size - read_loc;
        read_loc = 0;
    }
    
    memcpy(buffer, prb->buffer + read_loc, length_to_read);
    prb->read_index = read_loc + length_to_read;
    if( prb->read_index >= rb_size )
            prb->read_index = 0;

    return read_length;
}

int32_t rbGetReadAvailable(struct ringbuffer *prb)
{
    int32_t available;
    int32_t read_loc, write_loc;
    int32_t rb_size;

    read_loc = prb->read_index;
    write_loc = prb->write_index;

    if( read_loc > write_loc ) {
        rb_size = prb->size;
        available = (rb_size - read_loc) + write_loc;
    }
    else {
        available = write_loc - read_loc;
    }

    return available;
}

uint16_t rbClear (struct ringbuffer *prb)
{
    memset(prb->buffer, 0, prb->size);
    prb->read_index = 0;
    prb->write_index = 0;

    return rbOK;
}

