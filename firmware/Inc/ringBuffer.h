#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__
#include <stdint.h>

#define RBSIZE 512

/* Following USB Device status */
typedef enum
{
  RINGBUFFER_OK = 0U,
  RINGBUFFER_FULL,
} RINGBUFFER_StatusTypeDef;

uint8_t add2Buffer(uint8_t* Buf, uint16_t Len);
uint8_t FlushBuffer(void);
#endif
