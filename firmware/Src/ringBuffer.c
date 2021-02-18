#include "ringbuffer.h"
#include <stdio.h>

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[RBSIZE];

uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */

uint8_t FlushBuffer(void);

/**
  * @brief  add2Buffer
  *         Data to send over are loaded
  *         through this function.
  *         @note
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t add2Buffer(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = RINGBUFFER_OK;
  // load into buffer, discarded if not enough buffer remaining
  // it will wrap around the end of buffer

  uint16_t i = 1;
	
  if(Len == 0)
		return result;
	
  UserTxBufferFS[UserTxBufPtrIn++] = Buf[0];
  if(UserTxBufPtrIn >= RBSIZE)
	  UserTxBufPtrIn = 0;

  while(UserTxBufPtrIn != UserTxBufPtrOut && i < Len) {
	UserTxBufferFS[UserTxBufPtrIn++] = Buf[i++];
	// roll back if reach end
	if(UserTxBufPtrIn == RBSIZE)
	  UserTxBufPtrIn = 0;
  }
//  if(UserTxBufPtrIn != UserTxBufPtrOut) {
//	  result = FlushBuffer();
//  }
//  if(UserTxBufPtrIn != UserTxBufPtrOut) {
// 	  result = FlushBuffer();
//  }

  return result;
}

uint8_t FlushBuffer(void)
{
  uint32_t buffptr;
  uint32_t buffsize;
  uint8_t result = RINGBUFFER_OK;

  if(UserTxBufPtrOut != UserTxBufPtrIn)
  {
		if(UserTxBufPtrOut > UserTxBufPtrIn) /* Rollback */
		{
			buffsize = RBSIZE - UserTxBufPtrOut;
		}
		else
		{
			buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
		}

		buffptr = UserTxBufPtrOut;

		for(int i=0; i < buffsize; i++) {
			fputc(UserTxBufferFS[buffptr+i], NULL);
		}
	
		if(result == RINGBUFFER_OK)
		{
			UserTxBufPtrOut += buffsize;
			if (UserTxBufPtrOut >= RBSIZE)
			{
			UserTxBufPtrOut = 0;
			}
		}
  }
  return result;
}
