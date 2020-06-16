#ifndef _SPIM_H
#define _SPIM_H

#include "types.h"
//#include <stdint.h>

int8_t UART_Send(uint8_t *pBuff, uint16_t Len);
int ReadFifo_lora(unsigned char *fifo, unsigned char len);
void SendFifo_lora(unsigned char *fifo, unsigned char len);
#endif