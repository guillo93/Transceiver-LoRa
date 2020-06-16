#ifndef _TX_RX_H
#define _TX_RX_H

#include <types.h>
#define UART_BUFFER_SIZE 100
int Rx_lora();
int Tx_lora();
int8_t Lora_init();
int8_t Lora_sleep();
uint8_t buffer_init(uint8_t data_tx[100], uint8_t lendata);
#endif