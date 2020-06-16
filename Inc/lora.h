#ifndef _REYAX_LORA_H
#define _REYAX_LORA_H

#include <types.h>
void lora_reset();
int8_t SetAddress_lora(int address);
int8_t SetId_lora(unsigned char idn);
void SetMode_lora();
int8_t SetBand_lora(int setBand);
int8_t SetParameter_lora(uint8_t Spreading_Factor, uint8_t Bandwidth, uint8_t Coding_Rate, uint8_t Preamble);
int8_t SetPower_lora(unsigned char *setPower);
void SetPassword_lora();


#endif