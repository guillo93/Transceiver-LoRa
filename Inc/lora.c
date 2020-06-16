#include <string.h>
//#include <stdlib.h>
#include <stdio.h>
//#include "stm32l4xx_hal.h"
#include "usart.h"
#include "uart_lora.h"
#include "lora.h"
//#include "main.h"
//#include <stdint.h>
//int8_t UART_Send(uint8_t *pBuff, uint16_t Len);

//Глобальное переменное к обращению UART1
extern UART_HandleTypeDef huart1;
unsigned char varParam[100]={0};
unsigned char varBand[50]={0};
unsigned char varAdrr[30]={0};
unsigned char varId[40]={0};
unsigned char varPowr[100]={0};
//==============================================================================
//Функция для сброса модуля 
//==============================================================================
void lora_reset()
{
	char res[20];
	int data = 0;
  unsigned char reset[11] = "AT+RESET\r\n";
  uint16_t lenRes = strlen((char*)reset);//10;//strlen(present);
	UART_Send(reset, lenRes);
	//data = HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);

}

//==============================================================================
//Функция для установки адресс сети
//==============================================================================
int8_t SetAddress_lora(int address)
{
	//char res[20];
	//int data = 0;
	
	memset(varAdrr, 0, sizeof(varAdrr));
	//unsigned char address[15] = "AT+ADDRESS=1\r\n";
	snprintf((char*)varAdrr,50, "AT+ADDRESS=%d\r\n", (char) address);//соединяем длину и массива "fifo" в массиве "varAdrr"
	
  uint16_t lenAddr = strlen((char*)varAdrr);//13;
	UART_Send(varAdrr, lenAddr);
	//data = HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);
}

//==============================================================================
//Функция для установки Ид устройства
//==============================================================================
int8_t SetId_lora(unsigned char idn)
{
	//char res[20];
	//int data = 0;
	
  //unsigned char idn[17] = "AT+NETWORKID=5\r\n";
	snprintf((char*)varId,50, "AT+NETWORKID=%d\r\n", (int)idn);
	
	uint16_t lenId = strlen((char*)varId);//16;//strlen(idn);
	UART_Send(varId, lenId);
	//data = HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);
}

//==============================================================================
//Функция устоновленного режима приемно-передатчика
//==============================================================================
void SetMode_lora()
{
	char res[20];
	int data = 0;
	
  unsigned char mode[12] = "AT+MODE=0\r\n";
  uint16_t lenMod = strlen((char*)mode);//11;//strlen(present);
	UART_Send(mode, lenMod);
	//data = HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);
	
}

//==============================================================================
//Функция для установки центральной частоты ЛоРа
//==============================================================================
int8_t SetBand_lora(int setBand)
{
	//char res[20];
	//int data = 0;
	
  //unsigned char setBand[20] = "AT+BAND=868500000\r\n";
	snprintf((char*)varBand,50, "AT+BAND=%d\r\n", (int)setBand);
	uint16_t lenBand = strlen((char*)varBand);//20;//strlen(setBand);
	UART_Send(varBand, lenBand);
	//data = HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);
	
}

//==============================================================================
//Функция для устаноски параметров модуля ЛоРа
//==============================================================================
int8_t SetParameter_lora(uint8_t Spreading_Factor, uint8_t Bandwidth, uint8_t Coding_Rate, uint8_t Preamble)
{
	//char res[20];
	//int data = 0;
	
  //unsigned char setParameter[23] = "AT+PARAMETER=12,7,4,4\r\n";
	snprintf((char*)varParam,50, "AT+PARAMETER=%d,%d,%d,%d\r\n", Spreading_Factor, (int8_t)Bandwidth, (int8_t)Coding_Rate, (int8_t)Preamble);
	uint16_t lenPartr = strlen((char*)varParam);//23;//strlen(setParameter);
	UART_Send(varParam, lenPartr);
	//data = HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);


}

//==============================================================================
//Функция для установки выходного питания модуля
//==============================================================================
int8_t SetPower_lora(unsigned char *setPower)
{
	//char res[20];
	//int data = 0;
	
	snprintf((char*)varPowr,50, "AT+CRFOP=%d\r\n", (int)setPower);
  //unsigned char setPower[20] = "AT+CRFOP=10\r\n";
	uint16_t lenPow = strlen((char*)varPowr);//strlen(setPower);
	UART_Send(varPowr, lenPow);
	//data = HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);
}

//==============================================================================
//Функция пароля сети
//==============================================================================
void SetPassword_lora()
{
 unsigned char setPass[45] = "AT+CPIN=0D1538D78C6128A998832DBC10CE9223\r\n"; //пароль сети между модулями = Текс=> "Product of LoRa in working" Secret Key => lora lora in2019
 uint16_t lenPass = strlen((char*)setPass);
}
