/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <lora.h>
#include <uart_lora.h>
#include <stdlib.h>
#include <stdbool.h>
#include <Tx_Rx.h>

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

typedef struct 
{
 uint8_t addr[8];
 uint8_t lenDat[4];
 uint8_t data[15];
 uint8_t checksum[10];
 uint8_t modbus[10];
 uint8_t rssi[4];
 uint8_t snr[4];
} RCV_LoRa; 


//отработчик данных uart

#define BUFFER_RX_SIZE 100
volatile uint16_t RXBuffer[BUFFER_RX_SIZE];
extern volatile uint16_t uartBuffer[UART_BUFFER_SIZE];
uint16_t RxCnt;
uint8_t chrx;
uint16_t uartCnt;
uint8_t ch;

//включение и выключение светодиода  
#define LedOff()        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define LedOn()         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)

//вызова глобального переменного для времени и даты
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
//Буфер для хранения время и дата
char trans_str[64] = {0};
char rec_data[80] = {0};
uint8_t fifo[100] = {0};//массив для храннения данных буфера из модуля для передатчика
uint8_t count=0;//счетчик передатчика где будет храниться длину символы
uint8_t fiforx[100] = {0};	//uint8_t

uint8_t countrx=0;//счетчик для применика где будет храниться длину символы
//переменные время и дата
uint8_t t_hour;
uint8_t t_minute;
uint8_t t_second;
uint8_t d_date;
uint8_t d_month;
uint8_t d_year;
RCV_LoRa rcv;



//==============================================================================
//Функция для расчета контрольной сумме на целостности пакета методом CRC_16
//==============================================================================
// для расчета контрольной суммы CRC
// принимает указатель на область памяти с данными 
// и количество принятых байт
char *strdup(const char *c)
{
    char *dup = malloc(strlen(c) + 1);

    if (dup != NULL)
       strcpy(dup, c);

    return dup;
}

static const unsigned char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};
 
 
static const unsigned char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};
 
unsigned int Crc16_Modbus(unsigned char *pData, int length)
{
    unsigned char   ucCRCHi = 0xFF;
    unsigned char   ucCRCLo = 0xFF;
    int             idx;
 
    while( length-- )
    {
        idx = ucCRCLo ^ *( pData++ );
        ucCRCLo = ( unsigned char )( ucCRCHi ^ aucCRCHi[idx] );
        ucCRCHi = aucCRCLo[idx];
    }
     
    return ( unsigned int )( ucCRCHi << 8 | ucCRCLo );
}

#define CRC16 0x8005
uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    //Sanity check: 
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        //Get next bit: 
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        // Increment bit counter: 
        bits_read++;
			  int i = 0;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        // Cycle check: 
        if(bit_flag)
            out ^= CRC16;

    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}

//==============================================================================
//Функция вызова работы модуля
//==============================================================================
int8_t Lora_init()
{
	
	/*************************************************************************
	<Spreading_Factor>7~12, (default 12)
	<Bandwidth>0~9 list as below
	0 : 7.8KHz (not recommended, over spec.)
	1 : 10.4KHz (not recommended, over spec.)
	2 : 15.6KHz
	3 : 20.8 KHz
	4 : 31.25 KHz
	5 : 41.7 KHz
	6 : 62.5 KHz
	7 : 125 KHz (default).
	8 : 250 KHz
	9 : 500 KHz
	<Coding_Rate>1~4, (default 1)
	<Programmed Preamble> 4~7(default 4)
	
	***************************************************************************/
	uint8_t Spreading_Factor = 12;
	uint8_t Bandwidth = 9;
	uint8_t Coding_Rate = 4;
	uint8_t Preamble = 4;
	
	HAL_Delay(20);
	lora_reset();// сброс модуля перед иницилализации
	HAL_Delay(20);
	SetAddress_lora(1);// Адресс сети ЛоРа = 1
	HAL_Delay(20);
	SetId_lora(5);// установка пД = 5
	HAL_Delay(20);
	SetMode_lora();// установленный режим приемно-передатчика = 0
	HAL_Delay(20);
	SetBand_lora(868500000);// установленная центральная частота = 868,5 МГц
	HAL_Delay(20);
	SetParameter_lora(Spreading_Factor, Bandwidth, Coding_Rate, Preamble);// коэфф. ширина спектра = 12, ширина полосы = 7 (125 КГц), скорость кодирования = 4, преамбул = 4
	HAL_Delay(20);
	SetPassword_lora();
	HAL_Delay(9);


memset((uint8_t *)&uartBuffer,(char)0xff,sizeof(uartBuffer));
HAL_UART_Receive_DMA(&huart1, (uint8_t *)&uartBuffer, UART_BUFFER_SIZE);

   HAL_Delay(100);	
	LedOn();
	HAL_Delay(200);	
	LedOff();
	HAL_Delay(100);	
 
}

uint8_t buffer_init(uint8_t data_tx[100], uint8_t lendata)
{

	  for (uint16_t i = 0; i <UART_BUFFER_SIZE;i++)
   {      uartBuffer[i]=0xffff;
    
   }
	 //(unsigned char*)uartBuffer = data_tx;
	 
	// memcpy((void*)uartBuffer, data_tx, lendata);
	// lendata = UART_BUFFER_SIZE;
			HAL_UART_Receive_DMA(&huart3, (unsigned char*)uartBuffer, UART_BUFFER_SIZE);
		  HAL_Delay(5);
	 /*for(int i = 0; i < UART_BUFFER_SIZE; i++) 
	 {
	  uartBuffer[i];
	 }
	 */
	 
}	

//==============================================================================
//функция вызова сна в модуле LoRa
//==============================================================================
int8_t Lora_sleep()
{
  unsigned char present[5] = "AT\r\n";
  uint16_t len1 = strlen((char*)present);//4;//strlen(present);	
	unsigned char reset[11] = "AT+RESET\r\n";
  uint16_t lenRes = strlen((char*)reset);//10;//strlen(present);
  unsigned char address[15] = "AT+ADDRESS=1\r\n";
	uint16_t lenAddr = strlen((char*)address);//13;
  unsigned char idn[17] = "AT+NETWORKID=5\r\n";
	uint16_t lenId = strlen((char*)idn);//16;//strlen(idn);
  unsigned char mode[12] = "AT+MODE=1\r\n";
  uint16_t lenMod = strlen((char*)mode);//11;//strlen(present);
  unsigned char setBand[20] = "AT+BAND=868500000\r\n";
	uint16_t lenBand = strlen((char*)setBand);//20;//strlen(setBand);
	unsigned char setParameter[23] = "AT+PARAMETER=12,7,4,4\r\n";
	uint16_t lenPartr = strlen((char*)setParameter);//23;//strlen(setParameter);
	unsigned char setPower[20] = "AT+CRFOP=10\r\n";
	uint16_t lenPow = strlen((char*)setPower);//strlen(setPower);
  	
	//UART_Send(present, len1);
	
	//пнициалиации модуля LoRa
	HAL_Delay(20);
	UART_Send(reset, lenRes);
	HAL_Delay(20);
  UART_Send(address, lenAddr);
	HAL_Delay(20);
	UART_Send(idn, lenId);
	HAL_Delay(20);
  UART_Send(mode, lenMod);
	HAL_Delay(20);
  //UART_Send(setBand, lenBand);
	//HAL_Delay(20);
	//UART_Send(setParameter, lenPartr);
	//HAL_Delay(20);
}

//==============================================================================
//функция для подвеждения данных в буфере для передатчика
//==============================================================================
uint8_t getByte(uint8_t*b)
{
  
  if (uartBuffer[uartCnt]!=0xffff)
   {
      *b=(uint8_t)(uartBuffer[uartCnt]& 0x00FF);
      uartBuffer[uartCnt]=0xffff;
      uartCnt++;
      if (uartCnt==UART_BUFFER_SIZE) uartCnt=0;
      return 1;      
   } else
   {
      return 0;
   }
}

//==============================================================================
//функция для подвеждения данных в буфере для приемника
//==============================================================================
uint8_t getByterx(uint8_t*d)
{
  
  if (RXBuffer[RxCnt]!=0xffff)
   {
      *d=(uint8_t)(RXBuffer[RxCnt]& 0x00FF);
      RXBuffer[RxCnt]=0xffff;
      RxCnt++;
      if (RxCnt==BUFFER_RX_SIZE) RxCnt=0;
      return 1;      
   } else
   {
      return 0;
   }
}

//==============================================================================
//функция обработчика буфер dma для приемника
//==============================================================================
void dma_fifo_buffer_rx()
{
    uint32_t timeout;    		
		if(getByterx(&chrx) == 1)
	  {
			fiforx[0]=chrx;
			countrx++;
			timeout=0;
			while(timeout < 200000)
			{
				while ((getByterx(&chrx)==0) && (timeout<200000)) {timeout++;}
				if(timeout<200000)
				{
				 fiforx[countrx] = chrx;
				 countrx++;
				 timeout=0;
				}
			}	
			timeout=0;
	  }
}	

//==============================================================================
//функция обработчика буфер dma для передатчика
//==============================================================================
void dma_fifo_buffer()
{
    uint32_t timeout;    		
		if(getByte(&ch) == 1)
	  {
			fifo[0]=ch;
			count++;
			timeout=0;
			while(timeout < 200000)
			{
				while ((getByte(&ch)==0) && (timeout<200000)) {timeout++;}
				if(timeout<200000)
				{
				 fifo[count] = ch;
				 count++;
				 timeout=0;
				}
			}	
			timeout=0;
	  }
}	


								
//**************************************************
//буфер массива где хранится полученные данные из модуля для отображения, для передачи и вычисления CRC16
char* data=0;
unsigned char res[80];
unsigned char var[100];
unsigned char var_2[100];
unsigned char var_3[100];
uint16_t crc_1;
uint16_t crc_modbus;
uint16_t crc_2;
char crc_3[4];
unsigned char *fi;
int LenFifo = 0;
int LenCrc = 0;
int LenModbus = 0;
int LenVar = 0;
int LenVar2 = 0;
int LenVar3= 0;
uint8_t sn = 0;	
uint8_t rss = 0;	
uint8_t fidat[35] = {0};	
char *sig;
char *token, *last;
unsigned char coma;
char *p;




//=============================================================================================Receiver -- получение данных из приемника

int Rx_lora()
{	
	
	//перед началом очишаем полностю буфер
  memset(res, 0, sizeof(res));
	char print[100]={0};
	memset(print, 0, sizeof(print));
	memset(var, 0, sizeof(var));
	memset(var_2, 0, sizeof(var));
	memset(var_3, 0, sizeof(var));
	memset((void*)(&rcv), 0, sizeof(rcv)); 
	memset(crc_3, 0x00, sizeof(crc_3));
	
	huart3.RxState= HAL_UART_STATE_BUSY_RX;
	
	
			  
	      huart3.pRxBuffPtr = 0;
			  dma_fifo_buffer_rx();
				HAL_Delay(12);
				//memset((void*)(&rcv), 0, sizeof(rcv));
				//memset((void*)(&rcv), 0, sizeof(rcv));
				char rcvd_str[80]; //"+RCV=1,5,HELLO:0x34D2,-35,20" "+RCV=1,5,HELLO\r\n:0x34D2;0x4548,-35,20\r\n"
        ReadFifo_lora((unsigned char *)rcvd_str, 80);
				
        if((strstr((char *)rcvd_str, "ready")) && !(strstr((char *)rcvd_str, "Error")))
			 {
				 HAL_Delay(70);				 
				 HAL_UART_Transmit_IT(&huart3, (unsigned char *)rcvd_str, strlen((char*)rcvd_str));
				 HAL_Delay(70);
				 for(int r=0;r<=4;r++)//делаем признак индикации полученных данных на приеме 
						{						
							HAL_Delay(140);	
							LedOn();
							HAL_Delay(500);	
							LedOff();
							HAL_Delay(100);								
					  }
				 for(int s=0; s <= 80; s++)
						{
						  // string1[s] = 0;
							 rcvd_str[s]=0;
               								
						}
						
				 HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
         CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
         huart3.RxState= HAL_UART_STATE_READY;

         SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE); 						
				 HAL_UART_Transmit_IT(&huart3, (unsigned char *)"====================================================================\n\r", strlen((char*)"====================================================================\n\r"));				
				 huart3.RxState= HAL_UART_STATE_BUSY_RX;
				}
						
			 else if((strstr((char *)rcvd_str, "Error")) && !(strstr((char *)rcvd_str, "ready")))
			 { 
				 			 
				HAL_Delay(70);
				HAL_UART_Transmit_IT(&huart3, (unsigned char *)rcvd_str, strlen((char*)rcvd_str));
			  HAL_Delay(70);
				for(int s=0; s <= 80; s++)
				{
						  
							 rcvd_str[s]=0;
               								
				}
				HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
				HAL_UART_Transmit_IT(&huart3, (unsigned char *)"====================================================================\n\r", strlen((char*)"====================================================================\n\r"));	
				HAL_Delay(15);
				huart3.RxState= HAL_UART_STATE_BUSY_RX; 					
			 }
        
        
////////////---------------------------------------------------------------------------------------------------------------------------------------------------
			  data = strstr((char *)huart1.pRxBuffPtr, "+RCV=");//считаем первые 4 символа		
		    if((data || strstr(rcvd_str, "+RCV="))&&(!(strstr((char *)rcvd_str, "ready")) && !(strstr((char *)rcvd_str, "Error")))) 
				{	
					//memset(rcv, 0, sizeof(rcv));
					HAL_Delay(50);
         	//HAL_UART_DMAResume(&huart1);
					
						HAL_Delay(12);
						//for(int s=0;s<=3;s++)
						//{	
						  CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
             huart3.RxState= HAL_UART_STATE_READY;

              SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE); 
						 //------------------------------------------------------------------------------------------------------------
						//Сообщение приема данных по интерфейсу UART 3
						//------------------------------------------------------------------------------------------------------------
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Подготовка к принятию данных\n\r", strlen((char*)"Подготовка к принятию данных\n\r"));
						HAL_Delay(2);
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Получение данных...\n\r", strlen((char*)huart1.pRxBuffPtr));
						//snprintf(rec_data, 79, "Принимаемый данный: %s\n\r", res);
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
					  HAL_Delay(10);	
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Принимаемые данные:\n\r", strlen((char*)"Принимаемые данные:\n\r"));
						HAL_Delay(10);	
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
						//Отправляем полученные данные из приема на отображение по интерфейсу UART 3 
						//HAL_UART_Transmit_DMA(&huart3, (unsigned char *)fiforx, countrx);
						HAL_Delay(30);
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
						//HAL_UART_Transmit(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"), 1000);
						//HAL_UART_Transmit(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"), 1000);
						HAL_Delay(20);
						
						ReadFifo_lora((unsigned char*)rcvd_str, 80);//осуществляется чтение данных из передатчика
						for(int r=0;r<=4;r++)//делаем признак индикации полученных данных на приеме 
						{						
							HAL_Delay(50);	
							LedOn();
							HAL_Delay(50);	
							LedOff();
							HAL_Delay(100);								
					  }
						CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
            huart3.RxState= HAL_UART_STATE_READY;

            SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
						 //ReadFifo_lora((unsigned char*)fiforx, 70);//осуществляется чтение данных из передатчика
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)rcvd_str, strlen((char*)rcvd_str));
						
						  
														char * str2 = strdup(rcvd_str);

														char * del = ",:;\r\n";
														char * ptr = NULL;
                            
														ptr = strtok(str2, del);
														if (!ptr) return 0;
														strcpy((char*)rcv.addr, ptr);

														ptr = strtok(NULL, del);
														if (!ptr) return 0;
														strcpy((char*)rcv.lenDat, ptr);

														ptr = strtok(NULL, del);
														if (!ptr) return 0;
														strcpy((char*)rcv.data, ptr);

														ptr = strtok(NULL, del);
														if (!ptr) return 0;
														strcpy((char*)rcv.checksum, ptr);
														
														ptr = strtok(NULL, del);
														if (!ptr) return 0;
														strcpy((char*)rcv.modbus, ptr);

														ptr = strtok(NULL, del);
														if (!ptr) return 0;
														strcpy((char*)rcv.rssi, ptr);

														ptr = strtok(NULL, del);
														if (!ptr) return 0;
														strcpy((char*)rcv.snr, ptr);
														
														free(str2);
                    
						                
						LenVar2 = strlen((char*)rcv.data);
						LenVar3 = strlen((char*)"AT+SEND=1,7,ready\r\n");
						crc_2 = gen_crc16(rcv.data, LenVar2);
						//atoi((char*)rcv.checksum
						
						sprintf(crc_3, "%u", crc_2);
					/*	for(int i=0;crc_2>0;i++,crc_2/=10)
						{
						  crc_3[i] = crc_2%10+'0';
						}*/
						
						
						
            if((crc_3[0] != rcv.checksum[0])&&
							(crc_3[1] != rcv.checksum[1])&&
						  (crc_3[2] != rcv.checksum[2])&&
						  (crc_3[3] != rcv.checksum[3]))// && ((strncmp((char*)rcv.data, "ready",5) <= 0) || (strncmp((char*)rcv.data, "Error",5) <= 0)))
						{
							 HAL_Delay(30);
							 HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
							 HAL_Delay(10);	
							 HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
							 HAL_Delay(10);
						   HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Ошибка!\n\r", strlen((char*)"Ошибка!\n\r"));
							for(int d=0;d<=1;d++)
				      {	
							 SendFifo_lora((unsigned char*)"AT+SEND=1,7,Error\r\n", strlen((char*)"AT+SEND=1,7,Error\r\n"));//осуществляется передачи данных в эфире
							} 
								crc_2 = 0;
							  memset(rcv.addr, 0, sizeof(rcv.addr));
								memset(rcv.checksum, 0, sizeof(rcv.checksum));
								memset(rcv.data, 0, sizeof(rcv.data));
								memset(rcv.lenDat, 0, sizeof(rcv.lenDat));
								memset(rcv.modbus, 0, sizeof(rcv.modbus));
								memset(rcv.rssi, 0, sizeof(rcv.rssi));
								memset(rcv.snr, 0, sizeof(rcv.snr));
								//HAL_UART_DMAStop(&huart1);
								HAL_Delay(10);
								//HAL_UART_DMAResume(&huart1);
								//Lora_init();
								//break;
							 
						 }
             else
						{
							HAL_Delay(30);
						  HAL_UART_Transmit(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"), 1000);
							HAL_Delay(10);	
						  HAL_UART_Transmit(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"), 1000);
							HAL_Delay(70);
						  HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Пакет прошел успешно\n\r", strlen((char*)"Пакет прошел успешно\n\r"));
							hdma_usart1_rx.State = HAL_DMA_STATE_BUSY;
			        //hdma_usart1_tx.State = HAL_DMA_STATE_RESET;
			         hdma_usart3_rx.State = HAL_DMA_STATE_BUSY;
			        //hdma_usart3_tx.State = HAL_DMA_STATE_RESET;
							for(int d=0;d<=1;d++)
				     {	
							SendFifo_lora((unsigned char*)"AT+SEND=1,7,ready\r\n", LenVar3);//осуществляется передачи данных в эфире
							 HAL_Delay(700);
						 }	 
							memset(rcvd_str, 0, sizeof(rcv.modbus));
							//crc_2 = 0;
							HAL_Delay(40);//ждем 					
				
						CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
            huart3.RxState= HAL_UART_STATE_READY;

            SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
						HAL_Delay(10);											
						//ссылаем сообщение статуса очистки буфера
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"очишается буфер приемника\n\r", strlen((char*)"очишается буфер приемника\n\r"));
						data[0]=0;
						memset(rcvd_str, 0, sizeof(rcvd_str));
						memset(var_2, 0, sizeof(var_2));
						 huart1.pTxBuffPtr = 0;
						//memset(rcvd_str, 0, sizeof(rcvd_str));
						HAL_Delay(40);//ждем чистки буфера
            HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Буфер успешно очишен\n\r", strlen((char*)"Буфер успешно очишен\n\r"));
						HAL_Delay(15);
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"====================================================================\n\r", strlen((char*)"====================================================================\n\r"));							
					    HAL_Delay(30);
							memset(rcv.addr, 0, sizeof(rcv.addr));
							memset(rcv.checksum, 0, sizeof(rcv.checksum));
							memset(rcv.data, 0, sizeof(rcv.data));
							memset(rcv.lenDat, 0, sizeof(rcv.lenDat));
							memset(rcv.modbus, 0, sizeof(rcv.modbus));
							memset(rcv.rssi, 0, sizeof(rcv.rssi));
							memset(rcv.snr, 0, sizeof(rcv.snr));
							//HAL_UART_DMAStop(&huart1);
							HAL_Delay(50);
							//HAL_UART_DMAResume(&huart1);
							//Lora_init();
							//break;
							
							huart3.RxState= HAL_UART_STATE_BUSY_RX;
							
						}						 
           
						
			}  					
     
						
    	
	
}


 //======================================================================================================Transmiter -- отправка данных на передатчике
	

int Tx_lora()
{
	//data_tx = fifo;
	
	//перед началом очишаем полностю буфер
	memset(res, 0, sizeof(res));
	char print[100]={0};
	memset(print, 0, sizeof(print));
	memset(var, 0, sizeof(var));
	memset(var_2, 0, sizeof(var));
	memset(var_3, 0, sizeof(var));
	memset((void*)(&rcv), 0, sizeof(rcv)); 
	memset(crc_3, 0x00, sizeof(crc_3));
	
	
			memset(fifo, 0x00, sizeof(fifo)); //очишаем массив fifo  
			dma_fifo_buffer();
			int size;
		for(size=0; size<100;size++)
			if(fifo[size] !=0)		//huart1.pTxBuffPtr != 0
			{  if(count != 0)
				{ 
				 //------------------------------------------------------------------------------------------------------------
				 //Сообщение передачи данных по интерфейсу UART 3
				 //------------------------------------------------------------------------------------------------------------
					CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
          huart3.RxState= HAL_UART_STATE_READY;

          SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
					
				  HAL_UART_Transmit_IT(&huart3, (unsigned char *)"====================================================================\n\r", strlen((char*)"====================================================================\n\r"));
				  HAL_Delay(80);
				  HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Инициализация передатчика...\n\r", strlen((char*)"Инициализация передатчика...\n\r"));
          HAL_Delay(80);
          //UART_Send(string, lenStr);
					
					HAL_Delay(50);					
					HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Данные:\n", strlen((char*)"Данные:\n"));
					//Lora_init();
					//uint8_t str[] = "hi";
					
					LenFifo = count - 2;
					
					//Вычисляем CRC
					crc_1=gen_crc16(fifo, LenFifo);//strlen((char*)str)
					crc_modbus=Crc16_Modbus(fifo,LenFifo);
					LenCrc = sizeof(crc_1);
					LenModbus = sizeof(crc_modbus);
					
					snprintf((char*)var,50, "AT+SEND=1,%d,%s:%d;%d\r\n", LenFifo + 14, fifo, crc_1, crc_modbus);//соединяем длину и массива "fifo" в массиве "var" для отправки в эфире"\r\n
					
					
					HAL_UART_Transmit_DMA(&huart3,(unsigned char*)var, LenVar);//отображаем данные через интерфей UART3
					HAL_Delay(20);//ждем
					HAL_UART_Transmit_IT(&huart3, (unsigned char *)"\n\r", strlen((char*)"\n\r"));
					HAL_Delay(10);	
				  HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Отправка символов готова\n\r", strlen((char*)"Отправка символов готова\n\r"));
          HAL_Delay(10);						
				  HAL_UART_Transmit_IT(&huart3, (unsigned char *)"отправляется...\n\r", strlen((char*)"отправляется...\n\r"));
				  HAL_Delay(10);//ждем
					LenVar = strlen((char*)var);//strlen((char*)var);sizeof(var)
					
					memset(fifo, 0, sizeof(fifo));
					count = 0;
					size=0;
					int is;
				  for(is = 0; is<100; is++)
					if(var[is] != 0)
					{	
						
						for(int d=0;d<=0;d++)
						{	
							 
							 SendFifo_lora(var, LenVar);//осуществляется передачи данных в эфире
							 //HAL_Delay(11);
							 
							 HAL_Delay(1000);
								for(int i=0; i<4;i++)
								{
									 HAL_Delay(100);
										LedOn();
										HAL_Delay(100);	
										LedOff();
										HAL_Delay(100);						
								}
									 
						}
						//memset(fifo, 0, sizeof(fifo));
						memset(var, 0, sizeof(var));
						
				  }	
					HAL_Delay(5);//ждем
					
																
					if(strstr((char*)huart1.pRxBuffPtr, "+OK"))//проверяем если прошла успешна передача в эфире
					{	
						
						HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Отправка данных прошла успешна\n\r", strlen((char*)"Отправка данных прошла успешна\n\r"));
						HAL_Delay(1);
						
						//memset(rcvd_str, 0, sizeof(rcv.modbus));
								 
							  	
						
						for(int s=0; s < 80; s++)
						{
						  // string1[s] = 0;
							 fifo[s]=0;
               								
						}
						for(int s=0; s < 100; s++)
						{
						  // string1[s] = 0;
							 var[s]=0;
               								
						}
						HAL_Delay(10);
            HAL_UART_Transmit_IT(&huart3, (unsigned char *)"очишается буфер передатчика\n\r", strlen((char*)"очишается буфер передатчика\n\r"));						
            count=0;
						LenFifo = 0;
					  memset(var, 0, sizeof(var));
						memset(var_2, 0, sizeof(var));	
						memset(var_3, 0, sizeof(var));	
            //memset(string1, 0, sizeof(string1));						
				  }
          HAL_Delay(20);
          HAL_UART_Transmit_IT(&huart3, (unsigned char *)"Буфер успешно очишен\n\r", strlen((char*)"Буфер успешно очишен\n\r"));
					HAL_Delay(30);
          HAL_UART_Transmit_IT(&huart3, (unsigned char *)"====================================================================\n\r", strlen((char*)"====================================================================\n\r"));		
          CLEAR_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
           huart3.RxState= HAL_UART_STATE_READY;

          SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);					
        }
      } 	
}