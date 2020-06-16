//#include "stm32l4xx_hal_usart.h"
#include "stm32l4xx_hal.h"
#include "uart_lora.h"
#include "Delay/delay.h"
#include "main.h"
#include <stdint.h>
#include <stdint.h>
#include <types.h>

//Глобальное переменное
extern UART_HandleTypeDef huart1;
extern struct RCV_LoRa rcv;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
//==============================================================================
//Функция для отправки Ат команды по интерфейсу UART1
//==============================================================================
int8_t UART_Send(unsigned char *pBuff, uint16_t Len)
{
  //UARTstate.TxOvr = 0;
  //char data = 0;
	char *res[20];
 // while (Len--)
  //{
	  HAL_Delay(20);
    HAL_UART_Transmit_DMA(&huart1, pBuff, Len);
		HAL_Delay(100);
		HAL_UART_Receive_DMA(&huart1, (unsigned char *)res, 10);
		HAL_Delay(40);
    //TOcntr = UART_TIMEOUT_BYTE;
  
   // if (!TOcntr)
    //  return UART_ERR_HW_TIMEOUT;
  //}
  
  //return UART_ERR_OK;
	//return (int8_t)res;
	  pBuff = 0;
	  //res[0] =0;
	  //res[1] =0;
	  //res[2] =0;
	//	res[3] =0;
	//	res[4] =0;
   // huart1.pTxBuffPtr = 0;
		//huart1.pRxBuffPtr = 0;
		
}
//==============================================================================
//Функция для отправки данных символы и их длину по интерфейсу UART1
//==============================================================================
void SendFifo_lora(unsigned char *fifoBuff, unsigned char len)
{
	
		  for(int i=0;i<len;i++)
			{
				HAL_Delay(20);
			  HAL_UART_Transmit_DMA(&huart1, fifoBuff, len);
				HAL_Delay(20);				
				//fifo++;			
			}
	
			//HAL_UART_DMAPause(&huart1);
			//HAL_UART_DMAResume(&huart1);
			/*for(int s=0;s<20;s++)
			{	
       fifoBuff[s]=0;
			}		*/	
	    //hdma_usart1_rx.State = HAL_DMA_STATE_BUSY;
			//hdma_usart1_tx.State = HAL_DMA_STATE_READY;
			//hdma_usart3_rx.State = HAL_DMA_STATE_RESET;
			//hdma_usart3_tx.State = HAL_DMA_STATE_READY;
}	
//==============================================================================
//Функция для чтения данных символы и их длину по интерфейсу UART1
//==============================================================================
int ReadFifo_lora(unsigned char *fifoBuff, unsigned char len)
{
 		  //HAL_UART_DMAStop(&huart1);
			//HAL_UART_DMAPause(&huart1);
			//HAL_UART_DMAResume(&huart1);		
		  for(int i=0;i<len;i++)
			{
				//HAL_UART_DMAStop(&huart1);
			  //HAL_UART_DMAPause(&huart1);
			  //HAL_UART_DMAResume(&huart1);
				HAL_Delay(20);
			  *fifoBuff = HAL_UART_Receive_DMA(&huart1, fifoBuff, len);
				HAL_Delay(20);
				//HAL_UART_DMAStop(&huart1);				
			 // HAL_UART_DMAPause(&huart1);
				HAL_Delay(40);
			 // HAL_UART_DMAResume(&huart1);
				//fifo++;		
			}
		  
	     
 return *fifoBuff;
}	