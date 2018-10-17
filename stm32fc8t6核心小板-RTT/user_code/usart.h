#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdio.h>

#define BaudRate	 115200//9600

#define USART              	 USART1
#define USART_RCC         	 RCC_APB2Periph_USART1

#define USART_GPIO_RCC    		RCC_APB2Periph_GPIOA
#define USART_TX		        	GPIO_Pin_9	// out
#define USART_RX		        	GPIO_Pin_10	// in 
#define USART_GPIO_PORT    		GPIOA   


#define USART_IPQn   		      USART1_IRQn  
void NVIC_Configuration(void);
void USART1_Config(void);
#endif
