/* Define to prevent recursive inclusion -------------------------------------*/
	#ifndef __MAIN_H
	#define __MAIN_H
	
	/* Includes ------------------------------------------------------------------*/
	#include "stm32f0xx.h"
	#include "stm32f0xx_usart.h"
	#include "stm32f0xx_gpio.h"
	#include "stm32f0xx_rcc.h"
	
	
	#define BT_BAUD 9600
	#define MAX_STRLEN 1 // this is the maximum string length of our string in characters
	
	void TimingDelay_Decrement(void);
	void Delay(__IO uint32_t nTime);
	void init_USART1(uint32_t baudrate);
	#endif 
	
	

