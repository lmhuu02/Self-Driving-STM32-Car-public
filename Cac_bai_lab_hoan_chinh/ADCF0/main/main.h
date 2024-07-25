#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
//#include "stm32f0xx_it.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
//#include "stm32f0xx_usart.h"
//#include "stm32f0xx_exti.h"
#include "lcd16x2.h"

// CAC CHAN DIEU KHIEN
#define LCD16X2_EN		GPIO_Pin_12
#define LCD16X2_RW		GPIO_Pin_11
#define LCD16X2_RS		GPIO_Pin_10
// DATA
#define LCD16x2_Data	GPIOB->ODR
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
