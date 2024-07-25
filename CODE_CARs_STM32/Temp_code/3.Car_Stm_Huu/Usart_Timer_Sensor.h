/*
 * Distance can be calculated by using the following formula :- range = high level time * velocity (340m/s) / 2
We can also use uS / 58 = Distance in cm or uS / 148 = distance in inch
It is recommended to wait for at least 60ms before starting the operation again.
 */
#ifdef USART_TIMER_SENSOR_H
#define USART_TIMER_SENSOR_H

#include"stm32f4xx.h"
#include"stm32f4xx_gpio.h"
#include"stm32f4xx_rcc.h"
#include"stm32f4xx_usart.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include <stm32f4xx_tim.h>

void setup_PA3();
void EXTI_PA4_Config(void);
void EXTI_PA4_Config(void);
void delayMS(uint16_t time);
void usart_setup(void);
void USART_Send_Char(unsigned char data);
void Send_String(char str[100]);
void TIM3_Config (void);
void EXTI4_IRQHandler(void);
int sensor();
int ReadSensor();

#endif
