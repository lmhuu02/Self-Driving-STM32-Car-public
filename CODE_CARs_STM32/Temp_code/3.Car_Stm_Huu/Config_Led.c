/*
 * Power by Le Minh Huu - ICTU _KTMT K19A @copyright
 * Time: 21/2/2024
 * Project: control Car use stm32f407vg dv, L298N
 * IDE: Coocox CoIDE - langage: C
 * Docx: reference manual RM900. STM32f4xx, Huong dan thuc hanh, giao trinh lap trinh nhung nang cao ICTU - Ngo Thi Vinh.
 * file config_motor.h
 * */
#include "Config_Led.h"

void setup_Led(){
	GPIO_InitTypeDef GPIOD_Conf;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIOD_Conf.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
	GPIOD_Conf.GPIO_Mode = GPIO_Mode_OUT;
	GPIOD_Conf.GPIO_OType = GPIO_OType_PP;
	GPIOD_Conf.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOD_Conf.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOD,&GPIOD_Conf);
}
