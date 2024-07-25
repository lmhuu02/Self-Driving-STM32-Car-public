/*
 * Power by Le Minh Huu - ICTU _KTMT K19A @copyright
 * Time: 6/2024
 * Project: selfdrivingcar use stm32f407 discovery
 * IDE: Coocox CoIDE - langage: C
 * Docx: reference manual RM900. STM32f4xx.
 * Huong dan thuc hanh, giao trinh lap trinh nhung nang cao ICTU - Ngo Thi Vinh.
 * Describe: file self-created library .h input -> configPinOut.c input
 * 												-> file main.c
 * file configPinOut.h
 *
 * */
#ifndef CONFIGPINOUT_H
#define CONFIGPINOUT_H

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void ENBLE_Motor();
void config_Pin_Car();
void lui();
void tien();
void traiv2();
void trai();
void trailui();
void phai();
void phaiv2();
void phailui();
void dung();
void setup_PA3();

#endif
