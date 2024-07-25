/*
 * Power by Le Minh Huu - ICTU _KTMT K19A @copyright
 * Time: 21/2/2024
 * Project: control Car use stm32f407vg dv, L298N
 * IDE: Coocox CoIDE - langage: C
 * Docx: reference manual RM900. STM32f4xx, Huong dan thuc hanh, giao trinh lap trinh nhung nang cao ICTU - Ngo Thi Vinh.
 * file config_motor.h
 * */
#ifndef CONFIG_MOTOR_H
#define CONFIG_MOTOR_H

/*define function and var*/
void config_Pin_Car(); //
void config_Pin_Speed();
void EN_Motor();
void EN_Speed_Delay(unsigned int,unsigned int,unsigned int);
void delay_sp(unsigned int);
void tien();
void lui();
void trai();
void trai90();
void phai();
void phai90();
void dung();
void tien10();

#endif
