/*
 * Power by Le Minh Huu - ICTU _KTMT K19A @copyright
 * Time: 21/2/2024
 * Project: control Car use stm32f407vg dv, L298N
 * IDE: Coocox CoIDE - langage: C
 * Docx: reference manual RM900. STM32f4xx, Huong dan thuc hanh, giao trinh lap trinh nhung nang cao ICTU - Ngo Thi Vinh.
 ** //PIN ENA PA0
 * //PIN ENA PB1
 * //PIN IN1 PB11
 * //PIN IN2 PB12
 * //PIN IN3 PB13
 * //PIN IN4 PB14
 * */
#include "Config_Motor.h"
#include "stm32f4xx.h"

void config_Pin_Speed()
{
	///*cap xung clock cho port A and B. thuoc AHB1. | or voi 0x0...3 -> GPIOAEN, GPIOBEN*/
	RCC->AHB1ENR |= ((uint32_t)0x00000001);

	//PIN ENA PA0
	/*chon che do cho output cho PA0, bit 0 MODER = 1, thuat toan che bit xoa gia tri cu ~0x03 va ghi lai gia tri vao dich trai 4 bit*/
	GPIOA->MODER = (GPIOA->MODER &~((uint32_t)(0x3<< 0))) | (uint32_t)(0x1<<0);
	/*stm32f4.rm281/1745 - cau hinh chan day keo, push-pull bit 2 = 0 OTYPE*/
	GPIOA->OTYPER &= ~(uint16_t)(0x1<<0);
	/*stm32f4.rm282/1745 - config pupdr bit 4, 5 == 00 no pullup, pull down*/
	GPIOA->PUPDR &= ~(uint32_t)(0x3<<0);
	/*stm32f4.rm281/1745 - config speed medium bit 4,5 = 01*/
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~ ((uint32_t)(0x3<<0))) | (uint32_t)(0x3<<0);

	//PIN ENA PA1
	GPIOA->MODER = (GPIOA->MODER & ~((uint32_t)(0x3<<2))) | (uint32_t)(0x1<<2);
	GPIOA->OTYPER &= ~(uint16_t)(0x1<<1);
	GPIOA->PUPDR &= ~(uint32_t)(0x3<<2);
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((uint32_t)(0x3<<2))) | (uint32_t)(0x3<<2);

}

void config_Pin_Car()
{
	RCC->AHB1ENR |= ((uint32_t)0x00000002);
	//PIN IN1 PB11
	GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x3<<22))) | (uint32_t)(0x1<<22);
	GPIOB->OTYPER &= ~(uint16_t)(0x1<<11);
	GPIOB->PUPDR &= ~(uint32_t)(0x3<<22);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~((uint32_t)(0x3<<22))) | (uint32_t)(0x3<<22);

	//PIN IN2 PB12
	GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x3<<24))) | (uint32_t)(0x1<<24);
	GPIOB->OTYPER &= ~(uint16_t)(0x1<<12);
	GPIOB->PUPDR &= ~(uint32_t)(0x3<<24);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~((uint32_t)(0x3<<24))) | (uint32_t)(0x3<<24);

	//PIN IN3 PB13
	GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x03<<26))) | (uint32_t)(0x01<<26);
	GPIOB->OTYPER &= ~(uint16_t)(0x01<<13);
	GPIOB->PUPDR &= ~(uint32_t)(0x03<<26);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~((uint32_t)(0x03<<26))) | (uint32_t)(0x03<<26);

	//PIN IN4 PB14
	GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x03<<28))) | (uint32_t)(0x01<<28);
	GPIOB->OTYPER &= ~(uint16_t)(0x01<<14);
	GPIOB->PUPDR &= ~(uint32_t)(0x03<<28);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~((uint32_t)(0x03<<28))) | (uint32_t)(0x03<<28);
}

void EN_Motor()
{
	GPIOA->ODR |= (uint16_t)(0x1); //PA0
	GPIOA->ODR |= (uint16_t)(0x2);//PA1
}
void DS_Motor(){
	GPIOA->ODR &= ~(uint16_t)(0x1); //PA0
	GPIOA->ODR &= ~(uint16_t)(0x2);//PA1
}
void delay_sp(unsigned int time)
{
	while(time--);
}

void EN_Speed_Delay(unsigned int time, unsigned int EN_speed, unsigned int DS_speed){
	while(time--)
	{
		EN_Motor();	//bat xung trong tg EN_speed
		delay_sp(EN_speed);
		DS_Motor();	//tat xung trong tg DS_speed
		delay_sp(DS_speed);
	}
}


void lui()
{
	/*stm32f4.rm281/1745 - | la or/hoac, giong nhu phep cong. cho bit 4 = 1 ODR*/
	GPIOB->ODR |= (uint16_t)(0x1<<11); //PB11
	/*& la va, phep nhan. cho bit 5 = 0 ODR*/
	GPIOB->ODR &= ~(uint16_t)(0x1<<12); //PB12
	/*| la or/hoac, giong nhu phep cong. cho bit 6 = 1 ODR*/
	GPIOB->ODR |= (uint16_t)(0x1<<13); //PB13
	/*& la va, phep nhan. cho bit 7 = 0 ODR*/
	GPIOB->ODR &= ~(uint16_t)(0x1<<14); //PB14
	EN_Speed_Delay(140, 200, 600);
}
void tien()
{
	GPIOB->ODR &= ~(uint16_t)(0x1<<11); //PB11
	GPIOB->ODR |= (uint16_t)(0x1<<12);	//PB12
	GPIOB->ODR &= ~(uint16_t)(0x1<<13); //PB13
	GPIOB->ODR |= (uint16_t)(0x1<<14); //PB14
	//EN_Speed_Delay(time run, time enable speed, time disable speed)
	EN_Speed_Delay(70, 200, 300);
}
void  phai()
{
	/*tat ben phai*/
	GPIOB->ODR &= ~(uint16_t)(0x0800);
	GPIOB->ODR &= ~(uint16_t)(0x1000);
	/*bat ben trai*/
	GPIOB->ODR &= ~(uint16_t)(0x2000);
	GPIOB->ODR |= (uint16_t)(0x4000);
}
void trai()
{
	/*bat ben phai*/
	GPIOB->ODR &= ~(uint16_t)(0x0800);
	GPIOB->ODR |= (uint16_t)(0x1000);
	/*tat ben trai*/
	GPIOB->ODR &= ~(uint16_t)(0x2000);
	GPIOB->ODR &= ~(uint16_t)(0x4000);
}
void tien10()
{
	GPIOB->ODR &= ~(uint16_t)(0x1<<11); //PB11
	GPIOB->ODR |= (uint16_t)(0x1<<12);	//PB12
	GPIOB->ODR &= ~(uint16_t)(0x1<<13); //PB13
	GPIOB->ODR |= (uint16_t)(0x1<<14); //PB14
	//EN_Speed_Delay(time run, time enable speed, time disable speed)
	EN_Speed_Delay(105, 240, 650);
}
void phai90()
{
	/*tat ben phai*/
	GPIOB->ODR |= (uint16_t)(0x0800);
	GPIOB->ODR &= ~(uint16_t)(0x1000);
	/*bat ben trai*/
	GPIOB->ODR &= ~(uint16_t)(0x2000);
	GPIOB->ODR |= (uint16_t)(0x4000);
	//EN_Speed_Delay(time run, time enable speed, time disable speed)
	EN_Speed_Delay(58, 400, 600);
}
void trai90()
{
	/*tat ben trai*/
	GPIOB->ODR &= ~(uint16_t)(0x0800);
	GPIOB->ODR |= (uint16_t)(0x1000);
	/*bat ben phai*/
	GPIOB->ODR |= (uint16_t)(0x2000);
	GPIOB->ODR &= ~(uint16_t)(0x4000);
	//EN_Speed_Delay(time run, time enable speed, time disable speed)
	EN_Speed_Delay(58, 400, 600);
}

void dung(){
	GPIOB->ODR &= ~(uint16_t)(0x1 <<11);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<12);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<13);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<14);
	EN_Speed_Delay(404, 1, 500);
}
