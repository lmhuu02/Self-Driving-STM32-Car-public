/*
 * Power by Le Minh Huu - ICTU _KTMT K19A @copyright
 * Time: 6/2024
 * Project: selfdrivingcar use stm32f407 discovery
 * IDE: Coocox CoIDE - langage: C
 * Docx: reference manual RM900. STM32f4xx.
 * Huong dan thuc hanh, giao trinh lap trinh nhung nang cao ICTU - Ngo Thi Vinh.
 * Describe: file self-created library .h input -> configPinOut.c input
 * 												-> file main.c
 * file configPinOut.c
 * */

#include "configPinOut.h"

void ENBLE_Motor()
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

//11 12 ben phai, 13 14 ben trai
//12 14 la tien, 11 13 la lui
void lui()
{
	GPIOB->ODR |= (uint16_t)(0x1 << 11);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 12);
	GPIOB->ODR |= (uint16_t)(0x1 << 13);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 14);
}
void tien()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 11);
	GPIOB->ODR |= (uint16_t)(0x1 << 12);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 13);
	GPIOB->ODR |= (uint16_t)(0x1 << 14);
}
void traiv2()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 11);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 14);
	delay(300);
	GPIOB->ODR |= (uint16_t)(0x1 << 12);
	GPIOB->ODR |= (uint16_t)(0x1 << 13);
}
void trai()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 11);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 13);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 14);
	delay(300);
	GPIOB->ODR |= (uint16_t)(0x1 << 12);
}
void trailui()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 12);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 11);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 14);
	delay(300);
	GPIOB->ODR |= (uint16_t)(0x1 << 13);
}
void phai()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 11);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 12);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 13);
	delay(300);
	GPIOB->ODR |= (uint16_t)(0x1 << 14);
}
void phaiv2()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 12);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 13);
	delay(300);
	GPIOB->ODR |= (uint16_t)(0x1 << 14);
	GPIOB->ODR |= (uint16_t)(0x1 << 11);
}
void phailui()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 12);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 13);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 14);
	delay(300);
	GPIOB->ODR |= (uint16_t)(0x1 << 11);
}
void dung()
{
	GPIOB->ODR &= ~(uint16_t)(0x1 << 11);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 12);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 13);
	GPIOB->ODR &= ~(uint16_t)(0x1 << 14);
}

//chan triger tao xung phat song sieu am
void setup_PA3(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

