/*
 * Distance can be calculated by using the following formula :- range = high level time * velocity (340m/s) / 2
We can also use uS / 58 = Distance in cm or uS / 148 = distance in inch
It is recommended to wait for at least 60ms before starting the operation again.
 */

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include <stm32f4xx_tim.h>

// Thi du cau hinh chan PA2 la chan ra
void configPA2()
{
	// Cho phep xung nhip cong A, dat bit 0 trong thanh ghi AHB1ENR bang 1
	RCC->AHB1ENR |= ((uint32_t)0x00000001);

	// Cau hinh chan PA2

	// Thanh ghi Moder cong A: bit 5, bit 4 = 01 - che do ra
	GPIOA->MODER = (GPIOA->MODER & ~((uint32_t)(0x3 << 4))) | (uint32_t)(0x1 << 4);
	// Thanh ghi OTYPE cong A: Bit 2 = 0 - chon che do day keo
	GPIOA->OTYPER &= ~(uint16_t)(0x1 << 2);
	// Thanh ghi PUPDR cong A: bit 5, bit 4 = 00 - khong dung tro keo
	GPIOA->PUPDR &= ~(uint32_t)(0x3 << 4);
	// Thanh ghi OSPEEDR cong A: Bit 5, bit 4 = 10 - Toc do
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((uint32_t)(0x3 << 2))) | (uint32_t)(0x1 << 2);
}

void ENBLE_Motor()
{
	RCC->CFGR |= 0 << 10; // set APB1 = 16 MHz

	RCC->AHB1ENR |= ((uint32_t)0x00000001);
	// PA0
	GPIOA->MODER = (GPIOA->MODER & ~((uint32_t)(0x3 << 0))) | (uint32_t)(0x1 << 0);
	GPIOA->OTYPER &= ~(uint16_t)(0x1 << 0);
	GPIOA->PUPDR &= ~(uint32_t)(0x3 << 0);
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((uint32_t)(0x3 << 0))) | (uint32_t)(0x1 << 0);
	// PA1
	GPIOA->MODER = (GPIOA->MODER & ~((uint32_t)(0x3 << 2))) | (uint32_t)(0x1 << 2);
	GPIOA->OTYPER &= ~(uint16_t)(0x1 << 1);
	GPIOA->PUPDR &= ~(uint32_t)(0x3 << 2);
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((uint32_t)(0x3 << 2))) | (uint32_t)(0x1 << 2);
}
void TIM2_Setup()
{
	RCC->APB1ENR |= 1;	// Enable clock TIM2
	TIM2->PSC = 16 - 1; // Setting the clock frequency to 1MHz.
	TIM2->ARR = 20000;	// Total period of the timer
	TIM2->CNT = 0;

	// Chennal 1
	TIM2->CCMR1 = 0x0060; // PWM mode for the timer
	TIM2->CCER |= 1;	  // Enable channel 1 as output
	TIM2->CCR1 = 500;	  // Pulse width for PWM

	// Chennal 2
	//    TIM2->CCMR1 = 0x6000; //PWM mode for the timer
	//    TIM2->CCER |= 0x10; //Enable channel 2 as output
	//    TIM2->CCR2 = 500; // Pulse width for PWM
}

void Config_Motor()
{

	RCC->AHB1ENR |= ((uint32_t)0x00000002);

	// PB11
	GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x3 << 22))) | (uint32_t)(0x1 << 22);
	GPIOB->OTYPER &= ~(uint16_t)(0x1 << 11);
	GPIOB->PUPDR &= ~(uint32_t)(0x3 << 22);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~((uint32_t)(0x3 << 22))) | (uint32_t)(0x1 << 22);

	// PB12

	GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x3 << 24))) | (uint32_t)(0x1 << 24);
	GPIOB->OTYPER &= ~(uint16_t)(0x1 << 12);
	GPIOB->PUPDR &= ~(uint32_t)(0x3 << 24);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(uint32_t)(0x3 << 24)) | (uint32_t)(0x1 << 24);

	// PB13
	GPIOB->MODER = (GPIOB->MODER & ~(uint32_t)(0x3 << 26)) | (uint32_t)(0x1 << 26);
	GPIOB->OTYPER &= ~(uint16_t)(0x1 << 13);
	GPIOB->PUPDR &= ~(uint32_t)(0x3 << 26);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(uint32_t)(0x3 << 26)) | (uint32_t)(0x1 << 26);

	// PB14
	GPIOB->MODER = (GPIOB->MODER & ~(uint32_t)(0x3 << 28)) | (uint32_t)(0x1 << 28);
	GPIOB->OTYPER &= ~(uint16_t)(0x1 << 14);
	GPIOB->PUPDR &= ~(uint32_t)(0x3 << 28);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(uint32_t)(0x3 << 28)) | (uint32_t)(0x1 << 28);
}

void setup_PA3()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void EXTI_PA4_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIOA clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Connect EXTI0 Line to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void usart_setup(void)
{
	// USART1: PA2 - Tx; PA3-Rx
	USART_InitTypeDef USART1_S;
	GPIO_InitTypeDef GPIOA_S;

	RCC_AHB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART1);

	GPIOA_S.GPIO_Pin = GPIO_Pin_2; //| GPIO_Pin_3;
	GPIOA_S.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOA_S.GPIO_Mode = GPIO_Mode_AF;
	GPIOA_S.GPIO_OType = GPIO_OType_PP;
	GPIOA_S.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIOA_S);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART1_S.USART_BaudRate = 115200;
	USART1_S.USART_WordLength = USART_WordLength_8b;
	USART1_S.USART_StopBits = USART_StopBits_1;
	USART1_S.USART_Parity = USART_Parity_No;
	USART1_S.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1_S.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART1_S);
	USART_Cmd(USART2, ENABLE);
}
//
void USART_Send_Char(unsigned char data)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART2, data);
}

void Send_String(char str[100])
{
	int i;
	for (i = 0; i < strlen(str); i++)
	{
		USART_Send_Char(str[i]);
	}
}

void TIM3_Config(void)
{
	// NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;  // he so chia toc do counter - AHB/1 = 1MHz
	TIM_TimeBaseStructure.TIM_Period = 60000; // autoreload ARR
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	// TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);  //cho phep ngat timer
	// TIM_Cmd(TIM3, ENABLE); //cho phep timer
	// NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelPriority=0;
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);
}

void delay(uint16_t time)
{
	while (time--)
		;
}
void delay_1MS(uint16_t time)
{ // 1MS = 16000
	uint16_t time2 = 1000;
	while (time--)
		while (time2--)
			;
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
void Turn_motor(uint16_t time)
{
	while (time)
	{
		GPIOA->ODR |= (uint16_t)0x001;
		GPIOA->ODR |= (uint16_t)0x002;
		delay(400); // tren phong la 300

		GPIOA->ODR &= ~(uint16_t)0x001;
		GPIOA->ODR &= ~(uint16_t)0x002;
		delay(1000);
		time--;
	}
}

void Turn_motor_speed(uint16_t time, uint16_t time_run)
{ // ham Turn motor dieu khien toc do mem deo
	while (time)
	{
		GPIOA->ODR |= (uint16_t)0x001;
		GPIOA->ODR |= (uint16_t)0x002;
		delay(time_run);
		GPIOA->ODR &= ~(uint16_t)0x001;
		GPIOA->ODR &= ~(uint16_t)0x002;
		delay(1000);
		time--;
	}
}

void Motor_Enable()
{
	GPIOA->ODR |= (uint16_t)0x001;
	GPIOA->ODR |= (uint16_t)0x002;
}
void Motor_Disable()
{
	GPIOA->ODR &= ~(uint16_t)0x001;
	GPIOA->ODR &= ~(uint16_t)0x002;
}

// chuong trinh con phuc vu ngat
unsigned int Flag_Raising_Falling = 0; //=0 echo high; =1 echo low
uint32_t Num_CLK = 0;
unsigned int num = 0;
unsigned char st[100];



uint8_t flagTurn = 1; //0 re trai, 1 re phai
uint8_t turnSquare = 20; //22~45 do
uint8_t turn = 0;
uint8_t now = 0;
uint8_t left = 1;
uint8_t right = 1;
uint8_t leftStone = 0;
uint8_t rightStone = 0;
//4000 cam bien xoay, 1250 cam bien lui

void setValue(){
	turn = turnSquare/4;
	now = turnSquare * 2;
	leftStone = now - turnSquare;
	rightStone = now + turnSquare;
}

uint8_t go = 0;

void quayXe(){
	if (Num_CLK <= 4300 && flagTurn == 1 && right == 1){
		dung();
		delay_1MS(1);
		phaiv2();
		Turn_motor_speed(turn, 1500);
		now += turn;
		delay(100);
		dung();
		delay(350);
	}
	else if (Num_CLK <= 4300 && flagTurn == 0 && left == 1){
		dung();
		delay_1MS(1);
		traiv2();
		Turn_motor_speed(turn, 1500);
		now -= turn;
		delay(100);
		dung();
		delay(350);
	}
	else
	go = 1;
}

void dieu_khien()
{
	checkTurn();
	delay(100);
	if (Num_CLK <= 1500){
		lui();
		delay(30000);
		quayXe();
	}
	else
		quayXe();
}

void checkTurn(){
	if (now - turn < leftStone){
		left = 0;
		flagTurn = 1;
	}
	else if (right == 0)
		left = 1;

	if (now + turn > rightStone){
		right = 0;
		flagTurn = 0;
	}
	else if (left == 0)
		right = 1;
}



void EXTI4_IRQHandler(void)
{
	// PA4
	if (EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		if (Flag_Raising_Falling == 0)
		{
			Flag_Raising_Falling = 1;
			Num_CLK = 0;
			TIM6->CNT = 0;
			TIM_Cmd(TIM6, ENABLE);
		}
		else
		{
			if (Flag_Raising_Falling == 1)
			{
				Num_CLK = TIM6->CNT;
				TIM_Cmd(TIM6, DISABLE);
				dieu_khien();
				Flag_Raising_Falling = 0;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

void readSensor()
{
	// read sensor
	GPIOA->ODR &= ~((uint16_t)0x8); // PA3=Trig=0
	delay(32);						// toc do vi dieu khien tren F4 (16MHz) 2us = 32 xung
	GPIOA->ODR |= (uint16_t)0x8;	// PA3=trig=1;
	delay(160);						// delay(160) ~ 10us (F4)
	GPIOA->ODR &= ~((uint16_t)0x8); // trig=0;
}

int main(void)
{

	setValue();

	uint32_t i;

	RCC->CFGR |= ((uint16_t)0xA << 4); // AHB 1MHz ~ 1 xung / 1us --> lay Timer 1MHz
	// motor
	ENBLE_Motor();
	Config_Motor();

	// Cam bien
	setup_PA3();
	EXTI_PA4_Config();

	// usart
	usart_setup();

	// Time
	TIM3_Config();

	// Tao da day motor tien

	while (1)
	{
		go = 0;
		delay(100);
		readSensor();
		Motor_Enable();
		if (go){
			tien();
			delay(500);
			Turn_motor(7);
		}

	}
}
