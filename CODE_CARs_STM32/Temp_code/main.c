#include"stm32f4xx.h"
#include"stm32f4xx_gpio.h"
#include"stm32f4xx_rcc.h"
#include"stm32f4xx_usart.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include <stm32f4xx_tim.h>


#include <stdio.h>
#include <string.h>

void ENBLE_Motor();
void Config_Motor();

void delay(uint16_t time);
void delay_1MS(uint16_t time);
void delayMS(uint16_t time);

void lui();
void tien();
void trai();
void phai();
void dung();
void Turn_motor(uint16_t time);
void Motor_Enable();
void Motor_Disable();

/*------------------------config cam bien-------------------------*/
void setup_PA3();
void EXTI_PA4_Config(void);
void usart_setup(void);
void USART_Send_Char(unsigned char data);
void Send_String(char str[100]);
void TIM3_Config (void);

/*------------------------config cam bien-------------------------*/


/*----------------------dieu kien xe chay-------------------------*/
void dieu_khien();

/*----------------------dieu kien xe chay-------------------------*/


/*------------------------bien toan cuc---------------------------*/
int Flag_Raising_Falling = 0; //=0 echo high; =1 echo low
unsigned int  Num_CLK=0;
char st[100];
uint8_t flashRL=0;
/*------------------------bien toan cuc---------------------------*/


void ENBLE_Motor(){
	RCC->CFGR |= 0<<10;   // set APB1 = 16 MHz

	RCC->AHB1ENR |= ((uint32_t)0x00000001);


	// PA0
	GPIOA->MODER = (GPIOA->MODER & ~((uint32_t)(0x3 << 0))) | (uint32_t)(0x1 << 0);
	GPIOA->OTYPER &= ~(uint16_t)(0x1 << 0) ;
	GPIOA->PUPDR &= ~(uint32_t)(0x3 << 0) ;
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((uint32_t)(0x3 << 0))) | (uint32_t)(0x1 << 0);
	// PA1
	GPIOA->MODER = (GPIOA->MODER & ~((uint32_t)(0x3 << 2))) | (uint32_t)(0x1 << 2);
	GPIOA->OTYPER &= ~(uint16_t)(0x1 << 1) ;
	GPIOA->PUPDR &= ~(uint32_t)(0x3 << 2) ;
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((uint32_t)(0x3 << 2))) | (uint32_t)(0x1 << 2);
}

void Config_Motor(){


		  RCC->AHB1ENR |= ((uint32_t)0x00000002);

		  // PB11
		  GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x3 << 22))) | (uint32_t)(0x1 << 22);
		  GPIOB->OTYPER &= ~(uint16_t)(0x1 << 11) ;
		  GPIOB->PUPDR &= ~(uint32_t)(0x3 << 22) ;
		  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~((uint32_t)(0x3 << 22))) | (uint32_t)(0x1 << 22);

		  //PB12

		  GPIOB->MODER = (GPIOB->MODER & ~((uint32_t)(0x3 << 24))) | (uint32_t)(0x1 << 24);
		  GPIOB->OTYPER &= ~(uint16_t)(0x1 << 12) ;
		  GPIOB->PUPDR &= ~(uint32_t)(0x3 << 24) ;
		  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(uint32_t)(0x3 << 24)) | (uint32_t)(0x1 << 24);

		 // PB13
			GPIOB->MODER = (GPIOB->MODER & ~(uint32_t)(0x3 << 26)) | (uint32_t) (0x1 << 26);
			GPIOB->OTYPER &= ~(uint16_t)(0x1 << 13) ;
			GPIOB->PUPDR &= ~(uint32_t)(0x3 << 26) ;
			GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(uint32_t)(0x3 << 26))| (uint32_t) (0x1 << 26);

		// PB14
			GPIOB->MODER = (GPIOB->MODER & ~(uint32_t)(0x3 << 28)) | (uint32_t) (0x1 << 28);
			GPIOB->OTYPER &= ~(uint16_t)(0x1 << 14) ;
			GPIOB->PUPDR &= ~(uint32_t)(0x3 << 28) ;
			GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(uint32_t)(0x3 << 28)) | (uint32_t) (0x1 << 28);

}

void delay(uint16_t time){
	while(time--);
}
void delay_1MS(uint16_t time){ // 1MS = 16000
	uint16_t  time2=1000;
	while(time--)
		while (time2--);
}
void lui(){
	GPIOB->ODR |= (uint16_t)(0x1 <<11);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<12);
	GPIOB->ODR |= (uint16_t)(0x1 <<13);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<14);
}
void tien(){
	GPIOB->ODR &= ~(uint16_t)(0x1 <<11);
	GPIOB->ODR |= (uint16_t)(0x1 <<12);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<13);
	GPIOB->ODR |= (uint16_t)(0x1 <<14);
}
void trai(){
	GPIOB->ODR &= ~(uint16_t)(0x1 <<11);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<12);
	GPIOB->ODR |= (uint16_t)(0x1 <<13);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<14);
}
void phai(){
	GPIOB->ODR &= ~(uint16_t)(0x1 <<11);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<12);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<13);
	GPIOB->ODR |= (uint16_t)(0x1 <<14);
}
void dung(){
	GPIOB->ODR &= ~(uint16_t)(0x1 <<11);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<12);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<13);
	GPIOB->ODR &= ~(uint16_t)(0x1 <<14);
}
void Turn_motor(uint16_t time){
	while(time){
	GPIOA->ODR |=(uint16_t)0x001;
	GPIOA->ODR |=(uint16_t)0x002;
	delay(300);
	GPIOA->ODR &=~(uint16_t)0x001;
	GPIOA->ODR &=~(uint16_t)0x002;
	delay(1000);
	time--;
	}
}
void Turn_motor_speed(uint16_t time,uint16_t time_speed){
	while(time){
	GPIOA->ODR |=(uint16_t)0x001;
	GPIOA->ODR |=(uint16_t)0x002;
	delay(time_speed);
	GPIOA->ODR &=~(uint16_t)0x001;
	GPIOA->ODR &=~(uint16_t)0x002;
	delay(1000);
	time--;
	}
}
void Motor_Enable(){

	GPIOA->ODR |=(uint16_t)0x001;
	GPIOA->ODR |=(uint16_t)0x002;
}
void Motor_Disable(){

	GPIOA->ODR &=~(uint16_t)0x001;
	GPIOA->ODR &=~(uint16_t)0x002;
}

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


void EXTI_PA4_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

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

void delayMS(uint16_t time){
	uint32_t i;
	while(time--){
		for (i=0;i<1000;i++); //1s
	}
}

void usart_setup(void)
{
	// USART1: PA2 - Tx; PA3-Rx
	USART_InitTypeDef USART1_S;
	GPIO_InitTypeDef GPIOA_S ;

	RCC_AHB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_USART2);

	GPIOA_S.GPIO_Pin    = GPIO_Pin_2 ;//| GPIO_Pin_3;
	GPIOA_S.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOA_S.GPIO_Mode    = GPIO_Mode_AF;
	GPIOA_S.GPIO_OType = GPIO_OType_PP;
	GPIOA_S.GPIO_PuPd    = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIOA_S);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	USART1_S.USART_BaudRate = 115200;
	USART1_S.USART_WordLength = USART_WordLength_8b;
	USART1_S.USART_StopBits = USART_StopBits_1;
	USART1_S.USART_Parity = USART_Parity_No;
	USART1_S.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1_S.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART1_S);
	USART_Cmd(USART2,ENABLE);
}
//
void USART_Send_Char(unsigned char data)
{
	  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)==RESET);
	  USART_SendData(USART2,data);
}

void Send_String(char str[100]){

	int i;
	for (i=0;i<strlen(str); i++){
		USART_Send_Char(str[i]);
	}
}
void TIM3_Config (void)
 {
// NVIC_InitTypeDef NVIC_InitStructure;
 TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;

 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 TIM_TimeBaseStructure.TIM_Prescaler = 0;// he so chia toc do counter - AHB/1 = 1MHz
 TIM_TimeBaseStructure.TIM_Period = 60000; //autoreload ARR
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

}

void dieu_khien(){
	if(Num_CLK <2500 && flashRL==0){
		Send_String("\ndung--\r\n");
		ENBLE_Motor();
		dung();
		delay(15);
		trai();
		Turn_motor_speed(10,600);
		ENBLE_Motor();
		delay(16000);
		tien();
		Turn_motor(1000);
		ENBLE_Motor();
		phai();
		Turn_motor_speed(10,600);
		flashRL=1;
	}
	if(Num_CLK <2500 && flashRL==1){
			Send_String("\ndung--\r\n");
			ENBLE_Motor();
			dung();
			delay(15);
			phai();
			Turn_motor_speed(10,600);
			ENBLE_Motor();
			delay(16000);
			tien();
			Turn_motor(1000);
			ENBLE_Motor();
			trai();
			Turn_motor_speed(10,600);
			flashRL=0;
	}

}

//chuong trinh con phuc vu ngat
void EXTI4_IRQHandler(void)
{
  //PA4
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
	 if ( Flag_Raising_Falling ==0){
    /* Toggle LED8 */
		 GPIOC->ODR=(uint16_t)0x0100;
		 Flag_Raising_Falling =1;
		 Num_CLK=0;
		 TIM3->CNT=0;
		 TIM_Cmd(TIM3, ENABLE);
	 }else{
	 if ( Flag_Raising_Falling ==1){
	     /* Toggle LED8 */
	 		 GPIOC->ODR=(uint16_t)0x0300;
	 		 /* Clear the EXTI line 0 pending bit */
	 		Num_CLK=TIM3->CNT;
	 		TIM_Cmd(TIM3, DISABLE);
//	 		Distance = Num_CLK*340/50;
	 		sprintf(st, "Distance: %d cm\r\n", Num_CLK);// (elapsed time * speed of sound (340 m / s)) / 100 / 2
	 		// S = V*t = van toc khong khi * t (doi xung ra giay)
//	 		st="ok";
	 		Send_String(st);
	 		Flag_Raising_Falling =0;
	 		dieu_khien();
	 }
	 }
	 EXTI_ClearITPendingBit(EXTI_Line4);
}
}

uint32_t i=0;
int main(void)
{

		RCC->CFGR &= ~RCC_CFGR_PPRE1_DIV16;   // set APB1 = 16 MHz
		ENBLE_Motor();
	    Config_Motor();

	    RCC->CFGR |= ((uint16_t)0xB<<4); //AHB 1MHz ~ 1 xung / 1us --> lay Timer 1MHz
		setup_PA3();
		EXTI_PA4_Config();
		usart_setup();
		TIM3_Config();

						Motor_Enable();
						delay(16000);
//						tien();
//			    		Turn_motor(6000);
//			    		trai();
//			    		Motor_Enable();
//			    		delay_1MS(16);
			    		tien();
//			    		Turn_motor(6000);
	    while(1){
	    	for(i=0;i<1000;i++){
	    		Turn_motor(1);
	    	}
	    	Send_String("bat usart:----- \r\n");
//	    	TIM3->CNT = 0;
	    	Flag_Raising_Falling = 0;

			//read sensor
    		GPIOA->ODR &= ~((uint16_t)0x8); 	//PA3=Trig=0
            delay(32);  //toc do vi dieu khien 8MHz (F0), 1us ~ 8 xung -> 2us ~ 16 xung; tren F4 (16MHz) 2us = 32 xung

            GPIOA->ODR |= (uint16_t)0x8; 		//PA3=trig=1;
            delay(160); //  delay 10us (F0); delay(160) (F4)
            GPIOA->ODR &= ~((uint16_t)0x8); 	//trig=0;
//
//            sprintf(st,"\nkhoang cach = %d---\r\n",Num_CLK);
//            Send_String(st);
//            dieu_khien();
//            delayMS(1000);
	   }
}
