/*--------------------------------------------------------------------------------*/
/* Project: selfdrivingcar use stm32f407 discovery
 * structure: STM32F407 discovery, servo sg90s, untrasonic sensor srf05, H bridge circuit L298N, Motor DC 12v.
 * IDE: CooCox CoIDE, STLink, hercules 328
 * Language: C
 * Time: 16/6/2024		add: ICTU ThaiNguyen
 * Adviser: Ngo Thi Vinh
 * Coder: Le Minh Huu*, Dinh Qang Ha, Dinh Bach Dang.
 * File: Main. all code. run car ok
 *
 * */
/*--------------------------------------------------------------------------------*/
/*
 * Describe: use GPIO A pin PA3 PA4 pwm control STM32.....
 * Distance can be calculated by using the following formula :- range = high level time * velocity (340m/s) / 2
 * We can also use uS / 58 = Distance in cm or uS / 148 = distance in inch
 * It is recommended to wait for at least 60ms before starting the operation again.

 * Reference docx: reference manual RM900. STM32f4xx.
 * Huong dan thuc hanh, giao trinh lap trinh nhung nang cao ICTU - Ngo Thi Vinh.

 * Pin config: GPIO A
 * PA0: 	->	ENA1 L298 - PWM	.
 * PA1: 	->	ENA2 L298 - PWM	.
 * PA2: 	->	test config- not use	.
 * PA3: 	->	Triger srf-05	- TIM.
 * PA4: 	->	Echo srf - 05	- TIM	.
 * PA5: 	->	Control SERVO SG90S - PWM - TIM2	.
 * PA11: 	->	IN1 L298	.
 * PA12: 	->	IN2 L298	.
 * PA13: 	->	IN3 L298	.
 * PA14: 	->	IN4 L298	.
 */
/*--------------------------------------------------------------------------------*/
/*=================================START================================================*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include <stm32f4xx_tim.h>
#include "misc.h"

#include "configPinOut.h"

//init function control servo sg90
void configPA2();
void TIM2_Setup();
void GPIO_Setup(void);
void delay1(int ms);
void servo_write(uint8_t angle);

// init function control car
void EXTI_PA4_Config(void); //chan echo nhan xung tu chan PA3 (phat xung sieu am). nhan va ngat
void usart_setup(void);
void USART_Send_Char(unsigned char data);
void Send_String(char str[100]);
void TIM3_Config(void);
void delay(uint16_t time);
void delay_1MS(uint16_t time);
void Turn_motor(uint16_t time);
void Turn_motor_speed(uint16_t time, uint16_t time_run);
void Motor_Enable();
void Motor_Disable();
void setValue();
int servo_right();
int servo_left();
void servo_thang();
void haiQuayXe();
void quayXe();
void dieu_khien();
void checkTurn();
void EXTI4_IRQHandler(void);
void readSensor();

/*===================================FUNCTION==================================================*/

void GPIO_Setup(){
    RCC->AHB1ENR |= 1; //Enable GPIOA clock. control SERVO SG90S PIN PA5 PWM
    GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
    GPIOA->MODER |= 0x00000800; //Set the PA5 pin alternate function
}
void TIM2_Setup(){
	//bo chia CFGR cho 1 -> 16mhz. bo chia sau = 4 (TIM2->PSC = 4;). 16mhz = 16 000 000 hz / 4 =  4 000 000 hz trong 1 giay
	//-> 4 000 000 hz / 1 000ms -> 1ms = 4 000 hz (clk)
	//de delay 16ms ->  dem 64 000 -1 -> dem 63 999  (TIM2->ARR = 64000-1;)
	//TIM2->PSC = 4; //Setting the clock frequency to 1MHz.//16-1
	//TIM2->ARR = 64000-1; // Total period of the timer

	 RCC->APB1ENR |=1;
	//bo chia CFGR cho 8 -> 16mhz/8 = 2mhz <=> 2 000 000 hz.
	TIM2->PSC = 2;	//-> 2 000 000 /2 = 1 000 000 hz/ 1s -> 1 000 clk/ 1ms.
	TIM2->ARR = 15999; 	//1 000 clk / 1ms. can 16ms = 16 x 1 000 = 16 000 -> 15 999 lan dem (tinh dem tu 0)
    TIM2->CNT = 0;

    TIM2->CCMR1 = 0x0060; //PWM mode for the timer
    TIM2->CCER |= 1; //Enable channel 1 as output
//    TIM2->CCR1 = 500; // Pulse width for PWM
}

//ham nay chi dung de delay don gian
void delay1(int ms)
{
	int i;
	for(; ms>0 ;ms--){
		for(i =0; i<3000;i++);
	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_write(uint8_t angle)
{
		if(angle<0){angle=0;}
		if(angle>180){angle=180;}
//		TIM2->CCR1=map (angle,0,180,2500,7000); //gia tri phai tinh lai theo tan so da chia
		TIM2->CCR1=map (angle,0,180,500,2500);	// 1000 clk / 1ms. -> min 0 do = 500 clk. max 180 do = 2500 clk
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
	while (time--);
}
void delay_1MS(uint16_t time)
{ // 1MS = 16000
	uint16_t time2 = 1000;
	while (time--)
		while (time2--)
			;
}

void Turn_motor(uint16_t time)
{
	while (time)
	{
		GPIOA->ODR |= (uint16_t)0x001;
		GPIOA->ODR |= (uint16_t)0x002;
//		delay(400); // tren phong la 300
		delay(150);

		GPIOA->ODR &= ~(uint16_t)0x001;
		GPIOA->ODR &= ~(uint16_t)0x002;
//		delay(1000);
		delay(800);
		time--;
	}
}

void Turn_motor_speed(uint16_t time, uint16_t time_run)
{ // ham Turn motor dieu khien toc do mem deo
	while (time)
	{
		Motor_Enable();
		delay(time_run);
		Motor_Disable();
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

void readSensor()
{
	// sensor pin PA3 triger
	GPIOA->ODR &= ~((uint16_t)0x8); // PA3=Trig=0
	delay(32);						// toc do vi dieu khien tren F4 (16MHz) 2us = 32 xung
	GPIOA->ODR |= (uint16_t)0x8;	// PA3=trig=1;
	delay(160);						// delay(160) ~ 10us (F4)
	GPIOA->ODR &= ~((uint16_t)0x8); // trig=0;
}


// chuong trinh con phuc vu ngat
unsigned int Flag_Raising_Falling = 0; //=0 echo high; =1 echo low
unsigned int Num_CLK = 0;
unsigned int num = 0;
unsigned char st[100];
uint8_t go = 0;
uint8_t turn = 1;

uint8_t flagTurn = 1; //0 re trai, 1 re phai
uint8_t turnSquare = 20; //22~45 do
uint8_t now = 0;
uint8_t left = 1;
uint8_t right = 1;
uint8_t leftStone = 0;
uint8_t rightStone = 0;
//4000 cam bien xoay, 1250 cam bien lui

void setValue()
{
	turn = turnSquare/4;
	now = turnSquare * 2;
	leftStone = now - turnSquare;
	rightStone = now + turnSquare;
}

unsigned int pos;
int servo_right()
{
	unsigned int num_right; //do khoang cach ben phai servo quay sang
//	servo quay sang phai
	pos = 0;
	servo_write(pos);
	delay1(100);
	readSensor(); //doc cam bien srf 05
	delay(100);
	num_right = (unsigned int)(TIM6->CNT);	//luu khoang cach ben phai xe
	return num_right;
//	return 1;
}

int servo_left()
{
	unsigned int num_left;	//do khoang cach ben trai servo quay sang
//	servo quay sang trai
	pos = 90;
	servo_write(pos);
	delay1(100);
	readSensor(); //doc cam bien srf 05
	delay(100);
	num_left = (unsigned int)(TIM6->CNT);	//luu khoang cach ben trai xe
	return num_left;
//	return 5;
}
void servo_thang()
{
//	servo quay ve chinh giua di thang
	pos = 50;
	servo_write(pos);
	delay1(50);
}


void haiQuayXe()
{
	unsigned int pos;
	unsigned int num_left = 0;
	unsigned int num_right = 0;
	//do khoang cach vat can
	if (Num_CLK <= 4000 ){
		dung();
		delay_1MS(1);
		//dieu khien servo va do khoang cach
		num_right =(unsigned int) servo_right();
		num_left = (unsigned int)servo_left();
		servo_thang();

		turn = 100;
		if(num_left < num_right)
		{
			//re sang ben phai. khoang cach ben phai xa hon vat can
			dung();
			delay_1MS(2);
			phaiv2();
			Turn_motor_speed(turn, 700);
			delay(300);
			tien();
			Turn_motor_speed(turn, 1000);
			delay(400);
		}else if(num_right < num_left)
		{
			//re sang ben trai. khoang cach ben trai xa hon vat cam
			dung();
			delay_1MS(2);
			traiv2();
			Turn_motor_speed(turn, 700);
			delay(300);
			tien();
			Turn_motor_speed(turn, 1000);
			delay(400);
		}
	}else {
		go = 1;
	}
}

void quayXe()
{

	if (Num_CLK <= 4300){
		dung();
		servo_left();
		servo_right();
		int num_left=3000;
		int num_right=2000;
		if(num_left > num_right )
		{
			delay_1MS(2);
			phai();
			//Turn_motor_speed(turn, 30000);
			now += turn;
			delay(100);
			dung();
			delay(350);
			}
		else if(num_right < num_left)
		{
			delay_1MS(1);
			traiv2();
			Turn_motor_speed(turn, 1500);
			now -= turn;
			delay(100);
			dung();
			delay(350);
		}
	}
	else{
	go = 1;
	}
}



void dieu_khien()
{
	delay(100);
	if (Num_CLK <= 1500){
		lui();
		delay(30000);
		haiQuayXe();
//		quayXe();
	}
	else
		haiQuayXe();
//		quayXe();
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
				//CALL FUNCTION CONTROL CAR TURN LEFT RIGHT
				dieu_khien();
				Flag_Raising_Falling = 0;
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}


/*=================================MAIN================================================*/
int main(void)
{


	setValue();
	uint32_t i;
//	RCC->CFGR = ((uint16_t)0 << 10);
	RCC->CFGR |= ((uint16_t)0x00A << 4); // AHB 2MHz. chi AHB cho 8 -> 2mhz, APB1 KH�NG CHIA -> cung bang 2mhz
	// motor
	ENBLE_Motor();
	config_Pin_Car();

	// Cam bien
	setup_PA3();
	EXTI_PA4_Config();

	// usart
	usart_setup();

	// Time
	TIM3_Config();

	int pos;
	GPIO_Setup();
	TIM2_Setup();
	TIM2->CR1 |= 1; // bat timer 2

	while (1)
	{
		go = 0;
		delay(100);
		readSensor();
		Motor_Enable();
		if (go){
			tien();
			delay(500);
			Turn_motor(20);
			//servo quay thang phai truoc
			pos = 50;
			servo_write(pos);              // tell servo to go to position in variable 'pos'
		}

	}
}
/*====================================END=============================================*/
