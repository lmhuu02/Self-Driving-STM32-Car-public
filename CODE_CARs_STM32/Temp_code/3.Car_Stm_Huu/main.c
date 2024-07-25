
/*===================================START FILE=======================================*/
/*
 * Power by Le Minh Huu - ICTU _KTMT K19A @copyright
 * Time: 21/2/2024
 * Project: control Car use stm32f407vg dv, L298N
 * IDE: Coocox CoIDE - langage: C
 * Docx: reference manual RM900. STM32f4xx, Huong dan thuc hanh, giao trinh lap trinh nhung nang cao ICTU - Ngo Thi Vinh.
 * */
#include "stm32f4xx.h"
#include "Config_Motor.h"
#include "Config_Led.h"
#include "Usart_Timer_Sensor.h"
//#include "var.h"

#define huongTrai 0
#define huongPhai 1

unsigned int khoangcach = 0;
const unsigned int value = 20000;
unsigned char test_stringA1[100];

unsigned int huong;
unsigned int tempKhoangCachTrai = 0;
unsigned int tempKhoangCachPhai = 0;

void delay1(uint16_t);
void test_car();
void quay_xe();
void runCar1();
void runCarAK();
void runCarAFF();
void ServoThang90();
void ServoTrai180();
void ServoPhai0();
void delayServo(int);

void GPIO_Setup(void);
void TIM2_Init(void);
void TIM4_ms_Delay(uint32_t delay);

/*=================================MAIN=========================================*/
int main(void)
{
//	RCC->CFGR |= 0<<10; // set APB1 = 16 MHz code cua servo

	setup_Led();
	config_Pin_Speed();
	config_Pin_Car();

	/*config usart timer sensor*/
	uint16_t  time1=0, time2=0, i;
	unsigned int  numTicks=0;
	RCC->CFGR |= ((uint16_t)0xA<<4); //AHB 1MHz ~ 1 xung / 1us --> lay Timer 1MHz code cua xe
	setup_PA3();
	EXTI_PA4_Config();
	usart_setup();
	TIM3_Config();
	sensor();
//	huong = huongTrai;
	huong = huongTrai;

	//setup servo
	int pos;
//	RCC->CFGR |= 0<<10; // set APB1 = 16 MHz
	GPIO_Setup();
	TIM2_Setup();
	TIM2->CR1 |= 1;

    while(1)
    {
    	pos = 0;
		servo_write(pos);
		delayServo(100);
		pos = 180;
		servo_write(pos);
		delayServo(300);
		pos = 90;
		servo_write(pos);
		delayServo(200);


    	sensor();
    	EN_Motor();
    	tien();
    	runCar1();

//    	delay(50000);
//    	runCarAK();
//    	runCarAFF();
    }
}

/*==================================FUNCTION========================================*/

void GPIO_Setup(){
    RCC->AHB1ENR |= 1; //Enable GPIOA clock
    GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
    GPIOA->MODER |= 0x00000800; //Set the PA5 pin alternate function
}
void TIM2_Setup(){
    RCC->APB1ENR |=1;
    TIM2->PSC = 15; //Setting the clock frequency to 1MHz.//16-1
    TIM2->ARR = 20000-1; // Total period of the timer
    TIM2->CNT = 0;

    TIM2->CCMR1 = 0x0060; //PWM mode for the timer
    TIM2->CCER |= 1; //Enable channel 1 as output
    TIM2->CCR1 = 1500; // Pulse width for PWM
}
void TIM4_ms_Delay(uint32_t delay){
    RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
    TIM4->PSC = 1000-1; //Setting the clock frequency to 1kHz.
    TIM4->ARR = (delayServo); // Total period of the timer
    TIM4->CNT = 0;
    TIM4->CR1 |= 1; //Start the Timer
    while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
    TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
}


void delayServo(int ms){
//	int i;
//	for(; ms>0 ;ms--){
//		for(i =0; i<3195;i++);
//	}
	TIM4_ms_Delay(ms);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_write(uint8_t angle)
{
	if(angle<0){angle=0;}
	if(angle>180){angle=180;}
	TIM2->CCR1=map (angle,0,180,2500,7000);
}

void delay1(uint16_t time)
{
	while(time--);
}
void test_car()
{
	//		EN_Speed_Delay(100,300,700);

			//call function test sensor
	//    	khoangcach = sensor();
	//    	sprintf(test_stringA1, "Distance FIX: %d mm\r\n", khoangcach);
	//    	Send_String(test_stringA1);
	//test xoaytrai phai
    	trai();
    	EN_Speed_Delay(100,300,700);
    	phai();
    	EN_Speed_Delay(100,300,700);
    	trai90();
		phai90();

	//EN_Speed_Delay(time delay, time enable, time disable);
	    	tien();
	    	EN_Speed_Delay(1000,300,700);
	    	lui();
	    	khoangcach = sensor();
	    	EN_Speed_Delay(500,300,700);
	    	trai();
	    	EN_Speed_Delay(1000,400,700);
			phai();
			EN_Speed_Delay(500,400,700);
			dung();
			EN_Speed_Delay(1000,4,700);
}

void runCarAFF()
{
	if(ReadSensor() < 40000)
	{
		if(huong == huongTrai)
		{
			trai90();
			dung();

			if( ReadSensor() < 10000)
			{
				lui();
				phai90();
				phai90();
				tien10();
				trai90();
				huong = huongTrai;
				if(ReadSensor() < 10000 )
				{
					dung();
					trai90();
					dung();
				}
			}else if(ReadSensor() > 10000 && ReadSensor() < 40000)
			{
				tien10();
				if(ReadSensor() < 10000)
				{
					dung();
					phai90();
					dung();
					huong = huongPhai;
				}
			}
		}else if(huong == huongPhai)
		{
			dung();
			phai90();
			dung();
			if(ReadSensor() < 10000)
			{
				lui();
				phai90();
				phai90();
				tien10();
				huong = huongPhai;
				if(ReadSensor() < 10000)
				{
					dung();
					phai90();
					tien10();
					dung();
				}
			}else if(ReadSensor() > 10000 && ReadSensor() < 40000)
			{
				tien10();
				if(ReadSensor() < 10000)
				{
					dung();
					trai90();
					tien10();
					dung();
				}
				huong = trai;
			}
		}
	}
}

void runCarAK()
{
	//xoay sang trai
	if(huong == huongTrai)
	{
		if( ReadSensor() < 40000)
		{
			//quay xe sang trai
			trai90();
			dung();
			if( ReadSensor() < 20000)
			{
				lui();
			}else{
				tien10();
			}
			dung();
			phai90();
			dung();
			huong = huongPhai;
		}
	}
	//xoay sang phai
	if(huong == huongPhai)
	{
		if( ReadSensor() < 40000)
		{
			//quay xe sang phai
			phai90();
			dung();
			if( ReadSensor() < 20000)
			{
				lui();
			}else{
				tien10();
			}
			dung();
			trai90();
			dung();
			huong = huongTrai;
		}
	}

}

void runCar1()
{
	//xoay sang trai
	if(huong == huongTrai)
	{
		if( ReadSensor() < 30000)
		{
			if( ReadSensor() < 20000)
			{
//				lui();
//				EN_Speed_Delay(50, 200, 600);
			}
			//quay xe sang trai
			trai90();
//			EN_Speed_Delay(50, 400, 600);
//			tien10();
//			while (ReadSensor() > 20000)
//			{
				tien10();
//			}

			dung();
			phai90();
			huong = huongPhai;
		}
	}
	//xoay sang phai
	if(huong == huongPhai)
	{
		if( ReadSensor() < 30000)
		{
			if( ReadSensor() < 20000)
			{
//				lui();
//				EN_Speed_Delay(50, 200, 600);
			}
			//quay xe sang phai
			phai90();
//			tien10();
//			while (ReadSensor() > 20000)
//			{
				tien10();
//			}

			dung();
			trai90();
			huong = huongTrai;
		}
	}
}

/*===================================END FILE=======================================*/

