
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

/*=================================MAIN=========================================*/
int main(void)
{
	setup_Led();
	config_Pin_Speed();
	config_Pin_Car();

	/*config usart timer sensor*/
	uint16_t  time1=0, time2=0, i;
	unsigned int  numTicks=0;
	RCC->CFGR |= ((uint16_t)0xA<<4); //AHB 1MHz ~ 1 xung / 1us --> lay Timer 1MHz
	setup_PA3();
	EXTI_PA4_Config();
	usart_setup();
	TIM3_Config();
	sensor();
//	huong = huongTrai;
	huong = huongTrai;

    while(1)
    {
    	sensor();
    	EN_Motor();
    	tien();
//    	delay(50000);
//    	runCarAK();
    	runCar1();
//    	runCarAFF();
    }
}

/*==================================FUNCTION========================================*/
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
//				if(ReadSensor() < 10000 )
//				{
					dung();
					trai90();
					dung();
//				}
			}else if(ReadSensor() > 10000 && ReadSensor() < 40000)
			{
				tien10();
//				if(ReadSensor() < 10000)
//				{
					dung();
					phai90();
					dung();
					huong = huongPhai;
//				}
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
//				if(ReadSensor() < 10000)
//				{
					dung();
					phai90();
					tien10();
					dung();
//				}
			}else if(ReadSensor() > 10000 && ReadSensor() < 40000)
			{
				tien10();
//				if(ReadSensor() < 10000)
//				{
					dung();
					trai90();
					tien10();
					dung();
//				}
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

