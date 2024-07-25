#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"  // thu vien cau hinh chan vao ra
#include "stm32l1xx_rcc.h"  // cung cấp xung clock cho các chân của cổng vào ra

void InitLED(void) //Dieu khien day led don
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//rcc.h


  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3 |GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;// 2 v 10
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//Cong B
  GPIO_InitTypeDef  GPIO_InitB;

    /* Enable the GPIO_LED Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);//rcc.h


    /* Configure the GPIO_LED pin */
    GPIO_InitB.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3;
    GPIO_InitB.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitB.GPIO_OType = GPIO_OType_PP;
    GPIO_InitB.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitB.GPIO_Speed = GPIO_Speed_10MHz;// 2 v 10
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void delay(int ticks)
	{
	  while(ticks--);
	}

void DelayMS(unsigned int ms){
	    unsigned int x;
	    for(x=0;x<ms;x++)
	        delay(600);
	}
void main(){
	int flag=0; int shift,i,j;
	InitLED();
	GPIOB->ODR=0;
	GPIOA->ODR=0x0;
	//GPIOA->ODR=0xFFFF;
	while(1){
		//Doc bit PB0
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==1) flag=1;
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==1) flag=2;
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)==1) flag=3;
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)==1) flag=4;
//dieu khien led don
		if (flag==1)	{
               //Dieu khien sang dan tu trai qua phai
		shift=0x1;
		for (i=0;i<=15;i++){

          GPIOA->ODR=shift ^ 0xFFFF;
          DelayMS(100);
          shift=shift<<1;
		}

		}
		if (flag==2)	{
		//Dieu khien tu phai qua trai sang dan
			shift=0x8000;
		for (i=0;i<=15;i++){

			          GPIOA->ODR=shift ^ 0xFFFF;
			          DelayMS(100);
			          shift=shift>>1;
					}
	}
		if (flag==3){
			//dieu khien led don sang dan tu ngoai vao trong
		shift=0x1;
		 for(i=0;i<=15;i++){
			 GPIOA->ODR=shift ^ 0xFFFF;
			 DelayMS(100);
			 shift=shift<<1^1;
		 }
		 }
		 if (flag==4){
			 // dieu khien led don sang dan tu trong tra ngoai
		shift=0x8000;
		for(i=0;i<=15;i++){
					 GPIOA->ODR=shift ^ 0xFFFF;
					 DelayMS(100);
					 shift=shift>>1^0x8000;
					 		}
		}
	}
}

