/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\main.c
  * @author  T.O.M.A.S. Team
  * @version V1.1.0
  * @date    14-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"


#include "stm32f0xx_rcc.h"

#include "stm32f0xx_gpio.h"
#include "stdint.h"

#define DATA_PORT GPIOB->ODR
#define CMD_PORT GPIOC->ODR

	#define DATA 6
	#define DATA_CLR 0xFFFFFC3F

	#define LCD_E  0x100     //PA8
	#define LCD_RS 0x200     //PA9

	#define CMD 0
	#define TXT 1

	#define        LINE1    0x80        // Start address of first line
	#define        LINE2    0xC0        // Start address of second line


	void WaitLCDBusy(void);
	void LCD_Init(void);
	void LCD_DATA(unsigned char data,unsigned char type);
	void LCD_NYB(unsigned char nyb,unsigned char type);
	void LCD_STR(const char *text);
	void LCD_LINE(char line);
	void DelayMS(unsigned int ms);

	static uint32_t nowvalue=0;

	void GPIO_Configuration(void);


int main(void)
{
	float ch=4.5;char st[50];
GPIO_Configuration();
LCD_Init();
LCD_LINE(1);
sprintf(st,"%3.1f",ch);
LCD_STR((const char*)"tc755");
while (1);

 }

void LCD_Init()
	{
	    CMD_PORT &= ~(LCD_E);  //LCD_E = 0;                    //clear enable
	    CMD_PORT &= ~(LCD_RS); //LCD_RS = 0;                 //going to write command

	    DelayMS(30);                //delay for LCD to initialise.
	    LCD_NYB(0x30,0);              //Required for initialisation
	    DelayMS(5);                 //required delay
	    LCD_NYB(0x30,0);              //Required for initialisation
	    DelayMS(1);                 //required delay
	    LCD_DATA(0x02,0);           //set to 4 bit interface, 1 line and 5*7 font
	    LCD_DATA(0x28,0);           //set to 4 bit interface, 2 line and 5*10 font
	    LCD_DATA(0x0c,0);           //set to 4 bit interface, 2 line and 5*7 font
	    LCD_DATA(0x01,0);           //clear display
	    LCD_DATA(0x06,0);           //move cursor right after write
	}

	//--------------------------------------------------------------------------------//
	void LCD_DATA(unsigned char data,unsigned char type){

	    WaitLCDBusy();                  //TEST LCD FOR BUSY

	    if(type == CMD){
	        CMD_PORT &=~(LCD_RS);                 //COMMAND MODE
	    } else {
	        CMD_PORT |= LCD_RS;                 //CHARACTER/DATA MODE
	    }

	    LCD_NYB(data>>4,type);               //WRITE THE UPPER NIBBLE
	    LCD_NYB(data,type);                  //WRITE THE LOWER NIBBLE
	}
	//--------------------------------------------------------------------------------//
	 void Delay(__IO uint32_t nCount)
	 {
	   while(nCount--);
	 }
	void WaitLCDBusy(void){
	    DelayMS(2);              //DELAY 1 MilliSeconds
	}
	//--------------------------------------------------------------------------------//
	void LCD_NYB(unsigned char nyb,unsigned char type){
	    DATA_PORT &= DATA_CLR;    //LCD_PORT &= 0xF0;                     //CLEAR LOWER PORT NIBBLE
	    DATA_PORT |= (nyb<<DATA); //LCD_PORT |= (nyb & 0x0F);             //SEND DATA LINE THE INFO

	    if(type == CMD){
	        CMD_PORT &= ~(LCD_RS);                 //COMMAND MODE
	    } else {
	        CMD_PORT |= LCD_RS;                 //CHARACTER/DATA MODE
	    }

	    CMD_PORT |= LCD_E;    //LCD_E = 1;          //ENABLE LCD DATA LINE
	    Delay(100);                //SMALL DELAY
	    CMD_PORT &= ~(LCD_E); //LCD_E = 0;       //DISABLE LCD DATA LINE
	}
	//--------------------------------------------------------------------------------//
	void LCD_STR(const char *text){
	    while(*text){
	        LCD_DATA(*text++,1);
	    }
	}
	void LCD_STR2(unsigned char *text){
	    while(*text)
	    {
	        LCD_DATA(*text++,1);
	    }
	}
	//--------------------------------------------------------------------------------//
	void LCD_LINE(char line){
	    switch(line){
	        case 0:
	        case 1:
	            LCD_DATA(LINE1,0);
	            break;
	        case 2:
	            LCD_DATA(LINE2,0);
	            break;
	    }
	}
	//--------------------------------------------------------------------------------//

	//--------------------------------------------------------------------------------//
	void DelayMS(unsigned int ms){
	    unsigned int x;
	    for(x=0;x<ms;x++)
	        Delay(600);
	}
	//--------------------------------------------------------------------------------//
	//--------------------------------------------------------------------------------//

 void GPIO_Configuration(void)
 {
	 GPIO_InitTypeDef GPIO_InitStructure;
     //Chan PA0-PA3 cho LED; PA4,PA5 cho nut bam; PA8, PA9 cho LCD,
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
     GPIO_Init(GPIOB, &GPIO_InitStructure);
 }
