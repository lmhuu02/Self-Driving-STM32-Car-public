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
#include "stm32f30x.h"


#include "stm32f30x_rcc.h"

#include "stm32f30x_gpio.h"
/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define DATA_PORT GPIOA->ODR
#define CMD_PORT GPIOB->ODR

	#define DATA 0
	#define DATA_CLR 0xFFFFFC3F

	#define LCD_E  0x1     //PA8
	#define LCD_RS 0x2     //PA9

	#define CMD 0
	#define TXT 1

	#define        LINE1    0x80        // Start address of first line
	#define        LINE2    0xC0        // Start address of second line


/* Private functions ---------------------------------------------------------*/

/* Task functions declarations */

void delay(void)
    {
        int time;
        for(time=0;time<4000000;time++);// tre khoang 4000000 lenh
    }


void gpio_setup(void)
   {
    // khai bao GPIO_InitStructure = GPIO_InitTypeDef
    // 2 cach viet la nhu nhau
    GPIO_InitTypeDef        GPIO_InitStructure;
        /* cap xung cho PC */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

  /* cau hinh PC9 and PC8 la dau ra, khong dung tro keo */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_6|GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;// dung PC9,PC8

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// che do uotput
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// reset pin khi uotput
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// clock 50Mhz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// khong dung tro keo
  GPIO_Init(GPIOE, &GPIO_InitStructure);// khoi tao PC

    }


	  //Include main header for MCU

	void delay1(int ticks)
	{
	  while(ticks--);
	}

	void lcd_Init()
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
	        CMD_PORT &= ~(LCD_RS);                 //COMMAND MODE
	    } else {
	        CMD_PORT |= LCD_RS;                 //CHARACTER/DATA MODE
	    }

	    LCD_NYB(data>>4,type);               //WRITE THE UPPER NIBBLE
	    LCD_NYB(data,type);                  //WRITE THE LOWER NIBBLE
	}
	//--------------------------------------------------------------------------------//
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
	    delay1(100);                //SMALL DELAY
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
	        delay1(600);
	}
	//--------------------------------------------------------------------------------//
	//--------------------------------------------------------------------------------//
	void initDiscovery(void)
	{
	  RCC->CFGR = RCC_CFGR_SW_HSE;
	  //RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOCEN;
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	  RCC->CR |= RCC_CR_HSEON;

	  while((RCC->CR & RCC_CR_HSERDY)==0);

	  GPIOA->MODER=GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0|GPIO_MODER_MODER2_0|GPIO_MODER_MODER3_0;

	  GPIOB->MODER=GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0;

	  GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0_1 | GPIO_OSPEEDER_OSPEEDR1_1 | GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR3_1;

	  GPIOB->OSPEEDR = GPIO_OSPEEDER_OSPEEDR0_1 | GPIO_OSPEEDER_OSPEEDR1_1;
	}

int main(void)
{

	 gpio_setup();
	 initDiscovery();

	  lcd_Init();
	  LCD_LINE(1);

	 LCD_STR((const char*)"NGUYEN MINH CHI ");
	 GPIOE->ODR=0xFF;
	 while(1);
}


