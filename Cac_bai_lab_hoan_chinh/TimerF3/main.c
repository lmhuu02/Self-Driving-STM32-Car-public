#include"stm32f30x.h"
#include"stm32f30x_gpio.h"
#include"stm32f30x_rcc.h"
#include"stm32f30x_misc.h"
#include"stm32f30x_tim.h"
#include"stdint.h"
#include"inttypes.h"

#define DATA_PORT GPIOB->ODR
#define CMD_PORT GPIOA->ODR

	#define DATA 6
	#define DATA_CLR 0xFFFFFC3F

	#define LCD_E  0x100     //PA8
	#define LCD_RS 0x200     //PA9

	#define CMD 0
	#define TXT 1

	#define        LINE1    0x80        // Start address of first line
	#define        LINE2    0xC0        // Start address of second line
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	void WaitLCDBusy(void);
	void LCD_Init(void);
	void LCD_DATA(unsigned char data,unsigned char type);
	void LCD_NYB(unsigned char nyb,unsigned char type);
	void LCD_STR(const char *text);
	void LCD_LINE(char line);
	void DelayMS(unsigned int ms);

	static uint32_t nowvalue=0;

	void GPIO_Configuration(void);
	void TIMbase_Configuration(void);
	void TIM2_Configuration(void);

int main(void)
{
	uint8_t ch=0;
GPIO_Configuration();
LCD_Init();
TIMbase_Configuration();
TIM2_Configuration();
GPIOA->ODR=0xF;
//GPIOA->ODR=GPIOA->ODR&0xCF;
while (1)
   {

     while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1)
    	 GPIOA->ODR=GPIOA->ODR & 0xFFF0;
     while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==1)
    	GPIOA->ODR=GPIOA->ODR|0xF;
    // DelayMS(100000);

   }
 }
void LCD_Display(){
	LCD_DATA(0x01,0);
	LCD_DATA(0x06,0);
	LCD_DATA(0x06,0);
	DelayMS(5);
	 LCD_LINE(1);
	 char str[20]="", str1[20]="";int i,dem=0;
	while (nowvalue!=0){
		str[dem]=nowvalue%10+0x30;
		nowvalue=nowvalue/10;
		dem=dem+1;
	}

	for (i=dem-1;i>=0;i--)
		str1[dem-1-i]=str[i];
	LCD_STR((const char*)"TIM 2 - TC:");
	DelayMS(5);
	LCD_LINE(2);
	LCD_STR((const char*)str1);
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
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

  // Chan PB0, PB1 cho LCD, chan PB2, PB3, PB4 cho LED
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11|GPIO_Pin_3 | GPIO_Pin_15|GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOE, &GPIO_InitStructure);

   //Chan PA0-PA3 cho LED; PA4,PA5 cho nut bam; PA8, PA9 cho LCD,
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
     GPIO_Init(GPIOB, &GPIO_InitStructure);

 }

void TIMbase_Configuration(void)
{
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

/* Time base configuration */
TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock/2)/10000)-1;     // frequency = 1000000
TIM_TimeBaseStructure.TIM_Period = 65535 - 1;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
TIM_Cmd(TIM4, ENABLE);

NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}
void TIM2_Configuration(void)
{
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

 /* Time base configuration */
TIM_TimeBaseStructure.TIM_Prescaler = 0;
TIM_TimeBaseStructure.TIM_Period = 0xFFFFFF;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
TIM_Cmd(TIM2, ENABLE);
}

 void TIM4_IRQHandler(void)
 {

  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
   {
	   nowvalue = TIM_GetCounter(TIM2);
//Hien thi gia tri Timer 2
	   LCD_Display();
	   //Bat LED cong B
	  // DelayMS(100000);

	   GPIOE->ODR=0xFF00;

	   Delay(8000000);
	   GPIOE->ODR=0x0000;
	 //  Delay(10000000);
   TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
   }
 }
