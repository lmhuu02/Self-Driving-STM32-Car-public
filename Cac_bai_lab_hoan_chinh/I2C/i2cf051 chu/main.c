#include "stm32f0xx.h"
#include"stm32f0xx_gpio.h"
#include"stm32f0xx_i2c.h"
#include"stm32f0xx_misc.h"
#include"stm32f0xx_rcc.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_exti.h"
//Pc9-dieu khien dong co quay tren slave; Pb11 de noi voi nut bam tren kit master
//pb6-pb6;pb7-pb7: ket noi i2c
//PA6,PA7,PA8,PA9-KET NOI VOI CHAN DB4-->DB7 tren lcd
//pc6-en tren lcd; pc7-chan rs tren lcd
#define slave_addr    0x30

GPIO_InitTypeDef  GPIO_InitStructure;
I2C_InitTypeDef   I2C_InitStructure;


//bat dau khai bao cho lcd
#define DATA_PORT GPIOA->ODR
#define CMD_PORT GPIOC->ODR

	#define DATA 6
	#define DATA_CLR 0xFFFFFC3F

	#define LCD_E  0x40     //Pc6
	#define LCD_RS 0x80     //Pc7

	#define CMD 0
	#define TXT 1

	#define        LINE1    0x80        // Start address of first line
	#define        LINE2    0xC0        // Start address of second line

GPIO_InitTypeDef  GPIO_InitStructure;
I2C_InitTypeDef   I2C_InitStructure;

	void WaitLCDBusy(void);
	void LCD_Init(void);
	void LCD_DATA(unsigned char data,unsigned char type);
	void LCD_NYB(unsigned char nyb,unsigned char type);
	void LCD_STR(const char *text);
	void LCD_LINE(char line);
	void DelayMS(unsigned int ms);
	void initDiscovery(void);
		void delay(void)
		    {
		        int time;
		        for(time=0;time<4000000;time++);// tre khoang 4000000 lenh
		    }

			void delay1(int ticks)
			{
			  while(ticks--);
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
			  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
			  RCC->CR |= RCC_CR_HSEON;
			  while((RCC->CR & RCC_CR_HSERDY)==0);
			  GPIOA->MODER=GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;
			  GPIOC->MODER=GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0;
			  GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR9_1;
			  GPIOC->OSPEEDR = GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1;
			}
//ket thuc khai bao lcd

void I2C_Configuration(void);
void Delay( uint32_t nCount)
{
while(nCount--)
  {
  }
}
void led()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

		/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		//GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_All ;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
}
//cau hinh ngat cho cong pb11
void EXTIB11_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Connect EXTI0 Line to PA0 pin */

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void send (char kt)
{
			I2C_ClearFlag(I2C1,I2C_FLAG_BUSY);
			I2C_ClearFlag(I2C1,I2C_FLAG_STOPF);//
			I2C_ClearFlag(I2C1,I2C_FLAG_TXE);

		  	 while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
		  		I2C_SlaveAddressConfig(I2C1,slave_addr);
		  	  /* Send Start*/
		  	  I2C_GenerateSTART(I2C1, ENABLE);
		  	  I2C_NumberOfBytesConfig(I2C1,1);
		  	I2C_SendData(I2C1,kt);
	  	GPIOC->ODR=0x0300;
	  	Delay(1000000);
	  	 GPIOC->ODR=0x000;
	  	Delay(1000000);
	  	//I2C1->CR2=0;
I2C_GenerateSTOP(I2C1, ENABLE);
I2C_Cmd(I2C1, DISABLE);
}

int main(void)
{

	initDiscovery();

	LCD_Init();
  led();
  EXTIB11_Config();
  I2C_Configuration();
I2C_10BitAddressingModeCmd(I2C1, DISABLE);
LCD_LINE(1);
	  		 LCD_STR((const char*)" Press Button ");

 while (1)
	 {


	   }

}

void I2C_Configuration(void)
{
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);//CĂ„â€Ă†â€™Ăƒâ€Ă‚Â¡Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚ÂºĂ„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â¥p xung cho cĂ„â€Ă†â€™Ăƒâ€Ă‚Â¡Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â»Ă„â€Ă‚Â¢ĂƒÂ¢Ă¢â‚¬Å¡Ă‚Â¬Ăƒâ€Ă‚Â¢ng B

  RCC_APB1PeriphClockCmd(RCC_APB1ENR_I2C1EN, ENABLE);//CĂ„â€Ă†â€™Ăƒâ€Ă‚Â¡Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚ÂºĂ„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â¥p xung clock cho I2C1
  /* Configure the I2C clock source. The clock is derived from the HSI */
 //RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // sĂ„â€Ă†â€™Ăƒâ€Ă‚Â¡Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â»Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â­ dĂ„â€Ă†â€™Ăƒâ€Ă‚Â¡Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â»Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â¥ng Ă„â€Ă†â€™ĂƒÂ¢Ă¢â€Â¬Ă‚Å¾Ă„â€Ă‚Â¢ĂƒÂ¢Ă¢â‚¬Å¡Ă‚Â¬Ăƒâ€¹Ă…â€œiĂ„â€Ă†â€™Ăƒâ€Ă‚Â¡Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â»Ă„â€Ă‚Â¢ĂƒÂ¢Ă¢â‚¬Å¡Ă‚Â¬Ăƒâ€Ă‚Â¡n trĂ„â€Ă†â€™Ăƒâ€Ă‚Â¡Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â»Ă„â€Ă¢â‚¬Â¦Ăƒâ€Ă‚Â¸ kĂ„â€Ă†â€™Ăƒâ€ Ă¢â‚¬â„¢Ă„â€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â©o
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_1);   // only connect to
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_1);   // only connect to
  // only connect to

/************************************* Master ******************************/
  /* I2C De-initialize */
  I2C_DeInit(I2C1);

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter=I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter=0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x01;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
      I2C_InitStructure.I2C_Timing = 0x10805E89;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStructure);
   /* I2C ENABLE */
  I2C_Cmd(I2C1, ENABLE);
}
void EXTI4_15_IRQHandler(void)
{
	char st[50]="Hello Word"; int i;

  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
	  I2C_Cmd(I2C1, ENABLE);
	  //Delay(10000);
	  LCD_LINE(1);

	  	LCD_STR((const char*)" Master sending ");
Delay(1000000);
for (i=0;i<strlen(st);i++){
	I2C_Cmd(I2C1, ENABLE);

	GPIO_SetBits(GPIOC,GPIO_PinSource8);
	GPIO_SetBits(GPIOC,GPIO_PinSource9);
	send(st[i]);
	Delay(1000);
GPIO_ResetBits(GPIOC,GPIO_PinSource8);
GPIO_ResetBits(GPIOC,GPIO_PinSource9);
Delay(1000);
}
GPIO_SetBits(GPIOC,GPIO_PinSource8);
	GPIO_SetBits(GPIOC,GPIO_PinSource9);
I2C_Cmd(I2C1, ENABLE);

send ('B');
GPIO_SetBits(GPIOC,GPIO_PinSource8);
GPIO_SetBits(GPIOC,GPIO_PinSource9);
//Delay(1000000);
	//Delay(100);
	LCD_LINE(1);

	LCD_STR((const char*)" Master complete ");
	Delay(1000000);
}
    /* Clear the EXTI line 0 pending bit */
   EXTI_ClearITPendingBit(EXTI_Line11);
  }






