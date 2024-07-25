#include <stm32f4xx.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>

#define LED1_PORT GPIOD
#define LED2_PORT GPIOD
#define LED3_PORT GPIOD
#define LED4_PORT GPIOD
 uint8_t SPI_data_get_slave;
 NVIC_InitTypeDef  NVIC_InitStructure;
 GPIO_InitTypeDef  GPIO_InitStructure;
SPI_InitTypeDef   SPI_InitStructure;
// this function initializes the SPI1 peripheral
  /*
   * cach noi day
   * SPI1          SPI2
   * PA4_SPI1_NSS	PB12-SPI2_NSS
   * PA5-SPI1_SCK	PB13-SPI2_SCK
   * PA6-SPI1_MISO	PB14-SPI2_MISO
   * PA7-SPI1_MOSI	PB15-SPI2_MOSI
   */
void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

void initLED(){
	  SystemInit();

	  	/* GPIOD Periph clock enable */
	  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	}
void init_SPI2(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_Init(GPIOB, &GPIO_InitStructure);

		    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_SPI2);   // only connect to
		    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);   // only connect to
		    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);   // only connect to
		    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);   // only connect to
	// enable clock for used IO pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);



	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_SSOutputCmd(SPI2,ENABLE);
	SPI_Init(SPI2, &SPI_InitStruct);


	  //cau hinh ngat cho slave khi nhan du lieu
	    NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

	    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	    //ket thuc cau hinh ngat cho spi2
	    	    /* Enable SPI_SLAVE */
	    SPI_Cmd(SPI2, ENABLE);
}
//-------------Bat dau LCD----------------------------------------//
//PC4-->PC7-KET NOI VOI CHAN DB4-->DB7 tren lcd
//PA1-en tren lcd; PA2-chan rs tren lcd

////bat dau khai bao cho lcd
#define DATA_PORT GPIOC->ODR
#define CMD_PORT GPIOA->ODR

	#define DATA 4
	#define DATA_CLR 0xFFFFFF0F

	#define LCD_E  0x2     //PA1
	#define LCD_RS 0x4     //PA2

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
			  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
			  RCC->CR |= RCC_CR_HSEON;
			  while((RCC->CR & RCC_CR_HSERDY)==0);
			  GPIOC->MODER=GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0|GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0;
			  GPIOA->MODER=GPIO_MODER_MODER1_0|GPIO_MODER_MODER2_0;

			  GPIOC->OSPEEDR = GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1;
			  GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR1_1 | GPIO_OSPEEDER_OSPEEDR2_1;
			}
//ket thuc khai bao lcd

int main(void){
	init_SPI2();
	initLED();
	initDiscovery();

	LCD_Init();

	while(1){
		GPIOD->ODR=0x0;
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);//thiet lap PB12=0 de chon slave bat dau giao tiep

		SPI_I2S_SendData(SPI2, 'A');
		 LCD_LINE(1);

					 LCD_STR((const char*)"    Seding: A    ");

	while( !(SPI2->SR & SPI_I2S_FLAG_TXE) );// Doi truyen thanh cong
	 GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_SET);
	 GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET);
				GPIO_SetBits(GPIOB, GPIO_Pin_12); // Thiet lap PB12=1 de ket thuc qua trinh giao tiep
		Delay(2000000);
		}
			}

void SPI2_IRQHandler(void)
 {
    if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
    {
      SPI_data_get_slave =  SPI_I2S_ReceiveData(SPI2);
      //LCD_Init();

          if ( SPI_data_get_slave=='B'){

      GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);
      GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
      LCD_LINE(2);
            LCD_STR((const char*)"   Receiving: B  ");
            Delay(2000000);
            GPIO_ResetBits(GPIOD, GPIO_Pin_12);
            GPIO_ResetBits(GPIOD, GPIO_Pin_15);
          }
      SPI_I2S_ClearFlag(SPI2, SPI_I2S_IT_RXNE);
    }
  }
