//--------------------------------------------------------------
// File     : main.c
// Datum    : 11.01.2013
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.4
// GCC      : 4.7 2012q4
// Module   : CMSIS_BOOT, M4_CMSIS_CORE
// Funktion : Demo der FreeRTOS-Library
// Hinweis  : Diese zwei Files muessen auf 8MHz stehen
//              "cmsis_boot/stm32f4xx.h"
//              "cmsis_boot/system_stm32f4xx.c"
//--------------------------------------------------------------


#include "stm32f4xx.h"


/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4xx_rcc.h"

#include "stm32f4xx_gpio.h"

#define DELAY 125     /* msec */
#define queueSIZE	6

#define DATA_PORT GPIOC->ODR
	#define CMD_PORT GPIOA->ODR

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
	void initDiscovery(void);
/* Private functions ---------------------------------------------------------*/

/* Task functions declarations */
static void vLEDTask( void *pvParameters );
static void vLCDTask( void *pvParameters );



/* semaphores, queues declarations */
//xSemaphoreHandle xSemaphoreSW  = NULL;
xQueueHandle xQueue;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* cau hinh PC9 and PC8 la dau ra, khong dung tro keo */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6|GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_7 | GPIO_Pin_5;// dung PC9,PC8

  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// che do uotput
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// reset pin khi uotput
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// clock 50Mhz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// khong dung tro keo
  GPIO_Init(GPIOB, &GPIO_InitStructure);// khoi tao PC

    }


	  //Include main header for MCU

	void delay1(int ticks)
	{
	  while(ticks--);
	}

	void lcd_init()
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

	  GPIOC->MODER=GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;
	  GPIOA->MODER=GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;

	  GPIOC->OSPEEDR = GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR9_1;
	  GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR9_1;
	}
	int main(void)
	{

	/* create a pipe for MEMS->TIM4 data exchange */
		xQueue=xQueueCreate(1,queueSIZE*sizeof(uint8_t));

		/* create semaphores... */
		//vSemaphoreCreateBinary( xSemaphoreSW );

		/* ...and clean them up */
		//if(xSemaphoreTake(xSemaphoreSW, ( portTickType ) 0) == pdTRUE);

		/* initialize hardware... */
	//	prvSetupHardware();
		 gpio_setup();
		 initDiscovery();

		  lcd_init();

		/* Start the tasks defined within this file/specific to this demo. */
		//xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED3", configMINIMAL_STACK_SIZE, (void *)LEDS[0],tskIDLE_PRIORITY, &xLED_Tasks[0] );
		xTaskCreate( vLEDTask, ( signed portCHAR * ) "LEDDON", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);
		xTaskCreate( vLCDTask, ( signed portCHAR * ) "LCD", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);


		/* Start the scheduler. */
		vTaskStartScheduler();

		/* Will only get here if there was not enough heap space to create the idle task. */
		return 0;
	}
	static void vLEDTask( void *pvParameters ){
		  int x1;
		  int x2;
		  int x3;
		  int x4;
		  int x5;
		  int x6;

		while(1){

			//for(x=0x10;x<=0x200;x=x<<1){
			x1=0x10;
			GPIOB->ODR=x1;
			DelayMS(500);

	x2=0x40;
	GPIOB->ODR=x2;
	DelayMS(500);

	x3=0x100;
	GPIOB->ODR=x3;
	DelayMS(500);

	x4=0x200;
	GPIOB->ODR=x4;
	DelayMS(500);
	x5=0x80;

	GPIOB->ODR=x5;
	DelayMS(500);

	x6=0x20;
	GPIOB->ODR=x6;
	DelayMS(500);
			//}


		}
	}
	static void vLCDTask( void *pvParameters ){
		int i;
		while(1)
		  {
			 //lcd_init();
			 // DelayMS(3);
		//  LCD_DATA(0x0C,0);
			  LCD_LINE(1);

			 LCD_STR((const char*)"NGUYEN MINH CHI            ");

		//	 LCD_DATA(0x0C,0);
			///LCD_DATA(0x0E,0);


			  LCD_DATA(0x0C,0);

		//	  LCD_DATA(0x0E,0);
			  LCD_LINE(2);

			  LCD_STR((const char*)"CNVT K9B");
			 // DelayMS(1);
			  //  LCD_DATA(0x0C,0);
			for( i=0;i<=8;i++){


			  LCD_DATA(0x18,0);
			 DelayMS(250);
		  }
	}
			   //Xoa man hinh

		  }

