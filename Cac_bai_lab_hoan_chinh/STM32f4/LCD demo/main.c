#include "stm32f4xx.h"    //Include main header for MCU

#define DATA_PORT GPIOB->ODR
#define CMD_PORT GPIOC->ODR

#define DATA 6
#define DATA_CLR 0xFFFFFC3F

#define LCD_E  0x10     //PA8
#define LCD_RS 0x20     //PA9

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
void delay(int ticks)
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
    delay(100);                //SMALL DELAY
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
        delay(600);
}
//--------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------//
//   MAIN   -   MAIN   -   MAIN   -   MAIN   -   MAIN   -   MAIN   -   MAIN       //
//--------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------//
void initDiscovery(void)
{
  RCC->CFGR = RCC_CFGR_SW_HSE;
  RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOCEN;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC->CR |= RCC_CR_HSEON;

  while((RCC->CR & RCC_CR_HSERDY)==0);

  GPIOB->MODER=GPIO_MODER_MODER11_0|GPIO_MODER_MODER12_0|GPIO_MODER_MODER13_0|GPIO_MODER_MODER14_0;
  GPIOC->MODER=GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0;

  GPIOB->OSPEEDR = GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR12_1 | GPIO_OSPEEDER_OSPEEDR13_1 | GPIO_OSPEEDER_OSPEEDR14_1;
  GPIOC->OSPEEDR = GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR5_1;
}

void main()
{
  initDiscovery();

  lcd_init();
  DelayMS(3);
  LCD_LINE(1);
  LCD_STR((const char*)"We are happy");
  DelayMS(100);
  LCD_LINE(2);
  LCD_STR((const char*)"to teach you");
  DelayMS(100);
  while(1)
  {

  }
}
