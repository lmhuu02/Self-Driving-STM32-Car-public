#include <stm32f0xx_gpio.h>//khai bÃƒÂ¡o thÃ†Â° viÃ¡Â»â€¡n danh cho cÃƒÂ¡c cÃ¡Â»â€¢ng
#include <stm32f0xx_usart.h>// thÃ†Â° viÃ¡Â»â€¡n dÃƒÂ¹ng cho ngoÃ¡ÂºÂ¡i vi usart
#include <stm32f0xx_rcc.h>// thÃ†Â° viÃ¡Â»â€¡n cung cÃ¡ÂºÂ¥p xung Clock
#include<string.h>
 lan=0;
char c;
void delay()
    {
    int time;
        for(time=0;time<4000000;time++);
    }
void usart_setup(void)
{
  USART_InitTypeDef USART_InitStructure; //khai bÃƒÂ¡o kiÃ¡Â»Æ’u cÃ¡ÂºÂ¥u trÃƒÂºc cho usart
  GPIO_InitTypeDef GPIO_InitStructure; // khai bÃƒÂ¡o kiÃ¡Â»Æ’u cÃ¡ÂºÂ¥u trÃƒÂºc cho gpio
//Xung clock Ã¡Â»Å¸ port C bÃ¡ÂºÂ­t, cho phÃƒÂ©p cÃ¡Â»â€¢ng C hoÃ¡ÂºÂ¡t Ã„â€˜Ã¡Â»â„¢ng
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
// ThiÃ¡ÂºÂ¿t lÃ¡ÂºÂ­p cho 2 chÃƒÂ¢n 8 vÃƒÂ  9 Ã¡Â»Å¸ port C
  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_8|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOC, &GPIO_InitStructure);
// Cho phÃƒÂ©p bÃ¡ÂºÂ­t xung Clock Ã¡Â»Å¸ port A, thiÃ¡ÂºÂ¿t lÃ¡ÂºÂ­p 2 chÃƒÂ¢n 9 vÃƒÂ  10 dÃƒÂ nh cho USART1
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_10 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =     USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1,ENABLE);
}
void truyen(unsigned char uData)
// HÃƒÂ m truyÃ¡Â»ï¿½n cho phÃƒÂ©p gÃ¡Â»Â­i 1 kÃƒÂ½ tÃ¡Â»Â± tÃ¡Â»Â« mÃƒÂ¡y tÃƒÂ­nh sang khi cÃ¡Â»ï¿½ trÃ¡ÂºÂ¡ng thÃƒÂ¡i USART_FLAG_TXE cÃ¡Â»Â§a USART1 Ã¡Â»Å¸ chÃ¡ÂºÂ¿ Ã„â€˜Ã¡Â»â„¢ RESET
    {
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
  USART_SendData(USART1,uData);
    }
void ghi(char a[100])
    {
        int i;
    for(i=0;i<strlen(a);i++)
        {
            truyen(a[i]);
        }
    }


char USART_getc(void)
{
  char c;
//KiÃ¡Â»Æ’m tra xem cÃƒÂ³ bÃ¡Â»â€¹ lÃ¡Â»â€”i trÃƒÂ n bÃ¡Â»â„¢ Ã„â€˜Ã¡Â»â€¡m khÃƒÂ´ng
  if((USART1->ISR & USART_ISR_ORE) != 0)
  {
    //NÃ¡ÂºÂ¿u cÃƒÂ³ thÃƒÂ¬ xÃƒÂ³a
    USART1->ICR |= USART_ICR_ORECF;
  }
  // Ã„ï¿½Ã¡Â»Â£i nhÃ¡ÂºÂ­n dÃ¡Â»Â¯ liÃ¡Â»â€¡u Ã¡Â»Å¸ thanh ghi dÃ¡Â»Â¯ liÃ¡Â»â€¡u nhÃ¡ÂºÂ­n
  while((USART1->ISR & USART_ISR_RXNE) == 0){;}
  // Ã„ï¿½Ã¡Â»ï¿½c dÃ¡Â»Â¯ liÃ¡Â»â€¡u tÃ¡Â»Â« bÃ¡Â»â„¢ RDR
  c = (char)USART1->RDR;
  return(c);
}


int main(void)
{
    usart_setup();

while(1)
    {
        truyen(0x0d);
        truyen(0x0a);
        ghi("Hay nhap 1 ki tu:\n A-Bat LED tai PC7\n B-Bat LED tai PC8\Tat LED tai PC7\n 2-Tat LED tai PC8\n F-Tat ca 2 LED");
       c=USART_getc();
       if(c=='a'){
    	   GPIO_SetBits(GPIOC,GPIO_Pin_7);

               ghi("LED 1 sang\n");
               delay(10);}
       if(c=='b')
            {
    	   GPIO_SetBits(GPIOC,GPIO_Pin_8);
          ghi("LED 2 sang\n");
           delay(10);}
       if(c=='c')
        { GPIO_SetBits(GPIOC,GPIO_Pin_8|GPIO_Pin_7);
        ghi("ca 2 LED sang\n");
        delay(10);}

       if(c=='1')
                {
    	   	   GPIO_ResetBits(GPIOC,GPIO_Pin_7);
                ghi("LED 1 tat\n");
                delay(10);}
       if(c=='2')
                 { GPIO_ResetBits(GPIOC,GPIO_Pin_8);
                  ghi("LED 2 tat\n");
                  delay(10);}
        else if(c=='f')
        	{GPIO_ResetBits(GPIOC,GPIO_Pin_8|GPIO_Pin_7);
        ghi("ca 2 LED tat\n");
        delay();
    }
}}




