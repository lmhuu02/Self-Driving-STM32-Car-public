#include <stm32f0xx_gpio.h>
#include <stm32f0xx_usart.h>
#include <stm32f0xx_rcc.h>
int lan;
char dulieu[]="     I wish you good learning     ";
void delay()
    {
    int time;
        for(time=0;time<4000000;time++);
    }
void usart_setup(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9|GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_3 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =     USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2,ENABLE);
}
void truyen(unsigned char uData)
    {
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)==RESET);
  USART_SendData(USART2,uData);
    }
void ghi()
    {
        int i;
    for(i=0;i<29;i++)
        {
            truyen(dulieu[i]);
        }
    }
int main(void)
{
    usart_setup();
while(1)
    {
        ghi();
        truyen(0x0d);
        truyen(0x0a);
        if(lan==0)         GPIO_SetBits(GPIOC,GPIO_Pin_9|GPIO_Pin_8);
        else if(lan==1)    GPIO_ResetBits(GPIOC,GPIO_Pin_9|GPIO_Pin_8);
        lan++;
        if(lan==2) lan=0;
        delay();
    }
}
