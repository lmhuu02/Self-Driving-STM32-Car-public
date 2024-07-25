#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
int lan;
char dulieu[]="    Welcome Chinese students     ";
void delay()
    {
    int time;
        for(time=0;time<4000000;time++);
    }

void usart_setup(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);

  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  USART_Cmd(USART3,ENABLE);
}

void truyen(unsigned char uData)
    {
  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)==RESET);
  USART_SendData(USART3,uData);
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
        delay();
    }
}
