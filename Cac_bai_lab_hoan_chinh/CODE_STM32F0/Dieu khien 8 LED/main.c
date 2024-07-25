#include <stm32f0xx.h>                    // thu vien cho dong stm32f0xx
#include <stm32f0xx_rcc.h>            // thu vien cau hinh xung clock
#include <stm32f0xx_gpio.h>            // thu vien cau hinh gpio
// h� m delay
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
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* cau hinh PC6,PC7,PC8,PC9 la dau ra, khong dung tro keo */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6 ;// Dùng chân PC6-PC9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// che do uotput
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// reset pin khi uotput
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// clock 50Mhz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// khong dung tro keo
  GPIO_Init(GPIOC, &GPIO_InitStructure);// khoi tao cong PC
    }
int main(void)
{
    gpio_setup();
while(1)
    {
	GPIOC->ODR=0x0000;
    GPIOC->ODR=0x2000;
    delay();
    GPIOC->ODR=0x3000;
    delay();
    GPIOC->ODR=0x3800;
       delay();
    GPIOC->ODR=0x3C00;
    delay();
    GPIOC->ODR=0x3E00;
    delay();
    GPIOC->ODR=0x3F00;
    delay();
    GPIOC->ODR=0x3F80;
    delay();
    GPIOC->ODR=0x3FC0;
    delay();
    }
}





