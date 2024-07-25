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
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2| GPIO_Pin_1 | GPIO_Pin_0;// Dùng chân PC6-PC9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// che do uotput
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// reset pin khi uotput
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// clock 50Mhz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// khong dung tro keo
  GPIO_Init(GPIOC, &GPIO_InitStructure);// khoi tao cong PA
    }
int main(void)
{
    gpio_setup();
while(1)
    {
	    GPIOC->ODR=0x0000;
	    delay();
	    GPIOC->ODR=0x0001;
	       delay();
	    GPIOC->ODR=0x0002;
	    delay();
	    GPIOC->ODR=0x0003;
	    delay();
	    GPIOC->ODR=0x0004;
	    delay();
	    GPIOC->ODR=0x0005;
	    delay();
	    GPIOC->ODR=0x0006;
	    delay();
	    GPIOC->ODR=0x0007;
	    delay();
	    GPIOC->ODR=0x0008;
	    delay();
	    // �?m l�i
	    GPIOC->ODR=0x0009;
	    delay();
	    GPIOC->ODR=0x0000;
	    delay();
	    GPIOC->ODR=0x0009;
	    delay();
	    GPIOC->ODR=0x0008;
	    delay();
	    GPIOC->ODR=0x0007;
	    delay();
	    GPIOC->ODR=0x0006;
	    delay();
	    GPIOC->ODR=0x0005;
	    delay();
	    GPIOC->ODR=0x0004;
	    delay();
	    GPIOC->ODR=0x0003;
	    delay();
	    GPIOC->ODR=0x0002;
	    delay();
	    GPIOC->ODR=0x0001;
	   	    delay();

    }
}


