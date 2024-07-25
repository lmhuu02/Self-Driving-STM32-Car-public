#include <stm32f4xx.h>                    // thu vien cho dong stm32f0xx
#include <stm32f4xx_rcc.h>            // thu vien cau hinh xung clock
#include <stm32f4xx_gpio.h>            // thu vien cau hinh gpio
#include <stm32f4xx.h>

void delay(void)
    {
        int time;
        for(time=0;time<400000;time++);// tre khoang 4000000 lenh
    }
void gpio_setup(void)
    {
    // khai bao GPIO_InitStructure = GPIO_InitTypeDef
    // 2 cach viet la nhu nhau
    GPIO_InitTypeDef        GPIO_InitStructure;
        /* cap xung cho PC */
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

  /* cau hinh PC9 and PC8 la dau ra, khong dung tro keo */
    GPIO_InitTypeDef gpioInit;
       //dau tien cho phep xung clock cap toi GPIOD
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
       //khoi tao GPIOD
       gpioInit.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
       gpioInit.GPIO_Mode=GPIO_Mode_OUT;
       gpioInit.GPIO_Speed=GPIO_Speed_100MHz;
       gpioInit.GPIO_OType=GPIO_OType_PP;
       gpioInit.GPIO_PuPd=GPIO_PuPd_NOPULL;
       GPIO_Init(GPIOD, &gpioInit);
    }
int main(void)
{
    gpio_setup();
while(1)
{
    GPIOD->ODR=0x8000; //pc8=1 pc9=0
    delay();
    GPIOD->ODR=0xC000;
    delay();
    GPIOD->ODR=0xE000; //pc8=1 pc9=0
    delay();
    GPIOD->ODR=0xF000;
    delay();
    GPIOD->ODR=0x0000;// sang tu trai qua phai
    delay();
    GPIOD->ODR=0xFFFF; //pc8=1 pc9=0
      delay();
      GPIOD->ODR=0x0000;
      delay();
      GPIOD->ODR=0xFFFF; //pc8=1 pc9=0
      delay();
      GPIOD->ODR=0x0000;
      delay();
      GPIOD->ODR=0xFFFF;// sang tu trai qua phai
      GPIOD->ODR=0x0000;
           delay();
    }
}


