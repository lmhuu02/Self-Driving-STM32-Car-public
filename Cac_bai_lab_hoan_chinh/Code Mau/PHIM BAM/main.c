#include <stm32f0xx.h>                    // thu vien cho dong stm32f0xx
#include <stm32f0xx_rcc.h>            // thu vien cau hinh xung clock
#include <stm32f0xx_gpio.h>            // thu vien cau hinh gpio
// hàm delay
int dem;
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
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* cau hinh PC9 and PC8 la dau ra, khong dung tro keo */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0 | GPIO_Pin_0 ;// dung PC9,PC8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// che do uotput
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// reset pin khi uotput
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// clock 50Mhz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// khong dung tro keo
  GPIO_Init(GPIOB, &GPIO_InitStructure);// khoi tao PC

    // cau hinh cho PA0
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* cau hinh PA0 dau vao,tro keo PuPd_DOWN */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;// dung PA0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;// che do uotput
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// reset pin khi uotput
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// clock 50Mhz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;// khong dung tro keo
  GPIO_Init(GPIOA, &GPIO_InitStructure);// khoi tao PA
    }
int main(void)
{
    gpio_setup();
    GPIOB->ODR=0x0000; //PB1=0; PB2=0
while(1)
    {
    if((GPIOA->IDR&0x01)==1) // nếu PA0=1
        {
        while((GPIOA->IDR&0x01)==1);// khi chưa nhả phím, không làm gì cả
            dem++;
            if(dem==2) dem=0;
            if(dem==0) GPIOB->ODR=0x0001;//PB1=1,PB2 =0//
            else if(dem==1) GPIOB->ODR=0x0002;//PB1=0,PB2 =1
        }
    }
}
