#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

//void IOToggle(void);
GPIO_InitTypeDef  GPIO_InitStructure;
void delay(void)
    {
        int time;
        for(time=0;time<4000000;time++);// tre khoang 4000000 lenh
    }

int main(void)
{

	SystemInit();

	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8| GPIO_Pin_7| GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_All ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	while(1)
	{
	    GPIOD->ODR=0x0000;
	    delay();
	    GPIOD->ODR=0x0040;
	    delay();
	    GPIOD->ODR=0x0080;
	       delay();
	    GPIOD->ODR=0x00C0;
	    delay();
	    GPIOD->ODR=0x0100;
	    delay();
	    GPIOD->ODR=0x0140;99
	    delay();
	    GPIOD->ODR=0x0180;
	    delay();
	    GPIOD->ODR=0x01C0;
	    delay();
	    GPIOD->ODR=0x0200;
	    delay();
	    GPIOD->ODR=0x0240;
	    delay();


	    }}
