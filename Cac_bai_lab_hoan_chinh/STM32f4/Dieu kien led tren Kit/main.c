#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

//void IOToggle(void);
GPIO_InitTypeDef  GPIO_InitStructure;
//void Delay(__IO uint32_t nCount);
int i,j;
#define LED1_PORT GPIOD
#define LED2_PORT GPIOD
#define LED3_PORT GPIOD
#define LED4_PORT GPIOD
int main(void)
{

	SystemInit();

	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_All ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	while(1)
    {

		GPIO_WriteBit(LED1_PORT,GPIO_Pin_12,Bit_SET);
		    	    for (i=0;i<100000;i++);
		    	    GPIO_WriteBit(LED1_PORT,GPIO_Pin_12,Bit_RESET);
		    	    for (i=0;i<100000;i++);
		    	    GPIO_WriteBit(LED2_PORT,GPIO_Pin_13,Bit_SET);
		    	      	    for (i=0;i<100000;i++);
		    	      	    GPIO_WriteBit(LED2_PORT,GPIO_Pin_13,Bit_RESET);
		    	      	    for (i=0;i<100000;i++);

		    	      	    GPIO_WriteBit(LED3_PORT,GPIO_Pin_14,Bit_SET);
		    	      	    for (i=0;i<100000;i++);
		    	      	    GPIO_WriteBit(LED3_PORT,GPIO_Pin_14,Bit_RESET);
		    	      	    for (i=0;i<100000;i++);

		    	      	    GPIO_WriteBit(LED4_PORT,GPIO_Pin_15,Bit_SET);
		    	      	    for (i=0;i<100000;i++);
		    	      	    GPIO_WriteBit(LED4_PORT,GPIO_Pin_15,Bit_RESET);
		    	      	    for (i=0;i<100000;i++);


		    	      }

    	  }


