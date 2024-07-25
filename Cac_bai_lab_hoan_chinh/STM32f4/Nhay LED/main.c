#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

//void IOToggle(void);
GPIO_InitTypeDef  GPIO_InitStructure;
//void Delay(__IO uint32_t nCount);
int i,j;
#define LED1_PORT GPIOA
#define LED2_PORT GPIOA
#define LED3_PORT GPIOA
#define LED4_PORT GPIOA
void delay(void)
    {
        int time;
        for(time=0;time<4000000;time++);// tre khoang 4000000 lenh
    }
int main(void)
{

	SystemInit();

	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2| GPIO_Pin_4| GPIO_Pin_6|GPIO_Pin_1 | GPIO_Pin_3| GPIO_Pin_5| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_All ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	while(1)
    {
		            GPIO_WriteBit(LED1_PORT,GPIO_Pin_0,Bit_SET);delay();
		    	    GPIO_WriteBit(LED1_PORT,GPIO_Pin_0,Bit_RESET);delay();
		    	    GPIO_WriteBit(LED2_PORT,GPIO_Pin_2,Bit_SET);delay();
		    	    GPIO_WriteBit(LED2_PORT,GPIO_Pin_2,Bit_RESET);delay();
		    	    GPIO_WriteBit(LED3_PORT,GPIO_Pin_4,Bit_SET);delay();
		    	    GPIO_WriteBit(LED3_PORT,GPIO_Pin_4,Bit_RESET);delay();
		    	    GPIO_WriteBit(LED4_PORT,GPIO_Pin_6,Bit_SET);delay();
		    	    GPIO_WriteBit(LED4_PORT,GPIO_Pin_6,Bit_RESET);delay();
		    	    GPIO_WriteBit(LED1_PORT,GPIO_Pin_1,Bit_SET);delay();
		    	    GPIO_WriteBit(LED1_PORT,GPIO_Pin_1,Bit_RESET);delay();
		    	   GPIO_WriteBit(LED2_PORT,GPIO_Pin_3,Bit_SET);delay();
		    	    GPIO_WriteBit(LED2_PORT,GPIO_Pin_3,Bit_RESET);delay();
		    	    GPIO_WriteBit(LED3_PORT,GPIO_Pin_5,Bit_SET);delay();
		    	    GPIO_WriteBit(LED3_PORT,GPIO_Pin_5,Bit_RESET);delay();
		    	    GPIO_WriteBit(LED4_PORT,GPIO_Pin_7,Bit_SET);delay();
		    	    GPIO_WriteBit(LED4_PORT,GPIO_Pin_7,Bit_RESET);delay();

		    	      }

    	  }



