#include "stm32f0xx.h"
#include"stm32f0xx_gpio.h"
#include"stm32f0xx_i2c.h"
#include"stm32f0xx_misc.h"
#include"stm32f0xx_rcc.h"

/*	MASTER                    SLAVE
  PB6 --- I2C1_SCL		PB6 --- I2C2_SCL
  PB7 --- I2C1_SDA		PB7--- I2C2_SDA
*/

#define slave_addr    0X30

GPIO_InitTypeDef  GPIO_InitStructure;
I2C_InitTypeDef   I2C_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;

void I2C_Configuration(void);
uint8_t temp=0;
void Delay( uint32_t nCount)
{
  while(nCount--)
  {
  }
}
void led()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		//GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_All ;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
}

int main(void)
{

  led();
  I2C_10BitAddressingModeCmd(I2C1,DISABLE);
   I2C_Configuration();

   I2C_AcknowledgeConfig(I2C1,DISABLE);

  // if (I2C_GetAddressMatched(I2C1)==0x30)

  	//    I2C_AcknowledgeConfig(I2C1,ENABLE);

  while (1)
  {

 }
}


/*CĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚ÂºĂƒâ€Ă‚Â¥u hĂƒâ€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â¬nh cĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»ĂƒÂ¢Ă¢â€Â¬Ă‚Â¢ng I2C1*/
void I2C_Configuration(void)
{
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1ENR_I2C1EN, ENABLE);


 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//khai bĂƒâ€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â¡o cĂƒâ€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â³ sĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»Ăƒâ€Ă‚Â± chuyĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»Ăƒâ€ Ă¢â‚¬â„¢n ĂƒÆ’Ă¢â‚¬Å¾ĂƒÂ¢Ă¢â€Â¬Ă‹Å“ĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»ĂƒÂ¢Ă¢â€Â¬Ă‚Â¢i chĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»Ăƒâ€Ă‚Â©c nĂƒÆ’Ă¢â‚¬Å¾Ăƒâ€ Ă¢â‚¬â„¢ng cĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»Ăƒâ€Ă‚Â§a PB6 vĂƒâ€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â  PB7 ĂƒÆ’Ă¢â‚¬Å¾ĂƒÂ¢Ă¢â€Â¬Ă‹Å“ĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»Ăƒâ€ Ă¢â‚¬â„¢ trĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»Ăƒâ€¦Ă‚Â¸ thĂƒâ€Ă¢â‚¬Å¡Ăƒâ€Ă‚Â nh cĂƒÆ’Ă‚Â¡Ăƒâ€Ă‚Â»ĂƒÂ¢Ă¢â€Â¬Ă‚Â¢ng I2C
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // enable pull up resistors
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_1);   // only connect to
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_1);  // only connect to

  /* I2C De-initialize */
  I2C_DeInit(I2C1);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter=I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter=0x00;
  I2C_InitStructure.I2C_OwnAddress1 = slave_addr;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_Timing = 0x10800000;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStructure);


  /* Cau hinh ngat  */
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


   I2C_ITConfig(I2C1,I2C_IT_RXI|I2C_IT_ADDRI,ENABLE);
    //ket thuc cau hinh ngat cho spi2
    	    /* Enable SPI_SLAVE */
    I2C_Cmd(I2C1, ENABLE);

}
void I2C1_IRQHandler(void)
 {
	 if (I2C_GetITStatus(I2C1,I2C_IT_ADDR))
	    {
		// GPIO_SetBits(GPIOC, GPIO_Pin_8);
		 //GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		 I2C_AcknowledgeConfig(I2C1,ENABLE);
		    I2C_ClearITPendingBit(I2C1,I2C_IT_ADDR);
		    I2C_ClearFlag(I2C1,I2C_FLAG_ADDR);
		    I2C_AcknowledgeConfig(I2C1,DISABLE);
	    }

    if (I2C_GetITStatus(I2C1,I2C_IT_RXNE))
    {

    	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    			 GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    			 Delay(1000000);

    	 temp= I2C_ReceiveData(I2C1);
    	 I2C_AcknowledgeConfig(I2C1,ENABLE);

    	 GPIO_SetBits(GPIOC, GPIO_Pin_8);
    	  GPIO_SetBits(GPIOC, GPIO_Pin_9);
    	  Delay(1000000);
    	  I2C_AcknowledgeConfig(I2C1,DISABLE);
     	if(temp=='B'){
     		I2C_AcknowledgeConfig(I2C1,ENABLE);
    		GPIO_SetBits(GPIOC, GPIO_Pin_7);
    				 Delay(6000000);
    				 GPIO_ResetBits(GPIOC, GPIO_Pin_7);
    				 temp=0;
    				 I2C_AcknowledgeConfig(I2C1,DISABLE);
    		    	}
     	I2C_ClearFlag(I2C1,I2C_FLAG_RXNE);
     	I2C_ClearFlag(I2C1,I2C_IT_RXNE);
     	 I2C_ClearITPendingBit(I2C1,I2C_IT_RXNE);
      I2C_ClearITPendingBit(I2C1,I2C_IT_RXI);
    }
  }








