#include <stm32f4xx.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>

#define LED1_PORT GPIOD
#define LED2_PORT GPIOD
#define LED3_PORT GPIOD
#define LED4_PORT GPIOD
 uint8_t SPI_data_get_master;
 NVIC_InitTypeDef  NVIC_InitStructure;
 GPIO_InitTypeDef  GPIO_InitStructure;
SPI_InitTypeDef   SPI_InitStructure;
// this function initializes the SPI1 peripheral
  /*
   * cach noi day
   * SPI1          SPI2
   * PA4_SPI1_NSS	PB12-SPI2_NSS
   * PA5-SPI1_SCK	PB12-SPI2_SCK
   * PA6-SPI1_MISO	PB14-SPI2_MISO
   * PA7-SPI1_MOSI	PB15-SPI2_MOSI
   */
void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

void initLED(){
	  SystemInit();

	  	/* GPIOD Periph clock enable */
	  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	  	///
		/* GPIOD Periph clock enable */
		  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		  	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
		  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
		  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	}
void init_SPI2(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	/* SPI_SLAVE configuration ------------------------------------------------*/
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

	    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_SPI2);   // only connect to
	    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);   // only connect to
	    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);   // only connect to
	    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);   // only connect to

	    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	    SPI_InitStructure.SPI_CRCPolynomial = 7;
	    SPI_Init(SPI2, &SPI_InitStructure);

	  //cau hinh ngat cho slave khi nhan du lieu
	    NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

	    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	    //ket thuc cau hinh ngat cho spi2

	    /* Enable SPI_SLAVE */
	    SPI_Cmd(SPI2, ENABLE);

	//***********************************
	}

int main(void){
		init_SPI2();
	initLED();
	while(1){

		    		GPIOD->ODR=0x0000;
		    		GPIOC->ODR=0x00F0;
		    		Delay(1000000);
		    SPI_I2S_SendData(SPI2,'B');
		    while( !(SPI2->SR & SPI_I2S_FLAG_TXE) );// wait until transmit complete
		    GPIOC->ODR=0x0000;
		    		Delay(20000000);
		    	//}

			}

	}

void SPI2_IRQHandler(void)
 {
    if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
    {
      SPI_data_get_master = SPI_I2S_ReceiveData(SPI2);
     // for(int i=0;i<16;i++){
     if ( SPI_data_get_master=='A')
    	  GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);
     	 GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_SET);
     	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
     	GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET);
         SPI_I2S_ClearFlag(SPI2, SPI_I2S_IT_RXNE);
        }


  }

