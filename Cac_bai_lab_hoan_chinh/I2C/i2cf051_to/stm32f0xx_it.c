#include "stm32f0xx_it.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

//void I2C1_ACK_IRQHandler(void)
//{
	/* if (I2C_GetAddressMatched(I2C1)==0x30){
			  uint8_t temp1= I2C_ReceiveData(I2C1);
			  if(temp1==0)
			   {
			  			 GPIOC->ODR=0x0300; Delay(4000000);
			  			  	  GPIOC->ODR=0x000;
			  			  			  Delay(4000000);
			   }
			  I2C_AcknowledgeConfig(I2C1,ENABLE);
			  	//GPIOC->ODR=0x0300; Delay(4000000);
			  			 		 	  		//  GPIOC->ODR=0x000;
			  			 		 	  			//  Delay(4000000);


		  }*/
//}
//void I2C1_RECEIVE_IRQHandler(void)
//
	/* while(I2C_GetFlagStatus(I2C2, I2C_ISR_RXNE) == RESET);
		    /* Read data from RXDR */
		 //   GPIOC->ODR=0x0300; Delay(4000000);

		  //uint8_t temp= I2C_ReceiveData(I2C2);

		//  GPIOC->ODR=0x000; Delay(4000000);*/
//}
/**
  * @brief  This function handles I2Cx event interrupt request.
  * @param  None
  * @retval None
  */
// uint32_t Event = 0;
//volatile uint8_t Tx_Idx;
//volatile uint8_t Rx_Idx;
//extern uint8_t SlaveTxBuffer[];
//extern uint8_t SlaveRxBuffer[];

