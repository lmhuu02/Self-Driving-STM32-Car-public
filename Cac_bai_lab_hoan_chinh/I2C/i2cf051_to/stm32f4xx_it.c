#include "stm32f4xx_it.h"

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

void I2C2_ER_IRQHandler(void)
{
  /* Check on I2C2 AF flag and clear it */
  if (I2C_GetITStatus(I2C2, I2C_IT_AF)) 
  {
    I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
  }
}

/**
  * @brief  This function handles I2Cx event interrupt request.
  * @param  None
  * @retval None
  */
volatile uint32_t Event = 0;
volatile uint8_t Tx_Idx;
volatile uint8_t Rx_Idx;
extern uint8_t SlaveTxBuffer[];
extern uint8_t SlaveRxBuffer[];
void I2C2_EV_IRQHandler(void)
{
  /* Get Last I2C Event */
  Event = I2C_GetLastEvent(I2C2);
  switch (Event)
  { 
  /* ****************************************************************************/
  /*                          Slave Transmitter Events                          */
  /*                                                                            */
  /* ****************************************************************************/  
    
  /* Check on EV1 */
  case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:  
    Tx_Idx = 0;
    I2C_ITConfig(I2C2, I2C_IT_BUF , ENABLE);
    break;
  /* Check on EV3 */
  case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
  case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:    
    I2C_SendData(I2C2, SlaveTxBuffer[Tx_Idx++]);
    break;
    
  /* ****************************************************************************/
  /*                              Slave Receiver Events                         */
  /*                                                                            */
  /* ****************************************************************************/ 
    
  /* check on EV1*/
  case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
    Rx_Idx = 0;
    break;
    
  /* Check on EV2*/
  case I2C_EVENT_SLAVE_BYTE_RECEIVED:  
  case (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_SR1_BTF):  
    SlaveRxBuffer[Rx_Idx++] = I2C_ReceiveData(I2C2);
    break;
 
  /* Check on EV4 */
  case I2C_EVENT_SLAVE_STOP_DETECTED:             
    I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF);
    I2C_Cmd(I2C2, ENABLE);
    break;
    
  default:
    break;
  } 
}
