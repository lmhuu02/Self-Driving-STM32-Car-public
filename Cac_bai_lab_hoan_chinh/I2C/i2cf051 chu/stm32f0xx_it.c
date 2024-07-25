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

/*void I2C2_ER_IRQHandler(void)
{
  /* Check on I2C2 AF flag and clear it */
 // if (I2C_GetITStatus(I2C2, I2C_IT_ADDR))
// {
  // I2C_ClearITPendingBit(I2C2, I2C_IT_AF1);
//}
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

