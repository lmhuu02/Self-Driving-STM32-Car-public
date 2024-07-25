#include "stm32f0xx.h"
//#include "stm32f0xx_exti.h"  //Thu vien cau hinh ngat ngoai
#include "stm32f0xx_gpio.h"  // thu vien cau hinh chan vao ra
#include "stm32f0xx_rcc.h"  // cung cấp xung clock cho các chân của cổng vào ra
//#include "stm32f0xx_misc.h"  // thư viện phục vụ cấu hình bộ dk ngắt không vector NVIC
//#include "stm32f0xx_syscfg.h"  // xung hệ thông cần khi sử dụng ngắt

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup EXTI_Examples
  * @{
  */

/** @addtogroup EXTI_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static count=0;
void LEDInit(void);
//void EXTI0_Config(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void EXTI_Example(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */

  /* Initialize LEDs mounted on STM32F0-Discovery kit */

}

/**
  * @brief  Configures LED GPIO.
  * @param  None
  * @retval None
  */

void LED7Init(void) //Dieu khien day led don
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure1;
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//rcc.h


  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// 2 v 10
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);//rcc.h
  GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8 | GPIO_Pin_9;
   GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;// 2 v 10
   GPIO_Init(GPIOC, &GPIO_InitStructure1);
}

/**
  * @brief  Configure PA0 in interrupt mode
  * @param  None
  * @retval None
  */
int main(){
int i;
	 LED7Init();

GPIOA->ODR=8<<4;

GPIOC->ODR=8<<6;

while(1);
}


