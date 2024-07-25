#include "stm32f0xx.h"
#include "stm32f0xx_exti.h"  //Thu vien cau hinh ngat ngoai
#include "stm32f0xx_gpio.h"  // thu vien cau hinh chan vao ra
#include "stm32f0xx_rcc.h"  // cung cấp xung clock cho các chân của cổng vào ra
#include "stm32f0xx_misc.h"  // thư viện phục vụ cấu hình bộ dk ngắt không vector NVIC
#include "stm32f0xx_syscfg.h"  // xung hệ thông cần khi sử dụng ngắt

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
void EXTI0_Config(void);


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
void EXTIA0_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTIA1_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void delay(void)
    {
        int time;
        for(time=0;time<4000000;time++);// tre khoang 4000000 lenh
    }

void EXTI0_1_IRQHandler(void)
{

  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {

    /* Toggle LED3 and LED4 */
  //Tang LED
count++; if (count>99) count=99;
display7(count);
delay();
    /* Clear the EXTI line 0 pending bit */
   EXTI_ClearITPendingBit(EXTI_Line0);
  }
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {

      /* Toggle LED3 and LED4 */
    //Giam LED
	  count--; if (count<0) count=00;
	  display7(count);
	delay();
      /* Clear the EXTI line 0 pending bit */
      EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void display7(int count){
	int x,y;
	x=count/10;//lay phan nguyen
	y=count%10; //lay phan du
	GPIOA->ODR=x<<4;
	GPIOC->ODR=y<<6;
}
int main(){
int i;
	 LED7Init();

	  /* Configure PA0 in interrupt mode */
	  EXTIA0_Config();
	EXTIA1_Config();

	 EXTI_GenerateSWInterrupt(EXTI_Line0 | EXTI_Line1);

  while (1)
 {
	display7(count);

 delay();

}}


