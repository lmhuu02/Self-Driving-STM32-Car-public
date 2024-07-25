#include "stm32f0xx.h"
void USART_init(void)
{
  // GPIOA Periph clock enable
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // PA9 and PA10 Alternate function mode
  GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
  // Set alternate functions AF1 for PA9 and PA10
  GPIOA->AFR[1] |= 0x00000110; //GPIOA10,9 as USART1_Rx,Tx
  // USART1 clock enable
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  // 115200 Bd @ 48 MHz
  // USARTDIV = 48 MHz / 115200 = 416 = 0x01A0
  // BRR[15:4] = USARTDIV[15:4]
  // When OVER8 = 0, BRR [3:0] = USARTDIV [3:0]
  USART1->BRR = (uint16_t)(0x01A0); //USART1->BRR = (48000000/115200)
  // USART enable
  // Receiver enable
  // Transmitter enable
  USART1->CR1 = (uint32_t)(USART_CR1_UE |USART_CR1_RE |USART_CR1_TE);
 }

void USART_putc(char c)
{
  // Wait for Transmit data register empty
  while((USART1->ISR & USART_ISR_TXE) == 0){;}

  // Transmit data by writing to TDR
  USART1->TDR = c;
}

void USART_putstr(char *str)
{
  while(*str)
  {
    if(*str == '\n')
    {
      USART_putc('\r');
    }

    USART_putc(*str++);
  }
}

char USART_getc(void)
{
  char c;

  // Was there an Overrun error?
  if((USART1->ISR & USART_ISR_ORE) != 0)
  {
    // Yes, clear it
    USART1->ICR |= USART_ICR_ORECF;
  }
  // Wait for data in the Receive Data Register
  while((USART1->ISR & USART_ISR_RXNE) == 0){;}
  // Read data from RDR
  c = (char)USART1->RDR;
  return(c);
}

int main(void)
{
  char c;

  // --------------------------------------------------------------------------
  // Setup PC8 (blue LED)
  // GPIOC Periph clock enable
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // PC8 and PC9 in output mode
  GPIOC->MODER |= (GPIO_MODER_MODER8_0) ;
  // Push pull mode selected
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8) ;
  // Maximum speed setting (even though it is unnecessary)
  GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8);
  // Pull-up and pull-down resistors disabled
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);// PC9
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
   // PC8 and PC9 in output mode
   GPIOC->MODER |= (GPIO_MODER_MODER9_0) ;
   // Push pull mode selected
   GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8) ;
   // Maximum speed setting (even though it is unnecessary)
   GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9);
   // Pull-up and pull-down resistors disabled
   GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);// Pc7
   RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    // PC8 and PC9 in output mode
    GPIOC->MODER |= (GPIO_MODER_MODER7_0) ;
    // Push pull mode selected
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7) ;
    // Maximum speed setting (even though it is unnecessary)
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR7);
    // Pull-up and pull-down resistors disabled
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);

  // --------------------------------------------------------------------------
  // Setup USART1 (PA9 & PA10)
  USART_init();
  USART_putstr("Demo USART1\n");
  USART_putstr("DAI HOC CNTT & TT\n");
  USART_putstr("ho mau viet\n");
  USART_putstr("KTYS\n");
  USART_putstr("********************\n\n\n");
  USART_putstr("Nhan m de mo den, nhan t de tat den \n ");


  while(1)
  {
	c = USART_getc();
    USART_putc(c);
    USART_putstr("\n");
    if(c=='o')//Kiem tra co nhan m khong, neu co thi mo den
                {
                    GPIOC->BSRR = GPIO_BSRR_BS_8; //pc8=1
                    USART_putstr("Ban da mo den\n\n");
                    }
            else if(c=='f') //Kiem tra co nhan t khong, neu co thi tat den

                {
                    GPIOC->BSRR = GPIO_BSRR_BR_8;//pc8=0
                    USART_putstr("Ban da tat den\n\n");
                    }// pc9
    if(c=='m')//Kiem tra co nhan m khong, neu co thi mo den
                   {
                       GPIOC->BSRR = GPIO_BSRR_BS_9; //pc8=1
                       USART_putstr("Ban da mo den\n\n");
                       }
               else if(c=='t') //Kiem tra co nhan t khong, neu co thi tat den

                   {
                       GPIOC->BSRR = GPIO_BSRR_BR_9;//pc8=0
                       USART_putstr("Ban da tat den\n\n");
                       }//PC7
    if(c=='b')//Kiem tra co nhan m khong, neu co thi mo den
                   {
                       GPIOC->BSRR = GPIO_BSRR_BS_7; //pc8=1
                       USART_putstr("Ban da mo den\n\n");
                       }
               else if(c=='x') //Kiem tra co nhan t khong, neu co thi tat den

                   {
                       GPIOC->BSRR = GPIO_BSRR_BR_7;//pc8=0
                       USART_putstr("Ban da tat den\n\n");
                       }
    }
}
