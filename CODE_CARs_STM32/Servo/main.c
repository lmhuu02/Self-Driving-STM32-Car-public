/*--------------------------------------------------------------------------------*/
/* Project: selfdrivingcar use stm32f407 discovery
 * structure: STM32F407 discovery, servo sg90s, untrasonic sensor srf05, H bridge circuit L298N, Motor DC 12v.
 * IDE: CooCox CoIDE, STLink, hercules 328
 * Language: C
 * Time: 16/6/2024		add: ICTU ThaiNguyen
 * Adviser: Ngo Thi Vinh
 * Coder: Le Minh Huu*, Dinh Qang Ha, Dinh Bach Dang.
 * File: test servo 0, 180
 *
 * */
/*--------------------------------------------------------------------------------*/
/* Describe: use timer2, pwm control servo pin PA5 GPIO A STM32
 * Reference:
 * https://www.instructables.com/Servo-Motor-Control-With-STM32F4-ARM-MCU/
 * https://github.com/Ruturajn/Interfacing-sg90-servo-motor-with-stm32f407-DISC1/blob/main/README.md
 * https://stm32f4-discovery.net/2014/10/library-42-control-rc-servo-stm32f4/
 * https://deepbluembedded.com/stm32-servo-motor-control-with-pwm-servo-library-examples-code/
 * https://blog.embeddedexpert.io/?p=392
 * */
/*--------------------------------------------------------------------------------*/

/*=================================START================================================*/
#include "stm32f4xx.h"

//init function in file
void GPIO_Setup(void);
void TIM2_Init(void);
void TIM4_ms_Delay(uint32_t delay);

/*======================================FUNCTION===========================================*/
void GPIO_Setup(){
    RCC->AHB1ENR |= 1; //Enable GPIOA clock
    GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
    GPIOA->MODER |= 0x00000800; //Set the PA5 pin alternate function
}
void TIM2_Setup(){
    RCC->APB1ENR |=1;
    //bo chia CFGR cho 1 -> 16mhz. bo chia sau = 4 (TIM2->PSC = 4;). 16mhz = 16 000 000 hz / 4 =  4 000 000 hz trong 1 giay
    //-> 4 000 000 hz / 1 000ms -> 1ms = 4 000 hz (clk)
    //de delay 16ms ->  dem 64 000 -1 -> dem 63 999  (TIM2->ARR = 64000-1;)
//    TIM2->PSC = 4; //Setting the clock frequency to 1MHz.//16-1
//    TIM2->ARR = 64000-1; // Total period of the timer

    // bo chia CFGR cho 8 -> 16mhz/8 = 2mhz <=> 2 000 000 hz.
	TIM2->PSC = 2;	//-> 2 000 000 /2 = 1 000 000 hz/ 1s -> 1 000 clk/ 1ms.
	TIM2->ARR = 15999; 	//1 000 clk / 1ms. can 16ms = 16 x 1 000 = 16 000 -> 15 999 lan dem (tinh dem tu 0)
    TIM2->CNT = 0;

    TIM2->CCMR1 = 0x0060; //PWM mode for the timer
    TIM2->CCER |= 1; //Enable channel 1 as output
//    TIM2->CCR1 = 500; // Pulse width for PWM
}

//void TIM4_ms_Delay(uint32_t delay){
//    RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
//    TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
//    TIM4->ARR = (delay); // Total period of the timer
//    TIM4->CNT = 0;
//    TIM4->CR1 |= 1; //Start the Timer
//    while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
//    TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
//}

void delay(int ms)
{
	int i;
	for(; ms>0 ;ms--){
//		for(i =0; i<3195;i++);
		for(i =0; i<4000;i++);	//chi dung de delay
	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_write(uint8_t angle)
{
		if(angle<0){angle=0;}
		if(angle>180){angle=180;}
//		TIM2->CCR1=map (angle,0,180,2500,7000);
		TIM2->CCR1=map (angle,0,180,500,3000);
}
/*================================MAIN=================================================*/
int main(void)
{
		int pos;
//		RCC->CFGR |= 0<<10; // set APB1 = 16 MHz. bo chia cho 1 -> 16mhz/1 = 16mhz
		RCC->CFGR |= 0x6<<10; // APB1 16mhz chia 8 duoc 2 mhz
		GPIO_Setup();
		TIM2_Setup();
		TIM2->CR1 |= 1; // bat timer 2
//	    TIM2->CCR1 = 500;
//	    delay(40000);
//	    TIM2->CCR1 = 0;
//	    TIM4_ms_Delay(500);

	    while(1){
	    	  //servo quay sang phai
	    	  pos = 0;
	    	  servo_write(pos);              // tell servo to go to position in variable 'pos'
	    	  delay(1500);
	    	  //servo quay thang phai truoc
	    	  pos = 110;
			  servo_write(pos);              // tell servo to go to position in variable 'pos'
			  delay(1500);
			  //servo quay sang trai
			  pos = 190;
			  servo_write(pos);              // tell servo to go to position in variable 'pos'
			  delay(2500);
	   }
}
/*=========================================END========================================*/
