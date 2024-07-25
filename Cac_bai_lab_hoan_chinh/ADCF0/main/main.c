#include "main.h"

__IO uint16_t  ADC1ConvertedValue = 0;

float temp;

uint8_t str[10];
void delay()
{
int time;
for(time=0;time<4000000;time++);
}
// khoi tao uart
void GPIO_setup(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    // PORTB
		// D7 - PB7
		// D6 - PB6
		// D5 - PB5
		// D4 - PB4
GPIO_InitStructure.GPIO_Pin    	= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOB, &GPIO_InitStructure);
   
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
		// RS - PC10
		// RW - PC11
		// EN - PC12
GPIO_InitStructure.GPIO_Pin    	= GPIO_Pin_12 | GPIO_Pin_11|GPIO_Pin_10;
GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOC, &GPIO_InitStructure);   	 
}
void adc_setup(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
ADC_InitTypeDef  ADC_InitStructure;
    // clock to PA
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    /* ADC1 Periph clock enable */  
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Configure ADC Channel_10 as analog input */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADCs DeInit */
ADC_DeInit(ADC1);

   /* Initialize ADC structure */
ADC_StructInit(&ADC_InitStructure);

  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
ADC_Init(ADC1, &ADC_InitStructure);
ADC_ChannelConfig(ADC1, ADC_Channel_10 , ADC_SampleTime_239_5Cycles);
ADC_GetCalibrationFactor(ADC1);
ADC_Cmd(ADC1, ENABLE);
while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
ADC_StartOfConversion(ADC1);
}
void read_adc()
{
while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
ADC1ConvertedValue =ADC_GetConversionValue(ADC1);
			
}
int main(void)
{
GPIO_setup();
LCD16X2_Init ();
adc_setup();
	

LCD16X2_PutString("NHIET DO:");
delay(50);
LCD16X2_Gotoxy(0,1);
		
while(1)
{
read_adc();// doc ADC
temp=(float)ADC1ConvertedValue/16.384; //(2.048*8)=16,384
sprintf(str,"   %3.2f DO C",temp);
LCD16X2_Gotoxy(0,1);
LCD16X2_PutString(str);
	if (temp>=25 || temp<=18)
		GPIOB->ODR&=~0xF;
	else 
		GPIOB->ODR|=0xF;
delay(1000000);
}
}
