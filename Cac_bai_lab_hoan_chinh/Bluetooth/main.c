#include "main.h"
#include "string.h"
//ket noi UART   
static __IO uint32_t TimingDelay;
void UARTSend(const unsigned char * pucBuffer, unsigned long ulCount);
//các chuoi nhan duoc
char received_string[MAX_STRLEN+1];
uint8_t Buffer[6];
void delay(void){
int time;
for(time=0;time<10000000;time++);
}
// xu ly ban phim, 7 thanh
void init_gpio(){
GPIO_InitTypeDef GPIO_InitStructure;
RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);
//GPIOA led 7 thanh
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_Init(GPIOC, &GPIO_InitStructure);
//GPIOB ban phim cot 
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
GPIO_Init(GPIOB, &GPIO_InitStructure);
//GPIOC ban phim cot
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
GPIO_Init(GPIOC, &GPIO_InitStructure);
//GPIOB ban phim hang R1-R4
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//gui mot ký tu den UART
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
//vòng lap trong khi có nhieu ký tu de gui
  while(ulCount--)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t) *pucBuffer++);
    
  }
}//vong lap cho den khi ket thuc truyen
void display(char number){
		switch(number){ //lua chon
			case  '0': GPIOC->ODR=0x0000; break;
			
			case  '1': GPIOC->ODR=0x0040; break;
			
			case  '2': GPIOC->ODR=0x0080; break;
		
			case  '3': GPIOC->ODR=0x00C0; break;
			
			case  '4': GPIOC->ODR=0x0100; break;
			
			case  '5': GPIOC->ODR=0x0140; break;
			
			case  '6': GPIOC->ODR=0x0180; break;
			
			case  '7': GPIOC->ODR=0x01C0; break;
			
			case  '8': GPIOC->ODR=0x0200; break;
			
			case  '9': GPIOC->ODR=0x0240; break;
	default:
      //hien thi so 9 3 lan
		{	GPIOC->ODR=0x0240;
			delay();
			GPIOC->ODR=0x0000;
			delay();
			GPIOC->ODR=0x0240;}
			}
		  }
// khoi tao cac tb ngoai vi uart
void init_USART1(uint32_t baudrate){
		GPIO_InitTypeDef  GPIO_InitStructure;
   GPIO_InitTypeDef GPIO_InitStruct;// chan GPIO sd nh TX va RX
	USART_InitTypeDef USART_InitStruct; // khoi tao uart
  NVIC_InitTypeDef NVIC_InitStructure; // cau hinh cac NVIC
  // cho phep xung clock cap toi GPIOC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);//bat led tren kit
		// ÐK led tren PC	 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// che do output
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// reset chan khi output
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// khong dung tro keo
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// xung clock 
  GPIO_Init(GPIOC, &GPIO_InitStructure);	// khoi tao PC
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			 
  GPIO_Init(GPIOA, &GPIO_InitStruct);				
	
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	
	 
  USART_InitStruct.USART_BaudRate = 9600;			
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;	
  USART_InitStruct.USART_Parity = USART_Parity_No;		 
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  
  USART_Init(USART1, &USART_InitStruct);					 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
	
	 
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;		
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 
  NVIC_Init(&NVIC_InitStructure);		

  USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void){
  if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
    static uint8_t cnt = 0;
    char t = USART1->RDR; 
		display(t);//Hien thi len LED 7 doan
	
    if( (t != 'n') && (cnt < MAX_STRLEN) ){
      received_string[cnt] = t;
			
      cnt++;
    }
    else{ 
      cnt = 0;
    }
  }
}

void TimingDelay_Decrement(void){
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}
	
//***********************************MAIN**********************************
int main(void) {
  
  unsigned char welcome_str[] = "TU KIT STM32F0\r\n";
  uint8_t loop = 1; 
	init_gpio();
  init_USART1(BT_BAUD);
	// gui dl qua giao tiep blth
	
	UARTSend(welcome_str, sizeof(welcome_str));
received_string[0]='\n';
	
 while(loop){
//code ban phim
GPIOC->BSRR = GPIO_Pin_4;//thiet lap len muc cao
GPIOC->BRR = GPIO_Pin_5;//thiet lap o muc thap
GPIOB->BRR = GPIO_Pin_0;//thiet lap o muc thap
GPIOB->BRR = GPIO_Pin_1;//thiet lap o muc thap
{ 
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)){//doc dau vao bit PB12
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '*'); 
delay(); 
}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)){ //doc dau vao bit PB11
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '7'); 
  delay();
}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)){ //doc dau vao bit PB10
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
			
	USART_SendData(USART1, '4'); delay(); 
}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)){
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '1'); 	//doc dau vao bit PB2
		 delay();
}}

GPIOC->BRR = GPIO_Pin_4;//thiet lap o muc thap
GPIOC->BSRR = GPIO_Pin_5;//thiet lap o muc cao
GPIOB->BRR = GPIO_Pin_0;//thiet lap o muc thap
GPIOB->BRR = GPIO_Pin_1;//thiet lap o muc thap
{ 
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)) {
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '0');  delay();
		}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '8'); delay();
}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)){
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '5'); delay();
}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '2'); delay();
}
} 

GPIOC->BRR = GPIO_Pin_4;//set bit as low
GPIOC->BRR = GPIO_Pin_5;//set bit as low
GPIOB->BSRR = GPIO_Pin_0;//set bit as high
GPIOB->BRR = GPIO_Pin_1;//set bit as low
{
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '#'); delay();
	}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '9'); delay();
	}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '6'); delay();
	}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, '3'); delay();
	}
}

GPIOC->BRR = GPIO_Pin_4;//set bit as low
GPIOC->BRR = GPIO_Pin_5;//set bit as low
GPIOB->BRR = GPIO_Pin_0;//set bit as low
GPIOB->BSRR = GPIO_Pin_1;//set bit as high
{
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, 'D'); delay();
	}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, 'C'); delay();
	}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)){
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, 'B'); delay();
	}
if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)){ 
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	USART_SendData(USART1, 'A'); delay();
	}
  }

}
 	 
USART_Cmd(USART1, DISABLE);//  ngat ket noi uart
}