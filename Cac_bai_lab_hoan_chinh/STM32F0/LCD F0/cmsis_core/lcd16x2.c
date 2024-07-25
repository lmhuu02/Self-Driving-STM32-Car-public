/*******************************************************************************
  *Ten Tep     :      LCD16X2.c
  *Ngay        :      13/03/2014
  *Tac Gia     :      MinhHa R&D Team
  *Cong Ty     :      MinhHaGroup
  *Webside     :      BanLinhKien.vn
  *Phien Ban   :      V1.0
  *Tom Tat     :      Dinh nghia cac ham dieu khien LCD16X2 16x2.
  *
  ******************************************************************************
  *Chu Y       :      Phai dinh nghia cac chan su dung cho LCD16X2 vao file main.h
  *Vi Du       :      Kit AVR V2

    #define     LCD16X2_RS     PORTB_1
    #define     LCD16X2_RW     PORTC_2
    #define     LCD16X2_EN     PORTC_3
    #define     LCD16X2_D4     PORTD_4
    #define     LCD16X2_D5     PORTD_5
    #define     LCD16X2_D6     PORTD_6
    #define     LCD16X2_D7     PORTD_7
*******************************************************************************/
/*********************************** VI DU *************************************
   // khai bao 1 mang ki tu dung cho LCD16X2
   uint8_t str[16];
   // cac tham bien khac
   uint8_t Do_am;
   //Khoi tao LCD16X2
   LCD16X2_Init();
   // hien thi 1 chuoi ki tu
   sprintf(str,"BanLinhKien.Vn");
   LCD16X2_Puts(str);
   sprintf(str,"Do Am: %u",Do_am);
   LCD16X2_Gotoxy(0,1);
   LCD16X2_Puts(str);
*******************************************************************************/
#include "LCD16x2.h"
/*******************************************************************************
Noi Dung      :   Gui tin hieu Enable den LCD16X2.
Tham Bien     :   Khong.
Tra Ve        :   Khong.
*******************************************************************************/
void delay_us(unsigned int time)
	{
	unsigned int i;
		while(time--) for(i=0;i<200;i++);	
	}
/*******************************************************************************
Noi Dung      :   Gui tin hieu Enable den LCD16X2.
Tham Bien     :   Khong.
Tra Ve        :   Khong.
*******************************************************************************/
	
void delay_ms(unsigned int time)
	{
	
	while(time--) delay_us(1000);	
	}
/*******************************************************************************
Noi Dung      :   Gui tin hieu Enable den LCD16X2.
Tham Bien     :   Khong.
Tra Ve        :   Khong.
*******************************************************************************/
 void LCD16X2_Enable(void)
{
    GPIO_SetBits(GPIOC,LCD16X2_EN);
    delay_us(3);
    GPIO_ResetBits(GPIOC,LCD16X2_EN);
    delay_us(50);
}

/*******************************************************************************
Noi Dung      :   Gui 4 bit du lieu den LCD16X2.
Tham Bien     :   Data: 4 bit thap cua Data chua 4 bit du lieu can gui.
Tra Ve        :   Khong.
*******************************************************************************/
 void LCD16X2_Send4Bit( unsigned char  Data1 )
{
    LCD16x2_Data=(LCD16x2_Data&0xff0f)|(Data1<<4);
}

/*******************************************************************************
Noi Dung      :   Gui 1 byte du lieu den LCD16X2.
Tham Bien     :   command: lenh can ghi
Tra Ve        :   Khong.
*******************************************************************************/
 void LCD16X2_SendCommand (unsigned char  command )
{
    LCD16X2_Send4Bit  ( command >>4 );   /* Gui 4 bit cao */
    LCD16X2_Enable () ;
    LCD16X2_Send4Bit  ( command  );      /* Gui 4 bit thap*/
    LCD16X2_Enable () ;
}

/*******************************************************************************
Noi Dung     :   Khoi tao LCD16X2.
Tham Bien    :   Khong.
Tra Ve       :   Khong.
*******************************************************************************/
 void LCD16X2_Init ( void )
{
    LCD16X2_Send4Bit(0x00);
    delay_ms(20);
		GPIO_ResetBits(GPIOC,LCD16X2_RS);
		GPIO_ResetBits(GPIOC,LCD16X2_RW);
    LCD16X2_Send4Bit(0x03);
    LCD16X2_Enable();
    delay_ms(5);
    LCD16X2_Enable();
    delay_us(100);
    LCD16X2_Enable();
    LCD16X2_Send4Bit(0x02);
    LCD16X2_Enable();
    LCD16X2_SendCommand( 0x28);// giao thuc 4 bit, hien thi 2 hang, ki tu 5x7
    LCD16X2_SendCommand( 0x0c);// cho phep hien thi man hinh
    LCD16X2_SendCommand( 0x06);// tang ID, khong dich khung hinh
    LCD16X2_Clear();           // xoa toan bo khung hinh
}

/*******************************************************************************
Noi Dung     :   Thiet lap vi tri con tro LCD16X2.
Tham Bien    :   x: vi tri cot cua con tro. x = 0 - 15.
                 y: vi tri hang cua con tro. y= 0,1.
Tra Ve       :   Khong.
********************************************************************************/
void LCD16X2_Gotoxy(unsigned char  x, unsigned char  y)
{
  uint8_t  address;
  if(!y)
        address = (0x80+x);
  else
        address = (0xC0+x);

  delay_ms(1);
  LCD16X2_SendCommand(address);
  delay_ms(5);
}

/*******************************************************************************
Noi Dung     :   Xoa noi dung hien thi tren LCD16X2.
Tham Bien    :   Khong.
Tra Ve       :   Khong.
********************************************************************************/
void LCD16X2_Clear(void)
{
    LCD16X2_SendCommand(0x01);
    delay_ms(5);
}

/*******************************************************************************
Noi Dung    :   Viet 1 ki tu len LCD16X2.
Tham Bien   :   Data  :   Gia tri ki tu can hien thi.
Tra Ve      :   Khong.
********************************************************************************/
 void LCD16X2_PutChar ( unsigned char  Data )
{
		GPIO_SetBits(GPIOC,LCD16X2_RS);
    LCD16X2_SendCommand( Data );
		GPIO_ResetBits(GPIOC,LCD16X2_RS);
}

/*******************************************************************************
Noi Dung    :   Viet 1 chuoi ki tu len LCD16X2.
Tham Bien   :   *s   :  chuoi du lieu can hien thi.
Tra Ve      :   Khong.
********************************************************************************/
void LCD16X2_PutString (unsigned char *s)
{
    while (*s)
    {
        LCD16X2_PutChar(*s);
        s++;
    }
}
