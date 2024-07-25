/******************************************************************************
  *Ten Tep   :   LCD16X2.h
  *Ngay      :   13/03/2014
  *Tac Gia   :   MinhHa R&D Team
  *Cong Ty   :   MinhHaGroup
  *Webside   :   BanLinhKien.vn
  *Phien Ban :   V1.0
  *Tom Tat   :   Khai bao cac ham dieu khien LCD 16x2.
  *
  ******************************************************************************
  * Chu Y    :   Phai dinh nghia cac chan su dung cho LCD16X2 vao file main.h
  * Vi Du    :   Kit AVR V2
    #define     LCD16X2_RS     PORTB_1
    #define     LCD16X2_RW     PORTC_2
    #define     LCD16X2_EN     PORTC_3
    #define     LCD16X2_D4     PORTD_4
    #define     LCD16X2_D5     PORTD_5
    #define     LCD16X2_D6     PORTD_6
    #define     LCD16X2_D7     PORTD_7

******************************************************************************
    USER LIB        USER LIB        USER LIB        USER LIB
*******************************************************************************/
#ifndef __LCD16X2_H
#define __LCD16X2_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "Main.h"
/*******************************************************************************
Noi Dung     :   Gui tin hieu Enable den LCD16X2.
Tham Bien    :   Khong.
Tra Ve       :   Khong.
*******************************************************************************/
void LCD16X2_Enable(void);

/*******************************************************************************
Noi Dung     :   Gui 4 bit du lieu den LCD16X2.
Tham Bien    :   Data: 4 bit thap cua Data chua 4 bit du lieu can gui.
Tra Ve       :   Khong.
*******************************************************************************/
void LCD16X2_Send4Bit( unsigned char  Data1 );

/*******************************************************************************
Noi Dung     :   Gui 1 byte du lieu den LCD16X2.
Tham Bien    :   command: lenh can ghi
Tra Ve       :   Khong.
*******************************************************************************/
void LCD16X2_SendCommand (unsigned char command );

/*******************************************************************************
Noi Dung     :   Khoi tao LCD16X2.
Tham Bien    :   Khong.
Tra Ve       :   Khong.
*******************************************************************************/
 void LCD16X2_Init ( void );

/*******************************************************************************
Noi Dung     :   Thiet lap vi tri con tro LCD16X2.
Tham Bien    :   x: vi tri cot cua con tro. x = 0 - 15.
                 y: vi tri hang cua con tro. y= 0,1.
Tra Ve       :   Khong.
*******************************************************************************/
void LCD16X2_Gotoxy(unsigned char  x, unsigned char  y);

/*******************************************************************************
Noi Dung     :   Xoa noi dung hien thi tren LCD16X2.
Tham Bien    :   Khong.
Tra Ve       :   Khong.
*******************************************************************************/
void LCD16X2_Clear(void);

/*******************************************************************************
Noi Dung     :   Viet 1 ki tu len LCD16X2.
Tham Bien    :   Data  :   Gia tri ki tu can hien thi.
Tra Ve       :   Khong.
*******************************************************************************/
 void LCD16X2_PutChar ( unsigned char  Data );

/*******************************************************************************
Noi Dung     :   Viet 1 chuoi ki tu len LCD16X2.
Tham Bien    :   *s   :  chuoi du lieu can hien thi.
Tra Ve       :   Khong.
*******************************************************************************/
void LCD16X2_PutString (unsigned char *s);

#ifdef __cplusplus
}
#endif

#endif
/******************************KET THUC FILE************************************
______________________________ MinhHa R&D Team________________________________*/



