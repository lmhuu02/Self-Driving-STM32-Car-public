#include "mbed.h"

DigitalOut led0(PA_0);
DigitalOut led1(PA_1);
DigitalOut led2(PA_2);
DigitalOut led3(PA_3);
DigitalOut led4(PA_4);
DigitalOut led5(PA_5);
DigitalOut led6(PA_6);
DigitalOut led7(PA_7);
DigitalOut led8(PB_0);
DigitalOut led9(PB_1);
DigitalOut led10(PB_2);
DigitalOut led11(PB_3);
DigitalOut led12(PB_4);
DigitalOut led13(PB_5);
DigitalOut led14(PB_6);
DigitalOut led15(PB_7);
int a[] = { led0,led1, led2, led3,led4,led5,led6,led7,led8,led9,led10,led11,led12,led13,led14,led15};
int main() {
    while(1) {
        //hieu ung sang het tat ca cac den
        for (int i=0; i<16;i++)   //Lưu ý chỉ số mảng tính từ a[0]
    a[i] =0;                
        wait(3);
        //tat cac den
        for (int i=0; i<16;i++)  //Lưu ý chỉ số mảng tính từ a[0]
    a[i] =1;                
        wait(3);
      
        //sang lan luot tung den 1
        for (int i=0; i<16; i++) {
           a[i] =0;
           wait(1);
        }
        wait(1);
    }
}
