#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx.h"

#define STACK_SIZE  ( ( unsigned short ) 100 ) // Kich thuoc stack danh cho 1 task

// Do uu tien cua tien trinh nam trong khoang 0->MAX_PRIORITY
#define PRIORITY ( ( unsigned portBASE_TYPE ) 0U )

/* Khai bao cac Task */
static void vTask1( void *pvParameters );
static void vTask2( void *pvParameters );
 //........

int main(void)
	{
		// Tao 2 Task
		xTaskCreate( vTask1, ( signed portCHAR * ) "LED_PD12", STACK_SIZE, NULL,PRIORITY, NULL);
		xTaskCreate( vTask2, ( signed portCHAR * ) "LED_PD13", STACK_SIZE, NULL,PRIORITY, NULL);
		/* KHOI TAO BO LAP LICH  */
		vTaskStartScheduler();
		return 0;
	}
	// DINH NGHIA CHI TIET NOI DUNG TUNG TASK
	static void vTask1( void *pvParameters ){
		//dieu khien 1
	}
	static void vTask2( void *pvParameters ){
		//dieu khien 2
	}
