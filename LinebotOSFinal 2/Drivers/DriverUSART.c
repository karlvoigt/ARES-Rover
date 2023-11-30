#include "DriverUSART.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "hwconfig.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "AresTask.h"


#include "CustomProtocol.h"

static int stdio_putchar(char c, FILE * stream);
static int stdio_getchar(FILE *stream);
static FILE UsartStdio = FDEV_SETUP_STREAM(stdio_putchar, stdio_getchar,_FDEV_SETUP_RW);

QueueHandle_t UsartTxQueue;
QueueHandle_t UsartRxQueue;

uint8_t receiveStarted = 0;

void DriverUSARTInit(void)
{
	UsartTxQueue=xQueueCreate(UART_QUEUE_LENGTH,sizeof(char));
	UsartRxQueue=xQueueCreate(UART_QUEUE_LENGTH,sizeof(char));
		
	USART_PORT.DIRSET=0b00001000;	
	USART_PORT.DIRCLR=0b00000100;
	
	USART.CTRLA=0b00111000;
	USART.CTRLB=0b00011000; 
	USART.CTRLC=0b00000011;
	
	
	//TODO: change baud to 115200
	USART.BAUDCTRLA=0xE5; //BSEL=3301, BSCALE=-5 19200 baud
	USART.BAUDCTRLB=0xBC; 
	
	stdout=&UsartStdio;
	stdin=&UsartStdio;
	

}


static int stdio_putchar(char c, FILE * stream)
{
	int res;
	char cbuf;

	xQueueSend(UsartTxQueue,&c,portMAX_DELAY);

	if (USART.STATUS & (1<<5))
	{
		xQueueReceive(UsartTxQueue,&cbuf,0);
		USART.DATA=cbuf;
	}
		
	return 0;
}
	
static int stdio_getchar(FILE *stream)
{
	char c;
	xQueueReceive(UsartRxQueue,&c,portMAX_DELAY);
	return c;
}



ISR(USART_TXC_vect)
{
	char c;
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	if (xQueueReceiveFromISR(UsartTxQueue,&c,&xHigherPriorityTaskWoken)==pdPASS)
	{
		USART.DATA=c;	
	}
	
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


//Check for delimiters in ISR
ISR(USART_RXC_vect)
{
	char c;
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;
	
	c=USART.DATA;
	if (receiveStarted==1) {
		if (c==END_DELIMITER) {
			receiveStarted=0;
			USART_RX_transmission_complete = 1;
			vTaskNotifyGiveFromISR(AresTaskHandle,&xHigherPriorityTaskWoken);
		} else {
			USART_RX_Queue_has_data = 1;
			xQueueSendToBackFromISR(UsartRxQueue,&c,&xHigherPriorityTaskWoken);
		}
	} else if (c==START_DELIMITER) {
		receiveStarted=1;
		USART_RX_transmission_complete = 0;
	}
	// xQueueSendToBackFromISR(UsartRxQueue,&c,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
}