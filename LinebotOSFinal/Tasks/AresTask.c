#include "hwconfig.h"

#include "AresTask.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "CustomProtocol.h"

#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

QueueHandle_t InstructionQueue;

void WorkerADC(void *pvParameters);
void WorkerNavigation(void *pvParameters);

//Function definitions
void InitAresTask()
{
	InstructionQueue=xQueueCreate(5,sizeof(navigationInstruction));
	xTaskCreate( WorkerAres, "Ares", 512, NULL, tskIDLE_PRIORITY+2, NULL );
	xTaskCreate( WorkerNavigation, "Navigate", 1024, NULL, tskIDLE_PRIORITY+1, NULL );
}

void WorkerAres(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = 10; // 10 ms

	xLastWakeTime=xTaskGetTickCount();
	while(1)
	{
        if(USART_RX_Queue_has_data && USART_RX_transmission_complete) {
			USART_RX_Queue_has_data = 0;
			uint8_t Msg[201];		//length of 201 to support up to 5 navigationInstructions and one byte for instructionCount
			fgets(Msg,200,stdin);
			uint8_t instructionCount = Msg[0];
			navigationInstruction instructions[instructionCount];

			// Decode the instructions
			for (uint16_t i = 0; i < instructionCount; i++) {
				memcpy(&instructions[i], &Msg[i * sizeof(navigationInstruction) + 1], sizeof(navigationInstruction));
				xQueueSendToBack(InstructionQueue, &instructions[i], 0);
    		}
			vTaskDelayUntil( &xLastWakeTime, xPeriod );	
        }
	}
}

void WorkerNavigation(void *pvParameters)
{
	while(1)
	{
		navigationInstruction instruction;
		if (xQueueReceive(InstructionQueue,&instruction,0) == pdPASS)
		{
			switch (instruction.instructionType) {
				case RIGHT:
							break;
				case LEFT:
							break;
				case FORWARD:
							break;
				case BACKWARD:
							break;
				case CLOCKWISE:
							break;
				case COUNTERCLOCKWISE:
							break;
				case STOP:
							break;
				case MEASURE:
							break;
				case SPEED:
							break;
				default:
							break;
			}
		}
	}
}




