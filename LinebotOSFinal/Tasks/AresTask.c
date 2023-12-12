

#include "AresTask.h"
#include "CustomProtocol.h"

#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "EncoderPositioning.h"
#include "DriverDbgUSART.h"
#include "PowerManagement.h"

#include "NavigationTask.h"
#include "ReturnHomeTask.h"

//Private variables
uint8_t test;

//Function definitions
void InitAresTask()
{
  InstructionQueue = xQueueCreate(5, sizeof(navigationInstruction));

  USART_RX_Count = 0;
  
  xTaskCreate(WorkerAres, "Ares", 512, NULL, tskIDLE_PRIORITY+2, &AresTaskHandle);
  xTaskCreate(WorkerNavigation, "Navigate", 1024, NULL, tskIDLE_PRIORITY+3, &NavigationTaskHandle);
  //Create task to periodically check if the USART RX Queue has data
  xTaskCreate(WorkerUSARTCheck, "USARTCheck", 512, NULL, tskIDLE_PRIORITY+1, &USARTCheckTaskHandle);

  printf("Ares related tasks initiated\n");
	vTaskSuspend(NavigationTaskHandle);
}

void WorkerAres(void *pvParameters)
{
	uint16_t totalLength;
	uint8_t instructionCount;
	uint8_t *msg =NULL;
  
  stmInterruptSet();
	while(1)
	{
		uint32_t ulNotificationValue;
		if (USART_RX_Count == 0) {
			  vTaskSuspend(NULL);
		}
		if (USART_RX_Count>0) {
			//Debug print
			printf("Processing instructions \n");
			USART_RX_Count -= 1;
			// Step 1: Read the first byte to get the instruction count
			instructionCount = fgetc(stdin);
			// Step 2: Calculate the total message length
			totalLength = instructionCount * sizeof(navigationInstruction);
			msg = malloc(totalLength * sizeof(uint8_t)); // Create a buffer to store the message
			// Step 3: Read the rest of the message
			for (uint16_t i = 0; i < totalLength-1; i++) {
				test = fgetc(stdin);
				msg[i] = 0x11;
				//memcopy test to Msg[i]
				// memcpy(*Msg+i, &test, 1);
				printf("Test %d: %02x ",i, test);
				if (i!=0){
				  printf("Prev Msg: %02x ", msg[i-1]);
				}
				printf("Msg: %02x\n", msg[i]);
			  }
      printf("Msg: ");
      for (uint16_t i = 0; i < totalLength-1; i++) {
          printf("%02x ", msg[i]);
      }
      printf("\n");
	  
	  vTaskDelay(1000);

			// Step 4: Process the message
      navigationInstruction *instructions = malloc(instructionCount * sizeof(navigationInstruction));
			// Decode the instructions
			for (uint16_t i = 0; i < instructionCount; i++) {
				memcpy(&instructions[i], &msg[i * sizeof(navigationInstruction)], sizeof(navigationInstruction));
				xQueueSendToBack(InstructionQueue, &instructions[i], 0);	
			} 
			// Print the instructions
			printInstructions(instructions, instructionCount);
			  // Free the allocated memory
			  free(msg);
			  free(instructions);

			//Start the navigation task
			vTaskResume(NavigationTaskHandle);
		}
	}
}

// void WorkerNavigation(void *pvParameters)
// {
// 	while(1)
// 	{
// 		navigationInstruction instruction;
// 		if (xQueueReceive(InstructionQueue,&instruction,0) == pdPASS)
// 		{
//       if (!inMotion) {
//         inMotion = 1;
//       }
// 			switch (instruction.instructionType) {
// 				case RIGHT:
// 							doRotateCenter(90, 100);
//               vTaskResume(EncoderPositioningTaskHandle);
// 							doDriveStraight(instruction.instructionValue, 100);
//               vTaskSuspend(EncoderPositioningTaskHandle);
// 							break;
// 				case LEFT:
// 							doRotateCenter(-90, 100);
//               vTaskResume(EncoderPositioningTaskHandle);
// 							doDriveStraight(instruction.instructionValue, 100);
//               vTaskSuspend(EncoderPositioningTaskHandle);
// 							break;
// 				case FORWARD:
//               vTaskResume(EncoderPositioningTaskHandle);
// 							doDriveStraight(instruction.instructionValue, 100);
//               vTaskSuspend(EncoderPositioningTaskHandle);
// 							break;
// 				case BACKWARD:
//               vTaskResume(EncoderPositioningTaskHandle);
// 							doDriveStraight(-instruction.instructionValue, 100);
//               vTaskSuspend(EncoderPositioningTaskHandle);
// 							break;
// 				case CLOCKWISE:
// 							doRotateCenter(instruction.instructionValue, 100);
// 							break;
// 				case COUNTERCLOCKWISE:
// 							doRotateCenter(-instruction.instructionValue, 100);
// 							break;
// 				case STOP:
// 						  //Stop encoder positioning task
// 						  resetEncoderPosition();
// 						  inMotion = 0;

// 						  //Send data to STM32
// 						  transmitDataToSTM();
// 							break;
// 				case MEASURE:
// 							break;
// 				case SPEED:
// 							break;
// 				default:
// 							break;
// 			}
// 		} else {
// 			vTaskSuspend(NULL);
// 		}
// 	}
// }

//Worker task to periodically check if the USART RX Queue has data
void WorkerUSARTCheck(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // 10ms

  // Initialize the last wake time
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Check if USART_RX_transmission_complete
    if (USART_RX_transmission_complete)
    {
      // Clear the flag
      USART_RX_transmission_complete = 0;
      // Notify AresTask
      vTaskResume(AresTaskHandle);
    }
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}