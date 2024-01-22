

#include "AresTask.h"
#include "CustomProtocol.h"

#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "DriverDbgUSART.h"
#include "DriverUSART.h"
#include "PowerManagement.h"

#include "NavigationTask.h"
#include "ReturnHomeTask.h"
#include "GyroTask.h"

//Private variables
uint8_t test;
extern QueueHandle_t UsartRxQueue;
#define VERBOSE 0

//Function definitions
void InitAresTask()
{
  InstructionQueue = xQueueCreate(15, sizeof(navigationInstruction));

  USART_RX_Count = 0;
  
  xTaskCreate(WorkerAres, "Ares", 512, NULL, tskIDLE_PRIORITY+2, &AresTaskHandle);
  xTaskCreate(WorkerNavigation, "Navigate", 1024, NULL, tskIDLE_PRIORITY+3, &NavigationTaskHandle);
  //Create task to periodically check if the USART RX Queue has data
  xTaskCreate(WorkerUSARTCheck, "USARTCheck", 512, NULL, tskIDLE_PRIORITY+1, &USARTCheckTaskHandle);

  if (VERBOSE) printf("Ares related tasks initiated\n");
}

void WorkerAres(void *pvParameters)
{
	uint16_t totalLength;
	uint8_t instructionCount;
  
  stmInterruptSet();
	while(1)
	{
		//Just read out yaw for debugging
		// float yaw, yawRate;
		// GyroGet(&yawRate, &yaw);
		// printf("Yaw: %f\n", yaw);
		// printf("YawRate: %f\n", yawRate);
		// vTaskDelay(500);
		uint32_t ulNotificationValue;
		if (USART_RX_Count == 0) {
			  vTaskSuspend(NULL);
		}
		if (USART_RX_Count>0) {
			//Debug print
			USART_RX_Count -= 1;
			// Step 1: Read the first byte to get the instruction count
			xQueueReceive(UsartRxQueue,&instructionCount,portMAX_DELAY);
			// Step 2: Calculate the total message length
			totalLength = instructionCount * sizeof(navigationInstruction);
      // printf("Instruction count: %d\n", instructionCount);
			uint8_t msg[totalLength]; // Create a buffer to store the message
			// Step 3: Read the rest of the message
			for (uint16_t i = 0; i < totalLength; i++) {
				xQueueReceive(UsartRxQueue,msg+i,portMAX_DELAY);
      }
	  
      //Delay for debug printing, remove during production
      //TODO: Remove this delay
	    vTaskDelay(100);

			// Step 4: Process the message
      navigationInstruction instructions [instructionCount * sizeof(navigationInstruction)];
			// Decode the instructions
			for (uint16_t i = 0; i < instructionCount; i++) {
				memcpy(&instructions[i], &msg[i * sizeof(navigationInstruction)], sizeof(navigationInstruction));
				xQueueSendToBack(InstructionQueue, &instructions[i], 0);	
			} 
			// Print the instructions
			// printInstructions(instructions, instructionCount);
			//Start the navigation task
			vTaskResume(NavigationTaskHandle);
		}
	}
}

void loadTestInstructions() {
    // Define the test instructions
    navigationInstruction testInstructions[] = {
        {FORWARD, 1000},
        {COUNTERCLOCKWISE, 90},
        {FORWARD, 500},
        {COUNTERCLOCKWISE, 45},
        {COUNTERCLOCKWISE, 90},
        {FORWARD, 1118.033989},
		{COUNTERCLOCKWISE, 45},
		{COUNTERCLOCKWISE, 90},
        {STOP, 0}
    };

    // Get the number of instructions
    int numInstructions = sizeof(testInstructions) / sizeof(navigationInstruction);
	if (VERBOSE) ("NumInstructions: %d\n", numInstructions);
    // Enqueue the instructions
    for (int i = 0; i < numInstructions; i++) {
        // Enqueue the instruction into the InstructionQueue
        // Replace "InstructionQueue" with the actual name of your queue
        xQueueSend(InstructionQueue, &testInstructions[i], 0);
    }
	
	// Print the instructions
	if (VERBOSE) printInstructions(testInstructions, numInstructions);
    // Resume the navigation task
    // Replace "NavigationTaskHandle" with the actual handle of your navigation task
    vTaskResume(NavigationTaskHandle);
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