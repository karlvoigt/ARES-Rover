

#include "AresTask.h"
#include "CustomProtocol.h"

#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "EncoderPositioning.h"

//Private variables
uint8_t inMotion = 0;

//Function definitions
void InitAresTask()
{
  InstructionQueue = xQueueCreate(5, sizeof(navigationInstruction));
  
  xTaskCreate(WorkerAres, "Ares", 512, NULL, tskIDLE_PRIORITY+2, &AresTaskHandle);
  xTaskCreate(WorkerNavigation, "Navigate", 1024, NULL, tskIDLE_PRIORITY+1, &NavigationTaskHandle);

	vTaskSuspend(NavigationTaskHandle);
}

void WorkerAres(void *pvParameters)
{
  stmInterruptSet();
			uint8_t Msg[201];		//length of 201 to support up to 5 navigationInstructions and one byte for instructionCount
	while(1)
	{
		uint32_t ulNotificationValue;
		BaseType_t xResult = xTaskNotifyWait( pdFALSE, 0, &ulNotificationValue, portMAX_DELAY );
    if(USART_RX_Queue_has_data && USART_RX_transmission_complete) {
			USART_RX_Queue_has_data = 0;
			fgets(Msg,200,stdin);
			uint8_t instructionCount = Msg[0];
			navigationInstruction instructions[instructionCount];
			// Decode the instructions
			for (uint16_t i = 0; i < instructionCount; i++) {
				memcpy(&instructions[i], &Msg[i * sizeof(navigationInstruction) + 1], sizeof(navigationInstruction));
				xQueueSendToBack(InstructionQueue, &instructions[i], 0);	
      }
			// Print the instructions
			printInstructions(instructions, instructionCount);
      //Start the navigation task
      vTaskResume(NavigationTaskHandle);
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
      if (!inMotion) {
        inMotion = 1;
      }
			switch (instruction.instructionType) {
				case RIGHT:
							doRotateCenter(90, 100);
              vTaskResume(EncoderPositioningTaskHandle);
							doDriveStraight(instruction.instructionValue, 100);
              vTaskSuspend(EncoderPositioningTaskHandle);
							break;
				case LEFT:
							doRotateCenter(-90, 100);
              vTaskResume(EncoderPositioningTaskHandle);
							doDriveStraight(instruction.instructionValue, 100);
              vTaskSuspend(EncoderPositioningTaskHandle);
							break;
				case FORWARD:
              vTaskResume(EncoderPositioningTaskHandle);
							doDriveStraight(instruction.instructionValue, 100);
              vTaskSuspend(EncoderPositioningTaskHandle);
							break;
				case BACKWARD:
              vTaskResume(EncoderPositioningTaskHandle);
							doDriveStraight(-instruction.instructionValue, 100);
              vTaskSuspend(EncoderPositioningTaskHandle);
							break;
				case CLOCKWISE:
							doRotateCenter(instruction.instructionValue, 100);
							break;
				case COUNTERCLOCKWISE:
							doRotateCenter(-instruction.instructionValue, 100);
							break;
				case STOP:
              //Stop encoder positioning task
              resetEndcoderPosition();
              inMotion = 0;

              //Send data to STM32
              transmitDataToSTM();
							break;
				case MEASURE:
							break;
				case SPEED:
							break;
				default:
							break;
			}
		} else {
			vTaskSuspend(NULL);
		}
	}
}

void transferToSTM() {
	//Wake up the Stm32 with an interrupt on pin
	stmInterruptClear();
	vTaskDelay(100);
	transmitDataToSTM();
	stmInterruptSet();
}

//Function to transmit Acc and Gyro data to the STM32
void transmitDataToSTM() {
	//Send data to STM32
  LineBotToSTM32Message message;

  message.startDelimiter = START_DELIMITER;
  message.xCoord = getEncoderXCoord();
  message.yCoord = getEncoderYCoord();
  message.yaw = getEncoderYaw();

  transmitPositionData(&message);
}

//Function to print MEMS data
void printMEMSDataText(MEMS_Data* data) {
	printf("Gyro X: %d\n", data->gyroX);
	printf("Gyro Y: %d\n", data->gyroY);
	printf("Gyro Z: %d\n", data->gyroZ);
	printf("Accel X: %d\n", data->accelX);
	printf("Accel Y: %d\n", data->accelY);
	printf("Accel Z: %d\n", data->accelZ);
}

//Function to transmit MEMS data struct over UART
void transmitMEMSData(MEMS_Data* data) {
    fwrite(data, sizeof(MEMS_Data), 1, stdout);
}

//Function to transmit position data over UART
void transmitPositionData(LineBotToSTM32Message* message;) {
    fwrite(&message, sizeof(LineBotToSTM32Message), 1, stdout);
}

// Function to print the instructions
void printInstructions(navigationInstruction* instructions, uint16_t numInstructions) {
  for (uint16_t i = 0; i < numInstructions; i++) {
    printf("Instruction %d: ", i);
    switch (instructions[i].instructionType) {
      case LEFT:
        printf("Turn left and drive %f mm\n", instructions[i].instructionValue);
        break;
      case RIGHT:
        printf("Turn right and drive %f mm\n", instructions[i].instructionValue);
        break;
      case FORWARD:
        printf("Drive %f mm forwards\n", instructions[i].instructionValue);
        break;
      case BACKWARD:
        printf("Drive %f mm backwards\n", instructions[i].instructionValue);
        break;
      case CLOCKWISE:
        printf("Turn clockwise %f degrees\n", instructions[i].instructionValue);
        break;
      case COUNTERCLOCKWISE:
        printf("Turn counterclockwise %f degrees\n", instructions[i].instructionValue);
        break;
      case STOP:
        printf("Stop\n");
        break;
      default:
        printf("Unknown instruction type\n");
        break;
    }
  }
}

//Function to print instructions to the OLED screen
void printOLEDInstructions(navigationInstruction* instructions, uint16_t numInstructions) {
    char buffer[50];
    DriverOLEDClearScreen(); // Clear the screen once before the loop

    for (uint16_t i = 0; i < numInstructions; i++) {
        sprintf(buffer, "Instruction %d: ", i); // Print the instruction number
        DriverOLEDPrintSmText(i, buffer, 0);

        switch (instructions[i].instructionType) {
            case LEFT:
                sprintf(buffer, "Turn left and drive %f mm", instructions[i].instructionValue);
                break;
            case RIGHT:
                sprintf(buffer, "Turn right and drive %f mm", instructions[i].instructionValue);
                break;
            case FORWARD:
                sprintf(buffer, "Drive %f mm forwards", instructions[i].instructionValue);
                break;
            case BACKWARD:
                sprintf(buffer, "Drive %f mm backwards", instructions[i].instructionValue);
                break;
            case CLOCKWISE:
                sprintf(buffer, "Turn clockwise %f degrees", instructions[i].instructionValue);
                break;
            case COUNTERCLOCKWISE:
                sprintf(buffer, "Turn counterclockwise %f degrees", instructions[i].instructionValue);
                break;
            case STOP:
                sprintf(buffer, "Stop");
                break;
            default:
                sprintf(buffer, "Unknown instruction type");
                break;
        }
        DriverOLEDPrintSmText(i+1, buffer, 0); // Print the instruction details
    }

    DriverOLEDUpdate(); // Update the screen once after the loop
}

void stmInterruptPinInit(void)
{
	// Set PA5 as output
    PORTA.DIRSET = 0b00100000;

    // Configure PA5
    PORTA.PIN5CTRL = 0b00001000; // Totem pole out, inverted
}

void stmInterruptSet(void)
{
    // Set PA5 high
    PORTA.OUTSET = 0b00100000;
}

void stmInterruptClear(void)
{
    // Set PA5 low
    PORTA.OUTCLR = 0b00100000;
}

void stmInterruptToggle(void)
{
    // Toggle PA5
    PORTA.OUTTGL = 0b00100000;
}