#include "NavigationTask.h"

#include "CustomProtocol.h"

#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
// #include "EncoderPositioning.h"
#include "DriverDbgUSART.h"
#include "PowerManagement.h"
#include "DriverOled.h"
#include "hwconfig.h"

#include "MotionCommands.h"

#include "Globals.h"

void WorkerNavigation(void *pvParameters)
{
	while(1)
	{
		navigationInstruction instruction;
		if (xQueueReceive(InstructionQueue,&instruction,0) == pdPASS)
		{
      // if (!inMotion) {
      //   inMotion = 1;
      // }
			switch (instruction.instructionType) {
				case RIGHT:
							doRotateCenter(90, 200);
              inMotion = 1;
							doDriveStraight(instruction.instructionValue, 255);
              inMotion = 0;
							break;
				case LEFT:
							doRotateCenter(-90, 200);
              inMotion = 1;
							doDriveStraight(instruction.instructionValue, 255);
              inMotion = 0;
							break;
				case FORWARD:
              inMotion = 1;
							doDriveStraight(instruction.instructionValue, 255);
              inMotion = 0;
							break;
				case BACKWARD:
              inMotion = 1;
							doDriveStraight(-instruction.instructionValue, 255);
              inMotion = 0;
							break;
				case CLOCKWISE:
              inMotion = 0;
							doRotateCenter(instruction.instructionValue, 100);
							break;
				case COUNTERCLOCKWISE:
              inMotion = 0;
							doRotateCenter(-instruction.instructionValue, 100);
							break;
				case STOP:
						  //Stop encoder positioning task
						  inMotion = 0;
						  //Send data to STM32
						  transferToSTM();
							// Circular angle accuracy test
							//navigationInstruction testInstructions[] = {
								//{COUNTERCLOCKWISE,90},
								//{COUNTERCLOCKWISE,90},
								//{COUNTERCLOCKWISE,90},
								//{COUNTERCLOCKWISE,90},
								//{COUNTERCLOCKWISE,90},
								//{COUNTERCLOCKWISE,90},
								//{COUNTERCLOCKWISE,90},
								//{COUNTERCLOCKWISE,90},
								//{STOP, 0}
							//};
							//Triangle accuracy test
							// navigationInstruction testInstructions[] = {
							// 	{FORWARD, 1000},
							// 	{COUNTERCLOCKWISE, 90},
							// 	{FORWARD, 500},
							// 	{COUNTERCLOCKWISE, 135},
							// 	{FORWARD, 1118.033989},
							// 	{COUNTERCLOCKWISE, 135},
							// 	{STOP, 0}
							// };

							// // Get the number of instructions
							// int numInstructions = sizeof(testInstructions) / sizeof(navigationInstruction);
							// printf("NumInstructions: %d\n", numInstructions);
							// // Enqueue the instructions
							// for (int i = 0; i < numInstructions; i++) {
							// 	// Enqueue the instruction into the InstructionQueue
							// 	// Replace "InstructionQueue" with the actual name of your queue
							// 	xQueueSend(InstructionQueue, &testInstructions[i], 0);
							// }
							break;
				case MEASURE:
							break;
				case SPEED:
							break;
				default:
							break;
			}
		} else {
			//vTaskSuspend(NULL);
			vTaskDelay(1000);
		}
	}
}


void transferToSTM() {
	//Wake up the Stm32 with an interrupt on pin
	stmInterruptClear();
	vTaskDelay(10);
	stmInterruptSet();
	transmitDataToSTM();
}

//Function to transmit Acc and Gyro data to the STM32
void transmitDataToSTM() {
	//Send data to STM32
  LineBotToSTM32Message message;



  message.startDelimiter = START_DELIMITER;
  message.xCoord = getEncoderXCoord();
  message.yCoord = getEncoderYCoord();
  message.yaw = getEncoderYaw();
  message.endDelimiter = END_DELIMITER;

  transmitPositionData(&message);
	vTaskDelay(100);
	enterSleepMode();
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
void transmitPositionData(LineBotToSTM32Message* message) {

    char* rawBytes = (char*)message;
    for(int i = 0; i < sizeof(LineBotToSTM32Message); i++) {
        putchar(rawBytes[i]);
    }
    //printf("Sleeping/n");
	//printf("Continuing/n");
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