#ifndef NAVIGATIONTASK_H
#define NAVIGATIONTASK_H

// Include any necessary libraries or headers
#include "hwconfig.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Declare your class and its member functions
TaskHandle_t NavigationTaskHandle;
QueueHandle_t InstructionQueue;

void WorkerNavigation(void *pvParameters);

void transferToSTM();
void transmitDataToSTM();
void printMEMSDataText(MEMS_Data* data);
void transmitMEMSData(MEMS_Data* data);
void transmitPositionData(LineBotToSTM32Message* message);
void printInstructions(navigationInstruction* instructions, uint16_t numInstructions);
void printOLEDInstructions(navigationInstruction* instructions, uint16_t numInstructions);
void stmInterruptPinInit(void);
void stmInterruptSet(void);
void stmInterruptClear(void);
void stmInterruptToggle(void);

// Function to print the instructions
void printInstructions(navigationInstruction* instructions, uint16_t numInstructions);

#endif // NAVIGATIONTASK_H
