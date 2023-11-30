#ifndef ARESTASK_H
#define ARESTASK_H

#include "hwconfig.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "DriverOled.h"

#include "CustomProtocol.h"
#include "MotionCommands.h"

//Variables
QueueHandle_t InstructionQueue;
TaskHandle_t AresTaskHandle;
TaskHandle_t NavigationTaskHandle;

// Function declarations
void WorkerAres(void *pvParameters);
void WorkerNavigation(void *pvParameters);
void InitAresTask(void);
// Function to receive and decode instructions
uint8_t receiveInstructions(uint8_t* data, uint16_t dataLength);
void transferToSTM();
void transmitDataToSTM();
void printMEMSDataText(MEMS_Data* data);
void transmitMEMSData(MEMS_Data* data);
void printInstructions(navigationInstruction* instructions, uint16_t numInstructions);
void printOLEDInstructions(navigationInstruction* instructions, uint16_t numInstructions);
void stmInterruptPinInit(void);
void stmInterruptSet(void);
void stmInterruptClear(void);
void stmInterruptToggle(void);

// Function to print the instructions
void printInstructions(navigationInstruction* instructions, uint16_t numInstructions);

#endif // ARESTASK_H
