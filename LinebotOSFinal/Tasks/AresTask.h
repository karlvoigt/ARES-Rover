#ifndef ARESTASK_H
#define ARESTASK_H

#include "hwconfig.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//Variables
TaskHandle_t AresTaskHandle;
TaskHandle_t USARTCheckTaskHandle;

// Function declarations
void WorkerAres(void *pvParameters);
void WorkerUSARTCheck(void *pvParameters);
void InitAresTask(void);

#endif // ARESTASK_H
