#ifndef RETURN_HOME_TASK_H
#define RETURN_HOME_TASK_H

#include <stdint.h>
#include "hwconfig.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Task handle variable
extern TaskHandle_t ReturnHomeTaskHandle;

// Function declarations
void InitReturnHomeTask(void);
void WorkerReturnHome(void *pvParameters);

#endif // RETURN_HOME_TASK_H
