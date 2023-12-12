#ifndef RETURN_HOME_TASK_H
#define RETURN_HOME_TASK_H

#include <stdint.h>

// Task handle variable
extern TaskHandle_t ReturnHomeTaskHandle;

// Function declarations
void InitReturnHomeTask(void);
void WorkerReturnHome(void);

#endif // RETURN_HOME_TASK_H
