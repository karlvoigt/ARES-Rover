#ifndef ENCODER_POSITIONING_H
#define ENCODER_POSITIONING_H

#include "FreeRTOS.h" // Include this if you're using FreeRTOS types like TaskHandle_t
#include "task.h" // Include this if you're using FreeRTOS functions like xTaskGetTickCount

// Task handle
extern TaskHandle_t EncoderPositioningTaskHandle;

// Task priority
#define TASK_PRIORITY  configMAX_PRIORITIES - 1

// Task stack size
#define TASK_STACK_SIZE configMINIMAL_STACK_SIZE

// Task name
#define TASK_NAME "EncoderPositioning"

// Task function
void WorkerEncoderPositioning(void *pvParameters);

void InitEncoderPositioningTask(void);

// Function to get the X coordinate
float getEncoderXCoord(void);

// Function to get the Y coordinate
float getEncoderYCoord(void);

//Function to get the yaw
float getEncoderYaw(void);

#endif // ENCODER_POSITIONING_H