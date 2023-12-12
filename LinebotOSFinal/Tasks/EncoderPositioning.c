#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "DriverMotor.h"
#include "EncoderPositioning.h"
#include "GyroTask.h"
#include "hwconfig.h"
#include <math.h>
#include "Globals.h"

//Variables
float xCoord = 0;
float yCoord = 0;
float xCoordOld = 0;
float yCoordOld = 0;
float yaw = 0;
float yawRate = 0;

EncoderStruct Encoder;
EncoderStruct EncoderOld = {0, 0};

#define POSITIONING_FREQUENCY 100
#define POSITIONING_PERIOD_MS (1000/POSITIONING_FREQUENCY)
#define MM_TO_CNT(x) (x/WHEEL_CIRC*360/DEG_PER_CNT)
#define CNT_TO_MM(cnt) (cnt*WHEEL_CIRC*DEG_PER_CNT/360)
#define DEG_TO_RAD(x) (x*M_PI/180)



// Task function
void WorkerEncoderPositioning(void *pvParameters)
{
    TickType_t xLastWakeTime;

    while (1)
    {
        if (inMotion) {
            // Save last wakeup time
            xLastWakeTime = xTaskGetTickCount();

            //Get encoder positions and store in EncoderStruct variable
            Encoder = DriverMotorGetEncoder();

            GyroGet(&yawRate, &yaw);

            // Calculate the distance traveled by each wheel
            int16_t encoderLeft = Encoder.Cnt1 - EncoderOld.Cnt1;
            int16_t encoderRight = Encoder.Cnt2 - EncoderOld.Cnt2;
            //Average the two distances
            int16_t encoderAverage = (encoderLeft + encoderRight) / 2;
            float distance = CNT_TO_MM(encoderAverage);

            //Use distance and yaw rate to calcute new coordinates
            xCoordOld = xCoord;
            yCoordOld = yCoord;
            xCoord += distance * cosf(DEG_TO_RAD(yaw));
            yCoord += distance * sinf(DEG_TO_RAD(yaw)); 

            vTaskDelayUntil(&xLastWakeTime, POSITIONING_PERIOD_MS);
        } else {
            vTaskSuspend(NULL);
        }
    }
}

void ResumeEncoderPositioning()
{
    //Check if task is suspended
    if (eTaskGetState(EncoderPositioningTaskHandle) == eSuspended) 
        vTaskResume(EncoderPositioningTaskHandle);
}

//Task init function
void InitEncoderPositioningTask(void)
{
	inMotion=0;
    // Create the encoder positioning task
    xTaskCreate(WorkerEncoderPositioning, TASK_NAME, TASK_STACK_SIZE, NULL, TASK_PRIORITY, &EncoderPositioningTaskHandle);
}

// Function to get the X coordinate
float getEncoderXCoord(void)
{
    // Return the X coordinate
    return xCoord;
}

// Function to get the Y coordinate
float getEncoderYCoord(void)
{
    // Return the Y coordinate
    return yCoord;
}

//Function to get the yaw
float getEncoderYaw(void)
{
    // Return the yaw
    return yaw;
}

//Reset Encoder Position
void resetEncoderPosition()
{
    EncoderOld.Cnt1 = 0;
    EncoderOld.Cnt2 = 0;
}

