#include "AccTask.h"

#include "DriverMPU6050.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "trace.h"

#define ACCEL_CST (1.0/16384.0) 
#define CALIBRATION_DURATION 15000 // 5 seconds in milliseconds
#define CALIBRATION_INTERVAL 10  // 10 milliseconds

static SemaphoreHandle_t AccSema;
static float GlobVelX=0.0, GlobDistX=0.0, GlobAx = 0.0;
static float AccOffsetX=0.0;

//Private function prototypes
static void WorkerAcc(void *pvParameters);

//Function definitions
void InitAccTask()
{
    xTaskCreate( WorkerAcc, "acc", 256, NULL, tskIDLE_PRIORITY+3, NULL );    
    AccSema=xSemaphoreCreateMutex();
}

static void WorkerAcc(void *pvParameters)
{
    int16_t Ax;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = 10;
    float OldVelX=0;
    int calibrationCount = 0;
    float totalAccX = 0.0;
    
    DriverMPU6050Init();    //Initialize IMU
    xLastWakeTime=xTaskGetTickCount();
    
    // Calibration period
    while (calibrationCount * CALIBRATION_INTERVAL < CALIBRATION_DURATION)
    {
        DriverMPU6050AccGet(&Ax,NULL,NULL);
        totalAccX += (float) Ax * ACCEL_CST;
        calibrationCount++;
        vTaskDelay(CALIBRATION_INTERVAL);
    }
    AccOffsetX = totalAccX / calibrationCount;
    
    while (1)
    {        
        DriverMPU6050AccGet(&Ax,NULL,NULL);
        xSemaphoreTake(AccSema,portMAX_DELAY);
        
        // Subtract the offset from the measured acceleration
        float calibratedAccX = ((float) Ax * ACCEL_CST) - AccOffsetX;
		if (calibratedAccX < 0.005 && calibratedAccX > -0.005) calibratedAccX=0.0;
        GlobAx = calibratedAccX;
        
        // Integration to get velocity
        GlobVelX += calibratedAccX * xPeriod / 1000.0;
        
        // Integration to get distance
        GlobDistX += ((GlobVelX + OldVelX) / 2.0) * xPeriod / 1000.0;
        
        OldVelX = GlobVelX;
        
        xSemaphoreGive(AccSema);
        
        vTaskDelayUntil( &xLastWakeTime, xPeriod );
    }

}

void AccGet(float *AccX, float *VelX, float *DistX)
{
    xSemaphoreTake(AccSema,portMAX_DELAY);
    if (AccX != NULL) *AccX = GlobAx;
    if (VelX != NULL) *VelX = GlobVelX;
    if (DistX != NULL) *DistX = GlobDistX;
    xSemaphoreGive(AccSema);
}