#include "GyroTask.h"

#include "DriverMPU6050.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "trace.h"
#include <math.h>

#define RATE_CST (1.0/131.0)
#define THRESHOLD 0.001
#define MAX_ITERATIONS 1000

static SemaphoreHandle_t GyroSema;
static float GlobYawRate=0.0,GlobYaw=0.0;

//Private function prototypes
static void WorkerGyro(void *pvParameters);

//Function definitions
void InitGyroTask()
{
	xTaskCreate( WorkerGyro, "gyro", 256, NULL, tskIDLE_PRIORITY+3, NULL );	
	GyroSema=xSemaphoreCreateMutex();
}

static void WorkerGyro(void *pvParameters)
{
	int16_t Gz;
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = 10;
	float GyroYawOffset;
	float OldYawRate=0;
	
	DriverMPU6050Init();	//Initialize IMU
	xLastWakeTime=xTaskGetTickCount();
	
	vTaskDelay(200);
	//Calibrate gyro offset
	float sum = 0;
	int count = 0;
	float error = 0;
	float average = 0;

	for(int i = 0; i < MAX_ITERATIONS; i++) {
		DriverMPU6050GyroGet(NULL,NULL,&Gz);
		sum += Gz;
		count++;
		average = sum / count;
		error = fabs(Gz - average);

		if(error < THRESHOLD) {
			break;
		}
		vTaskDelay(5);
	}

	GyroYawOffset = average * RATE_CST;

	
	while (1)
	{		
		DriverMPU6050GyroGet(NULL,NULL,&Gz);
		xSemaphoreTake(GyroSema,portMAX_DELAY);
		GlobYawRate=(float) Gz*RATE_CST;
		GlobYawRate-=GyroYawOffset;
		
		//Integration to yaw angle
		GlobYaw+=((GlobYawRate/100)+(OldYawRate/100))/2;
		if (GlobYaw>360.0) GlobYaw-=360.0;
		if (GlobYaw<0.0) GlobYaw+=360.0;
		
		OldYawRate=GlobYawRate;
		
		xSemaphoreGive(GyroSema);
		if (GlobYawRate>0) 
			GyroYawOffset+=0.001;
		else
			GyroYawOffset-=0.001;
		
		vTaskDelayUntil( &xLastWakeTime, xPeriod );
	}

}

void GyroGet(float *YawRate, float *Yaw)
{
		xSemaphoreTake(GyroSema,portMAX_DELAY);
		if (YawRate!=NULL) *YawRate=GlobYawRate;
		if (Yaw!=NULL) *Yaw=GlobYaw;
		xSemaphoreGive(GyroSema);
	
}
