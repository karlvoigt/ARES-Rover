#include "IPS_Task.h"

#include "DriverMPU6050.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "trace.h"
#include <math.h>

#define ACCEL_CST (1.0/16384.0) 
#define GYRO_CST (1.0/131.0)
#define GRAVITY 9.81 // in m/s^2

static SemaphoreHandle_t IPS_Sema;
static float GlobPosX=0.0, GlobPosY=0.0, GlobPosZ=0.0;
static float GlobVelX=0.0, GlobVelY=0.0, GlobVelZ=0.0;
static float GlobYaw=0.0, GlobPitch=0.0, GlobRoll=0.0;
static float AccOffsetX=0.0, AccOffsetY=0.0, AccOffsetZ=0.0;
static float GlobAccX=0.0, GlobAccY=0.0, GlobAccZ=0.0;
static float GlobGyroX=0.0, GlobGyroY=0.0, GlobGyroZ=0.0;

//Private function prototypes
static void WorkerIPS(void *pvParameters);

void InitIPSTask()
{
    xTaskCreate( WorkerIPS, "IPS", 512, NULL, tskIDLE_PRIORITY+3, NULL );    
    IPS_Sema=xSemaphoreCreateMutex();
}

static void WorkerIPS(void *pvParameters)
{
    int16_t Ax, Ay, Az, Gx, Gy, Gz;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = 10;
    float OldVelX=0, OldVelY=0, OldVelZ=0;
    
    DriverMPU6050Init();    //Initialize IMU
    xLastWakeTime=xTaskGetTickCount();
    
    // Calibration and initialization code here...
    
    while (1)
    {        
        DriverMPU6050AccGet(&Ax, &Ay, &Az);
        DriverMPU6050GyroGet(&Gx, &Gy, &Gz);

        // Store the acceleration and gyroscope data in global variables
        GlobAccX = Ax * ACCEL_CST;
        GlobAccY = Ay * ACCEL_CST;
        GlobAccZ = Az * ACCEL_CST;
        
        GlobGyroX = Gx * GYRO_CST;
        GlobGyroY = Gy * GYRO_CST;
        GlobGyroZ = Gz * GYRO_CST;
        
        xSemaphoreTake(IPS_Sema,portMAX_DELAY);
        
        // Update orientation using gyroscope data
        GlobYaw += Gz * GYRO_CST * xPeriod / 1000.0;
        GlobPitch += Gy * GYRO_CST * xPeriod / 1000.0;
        GlobRoll += Gx * GYRO_CST * xPeriod / 1000.0;
        
        // Correct accelerometer data for gravity using orientation
        float gX = GRAVITY * sin(GlobPitch);
        float gY = GRAVITY * -sin(GlobRoll);
        float gZ = GRAVITY * cos(GlobPitch) * cos(GlobRoll);
        
        float accX = Ax * ACCEL_CST - gX;
        float accY = Ay * ACCEL_CST - gY;
        float accZ = Az * ACCEL_CST - gZ;
        
        // Integrate acceleration to get position
        GlobPosX += ((accX + OldVelX) / 2.0) * xPeriod / 1000.0;
        GlobPosY += ((accY + OldVelY) / 2.0) * xPeriod / 1000.0;
        GlobPosZ += ((accZ + OldVelZ) / 2.0) * xPeriod / 1000.0;
        
        OldVelX += accX * xPeriod / 1000.0;
        OldVelY += accY * xPeriod / 1000.0;
        OldVelZ += accZ * xPeriod / 1000.0;
        
        xSemaphoreGive(IPS_Sema);
        
        vTaskDelayUntil( &xLastWakeTime, xPeriod );
    }
}

void IPSGetPosition(float *PosX, float *PosY, float *PosZ)
{
    xSemaphoreTake(IPS_Sema,portMAX_DELAY);
    if (PosX != NULL) *PosX = GlobPosX;
    if (PosY != NULL) *PosY = GlobPosY;
    if (PosZ != NULL) *PosZ = GlobPosZ;
    xSemaphoreGive(IPS_Sema);
}

void IPSGetVelocity(float *VelX, float *VelY, float *VelZ)
{
	xSemaphoreTake(IPS_Sema,portMAX_DELAY);
	if (VelX != NULL) *VelX = GlobVelX;
	if (VelY != NULL) *VelY = GlobVelY;
	if (VelZ != NULL) *VelZ = GlobVelZ;
	xSemaphoreGive(IPS_Sema);
}

void IPSGetAcceleration(float *Ax, float *Ay, float *Az)
{
    xSemaphoreTake(IPS_Sema,portMAX_DELAY);
    if (Ax != NULL) *Ax = GlobAccX;
    if (Ay != NULL) *Ay = GlobAccY;
    if (Az != NULL) *Az = GlobAccZ;
    xSemaphoreGive(IPS_Sema);
}

void IPSGetGyroscope(float *Gx, float *Gy, float *Gz)
{
    xSemaphoreTake(IPS_Sema,portMAX_DELAY);
    if (Gx != NULL) *Gx = GlobGyroX;
    if (Gy != NULL) *Gy = GlobGyroY;
    if (Gz != NULL) *Gz = GlobGyroZ;
    xSemaphoreGive(IPS_Sema);
}