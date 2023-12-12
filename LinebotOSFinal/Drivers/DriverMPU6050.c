#include "FreeRTOS.h"
#include "hwconfig.h"
#include "DriverMPU6050.h"
#include "CustomProtocol.h"
//#include "MPU6050_HAL.h"

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

#define MPU6050_ADDR 0b1101000
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_GX 0x43
#define MPU6050_GY 0x45
#define MPU6050_GZ 0x47
#define MPU6050_AX 0x3B
#define MPU6050_AY 0x3D
#define MPU6050_AZ 0x3F
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACC_CONFIG 0x1C
#define MPU6050_SMPRT_DIV 0x19
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_PWR_MGMT_1 0x6B


void DriverMPU6050Init(void)
{
	uint8_t res;
	uint8_t Buffer[2];
	int a;

	//MPU6050_setAddress(MPU6050_ADDR);
	
	Buffer[0]=MPU6050_WHO_AM_I;
	res=TWIMWriteRead(MPU6050_ADDR,Buffer,1,Buffer,1);
	if (!res)
	{
		printf ("MPU6050: comms fail\r\n");
		return;
	}
	if (Buffer[0]!=MPU6050_ADDR)
	{
		printf ("MPU6050 WHO_AM_I readback fail: %x read, %x expected\r\n",Buffer[0],MPU6050_ADDR);
		return;
	}
	////test if mpu6050.c also working with test connection
	//if (MPU6050_testConnection())
	//{
		//printf ("MPU6050: test connection success\r\n");
		//return;
	//}

	//Setup sample rate
	Buffer[0]=MPU6050_SMPRT_DIV;
	Buffer[1]=0;	//Output rate/1 --> 1kHz
	res=TWIMWrite(MPU6050_ADDR,Buffer,2);
	
	//Setup CONFIG
	Buffer[0]=MPU6050_CONFIG;
	Buffer[1]=1;	//Fs=1kHz, 188Hz BW
	res=TWIMWrite(MPU6050_ADDR,Buffer,2);	
	
	//Setup GYRO_CONFIG
	Buffer[0]=MPU6050_GYRO_CONFIG;
	Buffer[1]=0;	//250�/s max rate
	res=TWIMWrite(MPU6050_ADDR,Buffer,2);	

	//Setup INT_PIN_CFG
	Buffer[0]=MPU6050_INT_PIN_CFG;
	Buffer[1]=1<<4;	//Clear INT on any read
	res=TWIMWrite(MPU6050_ADDR,Buffer,2);

	//Setup INT_ENABLE
	Buffer[0]=MPU6050_INT_ENABLE;
	Buffer[1]=1;	//Set INT on data ready
	res=TWIMWrite(MPU6050_ADDR,Buffer,2);
	
	//Setup PWR_MGMT1
	Buffer[0]=MPU6050_PWR_MGMT_1;
	Buffer[1]=1;	//Gyro X as clock
	res=TWIMWrite(MPU6050_ADDR,Buffer,2);

}

void DriverMPU6050MemsGet(MEMS_Data* data) {
	int16_t gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

	uint8_t res;
	uint8_t Buffer[18];

	Buffer[0]=MPU6050_AX;
	res=TWIMWriteRead(MPU6050_ADDR,Buffer,1,Buffer,18);

	configASSERT(res);
	
	((char *) (&accelX))[0]=Buffer[1];
	((char *) (&accelX))[1]=Buffer[0];
	
	((char *) (&accelY))[0]=Buffer[3];
	((char *) (&accelY))[1]=Buffer[2];
		
	((char *) (&accelZ))[0]=Buffer[5];
	((char *) (&accelZ))[1]=Buffer[4];

	((char *) (&gyroX))[0]=Buffer[12];
	((char *) (&gyroX))[1]=Buffer[11];
	
	((char *) (&gyroY))[0]=Buffer[14];
	((char *) (&gyroY))[1]=Buffer[13];
		
	((char *) (&gyroZ))[0]=Buffer[16];
	((char *) (&gyroZ))[1]=Buffer[15];

	if (data!=NULL)	{
		data->gyroX = gyroX;
		data->gyroY = gyroY;
		data->gyroZ = gyroZ;
		data->accelX = accelX;
		data->accelY = accelY;
		data->accelZ = accelZ;
	}
}

void DriverMPU6050GyroGet(int16_t *Gx,int16_t *Gy,int16_t *Gz)
{
	uint8_t res;
	uint16_t x,y,z;
	uint8_t Buffer[6];

	//Read Gyro data
	Buffer[0]=MPU6050_GX;
	res=TWIMWriteRead(MPU6050_ADDR,Buffer,1,Buffer,6);
	configASSERT(res);
	((char *) (&x))[0]=Buffer[1];
	((char *) (&x))[1]=Buffer[0];
	
	((char *) (&y))[0]=Buffer[3];
	((char *) (&y))[1]=Buffer[2];
		
	((char *) (&z))[0]=Buffer[5];
	((char *) (&z))[1]=Buffer[4];

	if (Gx!=NULL) *Gx=x;	
	if (Gy!=NULL) *Gy=y;	
	if (Gz!=NULL) *Gz=z;	
}

//Function to get the accelerometer data
void DriverMPU6050AccGet(int16_t *Ax,int16_t *Ay,int16_t *Az)
{
	uint8_t res;
	uint16_t x,y,z;
	uint8_t Buffer[6];

	//Read Acc data
	Buffer[0]=MPU6050_AX;
	res=TWIMWriteRead(MPU6050_ADDR,Buffer,1,Buffer,6);
	configASSERT(res);
	((char *) (&x))[0]=Buffer[1];
	((char *) (&x))[1]=Buffer[0];
	
	((char *) (&y))[0]=Buffer[3];
	((char *) (&y))[1]=Buffer[2];
		
	((char *) (&z))[0]=Buffer[5];
	((char *) (&z))[1]=Buffer[4];

	if (Ax!=NULL) *Ax=x;	
	if (Ay!=NULL) *Ay=y;	
	if (Az!=NULL) *Az=z;	
}