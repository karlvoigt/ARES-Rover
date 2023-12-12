#include "MotionCommands.h"

#include "DriverMotor.h"
#include "LineFollowerSpeedTask.h"
#include "MotorSpeedTask.h"
#include "MotorPosTask.h"
#include "ADCTask.h"

void doDriveStraight(float Distance, float Speed)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = 10;
	EncoderStruct Encoder,EncoderTarget;
	ADCStruct ADCData;
	int Cnt;

		
	float driveTime = Distance / Speed;
		
	xLastWakeTime = xTaskGetTickCount();
	EnableMotorPosTask();
	EnableMotorSpeedTask();
	Encoder=DriverMotorGetEncoder();
	if (Distance>0)
	{
		//Fwd
		EncoderTarget.Cnt1=Encoder.Cnt1+MM_TO_CNT(Distance);
				
		MotorSpeedSet(MM_TO_CNT(Speed),MM_TO_CNT(Speed));
	}
	else
	{
		//Reverse
		EncoderTarget.Cnt1=Encoder.Cnt1+MM_TO_CNT(Distance);
		MotorSpeedSet(-MM_TO_CNT(Speed),-MM_TO_CNT(Speed));
	}
	while (1)
	{
		Encoder=DriverMotorGetEncoder();
				
		//Stop if target distance is reached
		if (Distance>0)
		{//Fwd
			if (Encoder.Cnt1>EncoderTarget.Cnt1) break;
		}
		else
		{//Reverse
			if (Encoder.Cnt1<EncoderTarget.Cnt1) break;
		}
				
		vTaskDelayUntil( &xLastWakeTime, xPeriod );
	}
	DisableMotorPosTask();
	DisableMotorSpeedTask();	
	
}

void doDriveSegment(float Speed)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = 10;
	EncoderStruct Encoder,EncoderTarget;
	ADCStruct ADCData;
	int Cnt;
			
	float fa;
			
	xLastWakeTime = xTaskGetTickCount();
	EnableMotorPosTask();
	EnableMotorSpeedTask();
	Encoder=DriverMotorGetEncoder();
				
	StartLineFollower(MM_TO_CNT(Speed));

	Cnt=0;
	while (1)
	{
		ADCData=GetADCData();
		if ((ADCData.PhotoL+ADCData.PhotoR)/2>LINEFOLLOW_THRESHOLD)
		{
			Cnt++;
			if (Cnt>LINEFOLLOW_THRESHOLD_CNT) break;
		}
		else
		Cnt=0;
					
		vTaskDelayUntil( &xLastWakeTime, xPeriod );
	}
	StopLineFollower();
	DisableMotorPosTask();
	DisableMotorSpeedTask();
}

void doRotateCenter(float Angle, float Speed)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = 1;
    TickType_t timeLeft;
	EncoderStruct Encoder,EncoderTarget;
	ADCStruct ADCData;
	int Cnt;
		
	float fa;
	
	xLastWakeTime = xTaskGetTickCount();
	EnableMotorPosTask();
	EnableMotorSpeedTask();
	Encoder=DriverMotorGetEncoder();
	fa=WHEEL_DISTANCE*3.14/360*Angle; //Calculate wheel distance to travel
	if (fa>0)
	{
		//Fwd
		EncoderTarget.Cnt1=(float) Encoder.Cnt1-MM_TO_CNT(fa);
		MotorSpeedSet(-MM_TO_CNT(Speed),MM_TO_CNT(Speed));
	}
	else
	{
		//Reverse
		EncoderTarget.Cnt1=(float) Encoder.Cnt1-MM_TO_CNT(fa);
		MotorSpeedSet(MM_TO_CNT(Speed),-MM_TO_CNT(Speed));
	}
	while (1)
	{
		Encoder=DriverMotorGetEncoder();
		timeLeft =(EncoderTarget.Cnt1 - Encoder.Cnt1)/MM_TO_CNT(Speed);
		//Stop if target distance is reached
		if (fa>0)
		{//Fwd
			if (Encoder.Cnt1<EncoderTarget.Cnt1) break;
		}
		else
		{//Reverse
			if (Encoder.Cnt1>EncoderTarget.Cnt1) break;
		}

        //When further away from the target, use a longer period to save CPU time
        //When the target distance is close, use a shorter period to improve accuracy
		if (timeLeft < xPeriod*20) {
            vTaskDelayUntil( &xLastWakeTime, xPeriod);
        } else {
            vTaskDelayUntil( &xLastWakeTime, timeLeft/2 );
        }
	}
	DisableMotorPosTask();
	DisableMotorSpeedTask();
	
}

