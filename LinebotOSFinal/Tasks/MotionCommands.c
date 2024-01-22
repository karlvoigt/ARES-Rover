#include "MotionCommands.h"

#include "DriverMotor.h"
#include "LineFollowerSpeedTask.h"
#include "MotorSpeedTask.h"
#include "MotorPosTask.h"
#include "ADCTask.h"
#include "GyroTask.h"

#include <math.h>

#define MM_TO_CNT(x) (x/WHEEL_CIRC*360/DEG_PER_CNT)
#define CNT_TO_MM(cnt) (cnt*WHEEL_CIRC*DEG_PER_CNT/360)
#define DEG_TO_RAD(x) (x*M_PI/180)
#define POSITIONING_FREQUENCY 50
#define POSITIONING_PERIOD_MS (1000/POSITIONING_FREQUENCY)
float xCoord = 0;
float yCoord = 0;
float xCoordOld = 0;
float yCoordOld = 0;
float encoderAverage, distance,totalDistance, yaw, yawRate;
EncoderStruct Encoder,EncoderOld;

void encoderPositioning();
void resetEncoderPosition();

void doDriveStraight(float Distance, float Speed)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(POSITIONING_PERIOD_MS);
	ADCStruct ADCData;
	int Cnt;
	EncoderStruct EncoderTarget;

		
	float driveTime = Distance / Speed;
		
	xLastWakeTime = xTaskGetTickCount();
	EnableMotorPosTask();
	EnableMotorSpeedTask();
	Encoder=DriverMotorGetEncoder();
	totalDistance = 0;
	if (Distance>0)
	{
		//Fwd
		EncoderTarget.Cnt1=Encoder.Cnt1+MM_TO_CNT(Distance);
		EncoderOld.Cnt1 = Encoder.Cnt1;
		EncoderOld.Cnt2 = Encoder.Cnt2;
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
			// Calculate the distance traveled by each wheel
			if (Encoder.Cnt1>EncoderTarget.Cnt1) {
				DisableMotorPosTask();
				DisableMotorSpeedTask();
				encoderPositioning();
				//Final distance
				totalDistance = CNT_TO_MM((Encoder.Cnt1+Encoder.Cnt2)/2);
				resetEncoderPosition();
				vTaskDelay(250); //Stand still for about half a second to give the robot time to come to a full stop
				break;
			} else {
				encoderPositioning();
			}
		}
		else
		{//Reverse
			if (Encoder.Cnt1<EncoderTarget.Cnt1) break;
		}
				
		vTaskDelayUntil( &xLastWakeTime, POSITIONING_PERIOD_MS );
	}	
	
}

void encoderPositioning() {
	GyroGet(&yawRate, &yaw);
	// Calculate the distance traveled by each wheel
	int16_t encoderLeft = Encoder.Cnt1;
	int16_t encoderRight = Encoder.Cnt2;
	//Average the two distances
	encoderAverage = (encoderLeft - EncoderOld.Cnt1 + encoderRight - EncoderOld.Cnt2) / 2.0;
	distance = CNT_TO_MM(encoderAverage);
	totalDistance += distance;
	//Use distance and yaw rate to calcute new coordinates
	xCoordOld = xCoord;
	yCoordOld = yCoord;
	xCoord += distance * sinf(DEG_TO_RAD(yaw));
	yCoord += distance * cosf(DEG_TO_RAD(yaw));
	//printf("Encoder.Cnt1 = %d, Encoder.Cnt2 = %d\n", encoderLeft, encoderRight);
	//printf("EncoderOld.Cnt1 = %d, EncoderOld.Cnt2 = %d\n", EncoderOld.Cnt1, EncoderOld.Cnt2);
	//printf("xCoord = %f, yCoord = %f\n", xCoord, yCoord);
	//printf("distance = %f\n", distance);
	//printf("totalDistance = %f\n", totalDistance);
	//printf("yaw = %f\n", yaw);
	EncoderOld.Cnt1 = encoderLeft;
	EncoderOld.Cnt2 = encoderRight;
}

void doDriveSegment(float Speed)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = 10;
	EncoderStruct EncoderTarget;
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
	const TickType_t xPeriod = 10;
	float encoderAngle = 0.0f; 
    float filteredAngle = 0.0f;
    const float gyroWeight = 0.0f;
    const float encoderWeight = 1-gyroWeight;
	float avgEncoderChange;
    float initialYaw, yaw, gyroAngle; // Store the initial yaw
	EncoderStruct initialEncoder, Encoder;
	const float angleMargin= 1;	


	xLastWakeTime = xTaskGetTickCount();
	EnableMotorPosTask();
	EnableMotorSpeedTask();
	initialEncoder=DriverMotorGetEncoder();
	GyroGet(NULL, &initialYaw);

	if (Angle > 0) { // Clockwise rotation
        MotorSpeedSet(MM_TO_CNT(Speed),0);
    } else { // Counter-clockwise rotation
        MotorSpeedSet(0,MM_TO_CNT(Speed));
    }

	while (1) {
		// Get the encoder angle
		Encoder = DriverMotorGetEncoder();
		// Get the gyro angle
		GyroGet(NULL, &yaw);
		
		//Use the right encoder depending on the direction of rotation
		if (Angle > 0) { // Clockwise rotation
			avgEncoderChange = Encoder.Cnt1 - initialEncoder.Cnt1;
		} else { // Counter-clockwise rotation
			avgEncoderChange = Encoder.Cnt2 - initialEncoder.Cnt2;
		}
		// avgEncoderChange = (fabs(Encoder.Cnt1 - initialEncoder.Cnt1) + fabs(Encoder.Cnt2 - initialEncoder.Cnt2)) / 2.0;
		// Calculate the encoder angle (based on only one wheel spinning)
		encoderAngle = (CNT_TO_MM(avgEncoderChange) * 360.0f) /  (2 * WHEEL_DISTANCE * M_PI);
		// Calculate the gyro angle (fthrowing away the sign)
		gyroAngle = fabs(yaw - initialYaw);
		// Calculate the filtered angle
		filteredAngle = gyroWeight * gyroAngle + encoderWeight * encoderAngle;
		// printf("Encoder.Cnt1 = %d, Encoder.Cnt2 = %d\n", Encoder.Cnt1, Encoder.Cnt2);
		// printf("avgEncoderChange = %f\n", avgEncoderChange);
		// printf("encoderAngle = %f\n", encoderAngle);
		// printf("gyroAngle = %f\n", gyroAngle);
		// Check if the filtered angle is within the margin
		if (fabs(filteredAngle) > fabs(Angle)-angleMargin) {
			break;
		}
		// Delay for the next cycle
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
	if (Angle>0) {
		xCoord += WHEEL_DISTANCE/2 * cosf(DEG_TO_RAD(filteredAngle));
	} else {
		xCoord -= WHEEL_DISTANCE/2 * cosf(DEG_TO_RAD(filteredAngle));
	}
	yCoord += WHEEL_DISTANCE/2 * sinf(DEG_TO_RAD(filteredAngle));
	// TickType_t xLastWakeTime;
	// const TickType_t xPeriod = 10;
    // TickType_t timeLeft;
	// EncoderStruct EncoderTarget;
	// float GyroTarget, error;
	// float yaw, yawRate;
	// ADCStruct ADCData;
	// int Cnt, correcting;
	// const float yawMargin= 1;						//Counter for how many times the yaw is within the margin
	// const int correctionSpeed = 25;	
	// float fa;




	// xLastWakeTime = xTaskGetTickCount();
	// EnableMotorPosTask();
	// EnableMotorSpeedTask();
	// Encoder=DriverMotorGetEncoder();
	// fa=WHEEL_DISTANCE*M_PI/360*(-Angle); //Calculate wheel distance to travel
	// GyroGet(&yawRate, &yaw);
	// GyroTarget = fmod(yaw + Angle, 360);; //Calculate target yaw and ensure it is between 0 and 360

	// printf("yaw = %f\n", yaw);
	// printf("GyroTarget = %f\n", GyroTarget);
	// // if (fa>0)
	// // {
	// // 	//Fwd
	// // 	EncoderTarget.Cnt1=(float) Encoder.Cnt1-MM_TO_CNT(fa);
	// // 	MotorSpeedSet(-MM_TO_CNT(Speed),MM_TO_CNT(Speed));
	// // }
	// // else
	// // {
	// // 	//Reverse
	// // 	EncoderTarget.Cnt1=(float) Encoder.Cnt1-MM_TO_CNT(fa);
	// // 	MotorSpeedSet(MM_TO_CNT(Speed),-MM_TO_CNT(Speed));
	// // }
	// if (Angle > 0) { // Clockwise rotation
    //     MotorSpeedSet(MM_TO_CNT(Speed),-MM_TO_CNT(Speed));
    // } else { // Counter-clockwise rotation
    //     MotorSpeedSet(-MM_TO_CNT(Speed),MM_TO_CNT(Speed));
    // }
	// while (1)
	// {
	// 	Encoder=DriverMotorGetEncoder();
	// 	timeLeft =pdMS_TO_TICKS(fabs((EncoderTarget.Cnt1 - Encoder.Cnt1)/MM_TO_CNT(Speed)));
	// 	GyroGet(&yawRate, &yaw);
	// 	error = fmod(GyroTarget- yaw, 360);
    //     // if (error > 180) error -= 360; // Adjust for the shortest direction
	// 	//Check if yaw is within margin
	// 	// printf("error = %f\n", error);
	// 	// printf("yaw = %f\n", yaw);

	// 	if (fabs(error) < yawMargin) {
	// 		break;
	// 	} else {
	// 		if (Angle>0) { //Clockwise
	// 			if (error < 0) {
	// 				MotorSpeedSet(-MM_TO_CNT(Speed/2),MM_TO_CNT(Speed/2)); //Turn in the other direction slowly
	// 				// Angle = -Angle; //Update Angle
	// 			}
	// 		} else { //Counter Clockwise
	// 			if (error > 0) {
	// 				MotorSpeedSet(MM_TO_CNT(Speed/2),-MM_TO_CNT(Speed/2)); //Turn in the other direction slowly
	// 			}
	// 		}
	// 	}
		
	// 	//printf("timeLeft = %d\n", timeLeft);
	// 	//Stop if target distance is reached
	// 	// if (fa>0)
	// 	// {//Fwd
	// 	// 	// if (Encoder.Cnt1<EncoderTarget.Cnt1 && yawDiff < 2*yawMargin) break;
	// 	// 	//Check if yaw is within margin. If it is over target more than the margin, reverse motor speed and slowly correct
	// 	// 	if (yawDiff > yawMargin) {
	// 	// 		if (yaw > GyroTarget && !correcting) {
	// 	// 			correcting = 1;
	// 	// 			MotorSpeedSet(MM_TO_CNT(correctionSpeed),-MM_TO_CNT(correctionSpeed));
	// 	// 		}
	// 	// 	} else {
	// 	// 		break;
	// 	// 	}
	// 	// }
	// 	// else
	// 	// {//Reverse
	// 	// 	// if (Encoder.Cnt1>EncoderTarget.Cnt1 && yawDiff < 2*yawMargin) break;
	// 	// 	//Check if yaw is within margin. If it is over target more than the margin, reverse motor speed and slowly correct
	// 	// 	if (yawDiff > yawMargin) {
	// 	// 		if (yaw < GyroTarget && !correcting) {
	// 	// 			correcting = 1;
	// 	// 			MotorSpeedSet(-MM_TO_CNT(correctionSpeed),MM_TO_CNT(correctionSpeed));
	// 	// 		}
	// 	// 	} else {
	// 	// 		break;
	// 	// 	}
	// 	// }

    //     //When further away from the target, use a longer period to save CPU time
    //     //When the target distance is close, use a shorter period to improve accuracy
	// 	//if (timeLeft < xPeriod*20) {
    //         //vTaskDelayUntil( &xLastWakeTime, xPeriod);
    //     //} else {
    //         //vTaskDelayUntil( &xLastWakeTime, timeLeft/2 );
	// 		vTaskDelayUntil(&xLastWakeTime, xPeriod);
    //     //}
	// }
	DisableMotorPosTask();
	DisableMotorSpeedTask();
	resetEncoderPosition();
	vTaskDelay(500);
}

//Reset Encoder Position
void resetEncoderPosition()
{
	DriverMotorResetEncoder();
    EncoderOld.Cnt1 = 0;
    EncoderOld.Cnt2 = 0;
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