#ifndef MOTIONCOMMANDS_H
#define MOTIONCOMMANDS_H

#include "hwconfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdio.h>

#define CMD_DRIVE_STRAIGHT 1
#define CMD_DRIVE_SEGMENT 2
#define CMD_ROTATE_CENTER 3


typedef struct 
{
    uint8_t Cmd;
    float Params[2];
} MotionCmdStruct;
void doDriveStraight(float Distance, float Speed);
void doDriveSegment(float Speed);
void doRotateCenter(float Angle, float Speed);
void resetEncoderPositioning(void);
float getEncoderXCoord(void);
float getEncoderYCoord(void);
float getEncoderYaw(void);

#endif // MOTIONCOMMANDS_H
