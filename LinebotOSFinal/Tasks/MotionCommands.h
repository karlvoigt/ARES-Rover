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

#define MM_TO_CNT(x) (x/WHEEL_CIRC*360/DEG_PER_CNT)

typedef struct 
{
    uint8_t Cmd;
    float Params[2];
} MotionCmdStruct;
void doDriveStraight(float Distance, float Speed);
void doDriveSegment(float Speed);
void doRotateCenter(float Angle, float Speed);

#endif // MOTIONCOMMANDS_H
