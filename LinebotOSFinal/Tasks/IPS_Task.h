#ifndef IPS_TASK_H
#define IPS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Initializes the Inertial Positioning System task
void InitIPSTask(void);

// Gets the current position in the X, Y, and Z directions
void IPSGetPosition(float *PosX, float *PosY, float *PosZ);

// Gets the current acceleration in the X, Y, and Z directions
void IPSGetAcceleration(float *Ax, float *Ay, float *Az);

// Gets the current gyroscope data in the X, Y, and Z directions
void IPSGetGyroscope(float *Gx, float *Gy, float *Gz)

#endif // IPS_TASK_H
