#ifndef ACC_TASK_H
#define ACC_TASK_H

/**
 * Linebot accelerometer task. Using the MPU6050 reads X Y Z axis accelerometer data, calculates speed m/s and integrates to distance.
 * \file GyroTask.h
 * \brief Initialize gyroscope data acquisition task
*/


/**
 * \brief Initialize accelerometer data acquisition task
*/

void InitAccTask(void);

/**
 * \brief Fetch last accelerometer data
 * \param VelX: accelerometer velocity
 * \param DistX: accelerometer distance
*/
void AccGet(float *Ax, float *VelX, float *DistX);

#endif