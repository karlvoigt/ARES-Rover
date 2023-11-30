/**
 * MPU6050 IMU I�C driver. Currently only supports gyroscope function
 * \file DriverMPU6050.h
 * \brief MPU6050 I�C driver
*/


#ifndef DRIVER_MPU6050_H
#define DRIVER_MPU6050_H

#include "DriverTWIMaster.h"
#include <stdint.h>
#include "CustomProtocol.h"


/**
 * \brief Initialize MPU6050 driver. DriverTWIMaster should be initialized beforehand.
*/
void DriverMPU6050Init(void);

/**
 * \brief Requests gyroscope data, blocks until data is returned.
 * \param Gx: X gyroscope data. Pass NULL if not used. (1/131)�/s per unit
 * \param Gy: Y gyroscope data. Pass NULL if not used. (1/131)�/s per unit
 * \param Gz: Z gyroscope data. Pass NULL if not used. (1/131)�/s per unit
*/
void DriverMPU6050GyroGet(int16_t *Gx,int16_t *Gy,int16_t *Gz);
void DriverMPU6050AccGet(int16_t *Ax,int16_t *Ay,int16_t *Az);
void DriverMPU6050MemsGet(MEMS_Data* data);
#endif