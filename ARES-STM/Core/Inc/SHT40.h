/*
 * SHT40.h
 *
 *  Created on: Nov 7, 2023
 *      Author: karlvoigt
 */

#ifndef SRC_SHT40_H_
#define SRC_SHT40_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"  // Include the FreeRTOS header appropriate for your project
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define SHT40_I2C_ADDRESS (0x44 << 1) // The shifted 7-bit I2C address for the SHT40 sensor

/* Exported macro ------------------------------------------------------------*/
#define TEMPERATURE_SCALE_FACTOR 65535
#define RAW_VALUE_MAX 65535

/* Exported functions ------------------------------------------------------- */
void SHT40_SoftReset();
void SHT40_Init();
void SHT40_Read_Temp_Hum(uint16_t *temperature, uint16_t *humidity);

#ifdef __cplusplus
}
#endif


#endif /* SRC_SHT40_H_ */
