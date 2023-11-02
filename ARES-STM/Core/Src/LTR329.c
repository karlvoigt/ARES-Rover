#include "LTR329.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;  // Assuming you are using hi2c1

void LTR329_Init() {
    uint8_t data;

    // Activate the sensor
    data = LTR329_ACTIVE_MODE;
    HAL_I2C_Mem_Write(&hi2c1, LTR329_I2C_ADDRESS, LTR329_ALS_CONTR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // Set measurement rate
    data = LTR329_MEAS_RATE;
    HAL_I2C_Mem_Write(&hi2c1, LTR329_I2C_ADDRESS, LTR329_ALS_MEAS_RATE, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void LTR329_Read_Light(uint16_t *ch0, uint16_t *ch1) {
    uint8_t data[4];

    // Read 4 bytes of data starting from LTR329_ALS_DATA_CH1_0
    HAL_I2C_Mem_Read(&hi2c1, LTR329_I2C_ADDRESS, LTR329_ALS_DATA_CH1_0, I2C_MEMADD_SIZE_8BIT, data, 4, 100);

    // Combine bytes to get the light data for each channel
    *ch1 = (uint16_t)(data[1] << 8) | data[0];
    *ch0 = (uint16_t)(data[3] << 8) | data[2];
}
