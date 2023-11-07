/*
 * SHT40.c
 *
 *  Created on: Nov 7, 2023
 *      Author: karlvoigt
 */


#include "SHT40.h"

// Commands
#define SHT40_CMD_MEASURE_HIGH_PRECISION 0xFD  // Command for high precision measurement

// Delay for measurement (maximum time in datasheet for high precision is 10ms)
#define SHT40_MEASURE_DELAY_MS 10
// Delay for medium precision measurement (maximum time in datasheet is typically less than high precision)
#define SHT40_MEDIUM_PRECISION_DELAY_MS 5

#define SHT40_CMD_SOFT_RESET 0x94  // Command for soft reset
#define SHT40_CMD_MEASURE_MEDIUM_PRECISION 0xF6  // Command for medium precision measurement


extern I2C_HandleTypeDef hi2c1;  // Assuming you are using hi2c1

// Soft reset the SHT40 sensor
void SHT40_SoftReset() {
    uint8_t command = SHT40_CMD_SOFT_RESET;
    HAL_I2C_Master_Transmit(&hi2c1, SHT40_I2C_ADDRESS, &command, 1, 100);
    HAL_Delay(5); // Delay to ensure soft reset command is processed
}


// Initializes the SHT40 sensor
void SHT40_Init() {
	// Perform a soft reset to ensure the sensor is in a known state
	SHT40_SoftReset();
}

// Reads temperature and humidity from the SHT40 sensor
void SHT40_Read_Temp_Hum(uint16_t *temperature, uint16_t *humidity) {
    uint8_t data[6];  // Data buffer for temperature and humidity
    uint8_t command = SHT40_CMD_MEASURE_MEDIUM_PRECISION;

    // Send the command to measure temperature and humidity with highest precision
    HAL_I2C_Master_Transmit(&hi2c1, SHT40_I2C_ADDRESS, &command, 1, 100);

    // Delay to ensure the measurement is complete
    osDelay(pdMS_TO_TICKS(SHT40_MEDIUM_PRECISION_DELAY_MS));

    // Read 6 bytes of data (2 bytes temp, 1 byte CRC, 2 bytes humidity, 1 byte CRC)
    HAL_I2C_Master_Receive(&hi2c1, SHT40_I2C_ADDRESS, data, 6, 100);

    // Convert the raw values to temperature and humidity
    uint16_t rawTemp = ((uint16_t)data[0] << 8) | data[1];
    uint16_t rawHumidity = ((uint16_t)data[3] << 8) | data[4];

    // Calculate the temperature and humidity values according to the datasheet
    *temperature = rawTemp;  // Convert to degrees Celsius
    *humidity = rawHumidity;          // Convert to %RH
}
