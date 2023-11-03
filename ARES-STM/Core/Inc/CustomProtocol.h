/*
 * CustomProtocol.h
 *
 *  Created on: Nov 3, 2023
 *      Author: karlvoigt
 */

#ifndef INC_CUSTOMPROTOCOL_H_
#define INC_CUSTOMPROTOCOL_H_

#include <stdint.h>

#define START_DELIMITER 0xAA
#define END_DELIMITER   0x55
#define CUSTOM_MESSAGE_SIZE 10 // Expected size of CustomMessage

typedef enum {
    SENSOR_TYPE_TEMPERATURE = 0x01,
    SENSOR_TYPE_LIGHT       = 0x02
    // Add more sensor types as needed
} SensorType;

typedef struct __attribute__((packed))  {
    uint8_t startDelimiter;
    SensorType sensorType;
    uint32_t timestamp;
    uint16_t sensorData; // Assuming 2 bytes for sensor data
    uint8_t checksum;
    uint8_t endDelimiter;
} CustomMessage;



void sendMessage(SensorType sensorType, uint32_t timestamp, uint16_t sensorData);

void sendLightSensorData(uint16_t sensorData);
#endif /* INC_CUSTOMPROTOCOL_H_ */
