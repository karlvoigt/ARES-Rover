#ifndef CUSTOM_PROTOCOL_H
#define CUSTOM_PROTOCOL_H

#include <stdint.h>

// Custom protocol structure and constants
#define START_DELIMITER 0xAA
#define END_DELIMITER 0x55
#define CUSTOM_MESSAGE_SIZE 10 // Expected size of CustomMessage


typedef enum {
    TEMPERATURE_SENSOR = 0x01,
    HUMIDITY_SENSOR = 0x02,
    LIGHT_SENSOR = 0x03
    // Add other sensor types as needed
} SensorType;

typedef struct {
    uint8_t startDelimiter;
    SensorType sensorType;
    uint32_t timestamp;
    uint16_t sensorData;
    uint8_t checksum;
    uint8_t endDelimiter;
} CustomMessage;

#endif // CUSTOM_PROTOCOL_H
