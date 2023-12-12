/*
 * CustomProtocol.h
 *
 *  Created on: Nov 3, 2023
 *      Author: karlvoigt
 */

#ifndef INC_CUSTOMPROTOCOL_H_
#define INC_CUSTOMPROTOCOL_H_

#include <stdint.h>

#define START_DELIMITER 0x5B    // '['
#define END_DELIMITER   0x5D    // ']'
#define CUSTOM_MESSAGE_SIZE 10  // Expected size of CustomMessage

typedef enum {
    SENSOR_TYPE_TEMPERATURE = 0x1,
    SENSOR_TYPE_HUMIDITY    = 0x2,
    SENSOR_TYPE_LIGHT       = 0x3,
    SENSOR_TYPE_IR          = 0x4
    // Add more sensor types as needed
} SensorType;

typedef struct __attribute__((packed)) {
    uint8_t startDelimiter;
    SensorType sensorType;
    uint32_t timestamp;
    uint16_t sensorData;
    uint32_t checksum;
    uint8_t endDelimiter;
} CustomMessage;

typedef struct __attribute__((packed)) {
    int16_t gyroX; // 2 bytes each for x, y, z
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX; // 2 bytes each for x, y, z
    int16_t accelY;
    int16_t accelZ;
} MEMS_Data;

typedef enum {
    STMtoLB_MESSAGE = 0x1,
    LBtoSTM_MESSAGE = 0x2,
    STMtoD7_MESSAGE = 0x3,
    D7toSTM_MESSAGE = 0x4,
    BLE_MESSAGE     = 0x5
    // Add more command types as needed
} MessageType;

typedef struct __attribute__((packed)) {
    uint8_t startDelimiter;
    uint8_t commandType;
    uint8_t valueLength;
    uint16_t value; // Assuming 2 bytes for value
    uint8_t endDelimiter;
} STM32ToLineBotMessage;

typedef struct __attribute__((packed)) {
    uint8_t startDelimiter;
    uint16_t dataLength;
    MEMS_Data* data; // Pointer to an array of MEMS_Data
    uint8_t endDelimiter;
} LineBotToSTM32Message;

typedef struct __attribute__((packed)) {
    uint8_t startDelimiter;
    float xCoord;
    float yCoord;
    float angle; // In degrees
    uint16_t temperature; // Variable length, as defined by sensors
    uint16_t humidity;
    uint16_t light;
    uint16_t ir;
    uint32_t timestamp;
    uint8_t battery;
    uint8_t endDelimiter;
} STM32ToDash7Message;

//TODO: Add support for timestamps?
typedef struct __attribute__((packed)) {
    uint8_t startDelimiter;
    float xCoord;
    float yCoord;
    uint8_t sensorControl;
    uint8_t endDelimiter;
} Dash7ToSTM32Message;

typedef enum {
    MESSAGE_TYPE_POSITION_UPDATE = 0x01,
    MESSAGE_TYPE_SENSOR_DATA,
    // Add more message types as needed
} BLEMessageType;

typedef struct __attribute__((packed)) {
    uint8_t startDelimiter;
    MessageType messageType;
    uint8_t payloadLength;
    uint16_t payload; // Variable length, as defined by payload
    uint8_t endDelimiter;
} BLERoverMessage;


typedef union {
    STM32ToLineBotMessage stmToLb;
    LineBotToSTM32Message lbToStm;
    STM32ToDash7Message stmToD7;
    Dash7ToSTM32Message d7ToStm;
    BLERoverMessage ble;
} MessageUnion;

void sendMessage(SensorType sensorType, uint32_t timestamp, uint16_t sensorData);

void sendLightSensorData(uint16_t sensorData);

void parseUARTMessage(MessageUnion* message, MessageType messageType, uint8_t* data);

// Function to receive and decode instructions
//uint8_t receiveInstructions(uint8_t* data, uint16_t dataLength);

// Function to print the instructions
//void printInstructions(navigationInstruction* instructions, uint16_t numInstructions);

#endif /* INC_CUSTOMPROTOCOL_H_ */
