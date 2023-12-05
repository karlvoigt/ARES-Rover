/*
 * CustomProtocol.h
 *
 *  Created on: Nov 3, 2023
 *      Author: karlvoigt
 */

#ifndef INC_CUSTOMPROTOCOL_H_
#define INC_CUSTOMPROTOCOL_H_

#define START_DELIMITER 0x5B    // '['
#define END_DELIMITER   0x5D    // ']'
#define CUSTOM_MESSAGE_SIZE 10  // Expected size of CustomMessage

// #include <stdio.h>
//#include <stdlib.h>
#include <stdint.h>

volatile uint8_t USART_RX_Queue_has_data; //Flag to indicate whether there is data in the USART queue
volatile uint8_t USART_RX_transmission_complete; //Flag to indicate whether the USART transmission is complete


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
    float xCoord;
    float yCoord;
    float yaw;
    uint8_t endDelimiter;
} LineBotToSTM32Message;

typedef union {
    STM32ToLineBotMessage stmToLb;
    LineBotToSTM32Message lbToStm;
} MessageUnion;

//typedef enum uint8_t {
typedef enum {	
    RIGHT,
    LEFT,
    FORWARD,
    BACKWARD,
    CLOCKWISE,
    COUNTERCLOCKWISE,
    STOP,
    MEASURE,
    SPEED
} navInstructionType;

typedef struct __attribute__((packed)) {
    navInstructionType instructionType : 8;
    float instructionValue;
} navigationInstruction;

#endif /* INC_CUSTOMPROTOCOL_H_ */
