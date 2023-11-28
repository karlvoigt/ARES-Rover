/*
 * CustomProtocol.c
 *
 *  Created on: Nov 3, 2023
 *      Author: karlvoigt
 */


#include "CustomProtocol.h"
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2;


<<<<<<< Updated upstream
void sendMessage(SensorType sensorType, uint32_t timestamp, uint16_t sensorData) {
    CustomMessage message;
    message.startDelimiter = START_DELIMITER;
    message.sensorType = sensorType;
    message.timestamp = timestamp;
    message.sensorData = sensorData;
    message.checksum = START_DELIMITER + sensorType + (timestamp & 0xFF) + ((timestamp >> 8) & 0xFF) + ((timestamp >> 16) & 0xFF) + ((timestamp >> 24) & 0xFF) + (sensorData & 0xFF) + ((sensorData >> 8) & 0xFF);
    message.endDelimiter = END_DELIMITER;
    HAL_UART_Transmit(&huart2, (uint8_t*)&message, CUSTOM_MESSAGE_SIZE, HAL_MAX_DELAY);
}
=======
 void sendMessage(SensorType sensorType, uint32_t timestamp, uint16_t sensorData) {
     CustomMessage message;
     message.startDelimiter = START_DELIMITER;
     message.sensorType = sensorType;
     message.timestamp = timestamp;
     message.sensorData = sensorData;
     message.checksum = START_DELIMITER + sensorType + (timestamp & 0xFF) + ((timestamp >> 8) & 0xFF) + ((timestamp >> 16) & 0xFF) + ((timestamp >> 24) & 0xFF) + (sensorData & 0xFF) + ((sensorData >> 8) & 0xFF);
     message.endDelimiter = END_DELIMITER;
     HAL_UART_Transmit(&huart2, (uint8_t*)&message, CUSTOM_MESSAGE_SIZE, HAL_MAX_DELAY);
 }
>>>>>>> Stashed changes

 void sendLightSensorData(uint16_t sensorData) {
 	uint32_t curTime;
 	curTime = HAL_GetTick();
 	sendMessage(0x02, curTime, sensorData);
 }

// MessageUnion* parseUARTMessage(MessageType messageType, uint8_t* data) {
// 	MessageUnion* message;
//     switch (messageType) {
//         case STMtoLB_MESSAGE:
//             message->stmToLb = (STM32ToLineBotMessage*)malloc(sizeof(STM32ToLineBotMessage));
//             memcpy(&message->stmToLb, data, sizeof(STM32ToLineBotMessage));
//             return message;
//         case LBtoSTM_MESSAGE:
//             message->lbToStm = (LineBotToSTM32Message*)malloc(sizeof(LineBotToSTM32Message));
//             memcpy(&message->lbToStm, data, sizeof(LineBotToSTM32Message));
//             return message;
//         case STMtoD7_MESSAGE:
//             MessageUnion* message = (MessageUnion*)malloc(sizeof(STM32ToDash7Message));
//             STM32ToDash7Message* message = (STM32ToDash7Message*)malloc(sizeof(STM32ToDash7Message));
//             memcpy(message, data, sizeof(STM32ToDash7Message));
//             return message;
//         case D7toSTM_MESSAGE:
//             MessageUnion* message = (MessageUnion*)malloc(sizeof(Dash7ToSTM32Message));
//             Dash7ToSTM32Message* message = (Dash7ToSTM32Message*)malloc(sizeof(Dash7ToSTM32Message));
//             memcpy(message, data, sizeof(Dash7ToSTM32Message));
//             return message;
//         case BLE_MESSAGE:
//             MessageUnion* message = (MessageUnion*)malloc(sizeof(BLERoverMessage));
//             BLERoverMessage* message = (BLERoverMessage*)malloc(sizeof(BLERoverMessage));
//             memcpy(message, data, sizeof(BLERoverMessage));
//             return message;

//         if (message == NULL) {
//             return NULL; // Failed to allocate memory
//     }
//     }
// }

void parseUARTMessage(MessageUnion* message, MessageType messageType, uint8_t* data) {
    switch (messageType) {
        case STMtoLB_MESSAGE:
            memcpy(&message->stmToLb, data, sizeof(STM32ToLineBotMessage));
            break;
        case LBtoSTM_MESSAGE:
            memcpy(&message->lbToStm, data, sizeof(LineBotToSTM32Message));
            break;
        case STMtoD7_MESSAGE:
            memcpy(&message->stmToD7, data, sizeof(STM32ToDash7Message));
            break;
        case D7toSTM_MESSAGE:
            memcpy(&message->d7ToStm, data, sizeof(Dash7ToSTM32Message));
            break;
        case BLE_MESSAGE:
            memcpy(&message->ble, data, sizeof(BLERoverMessage));
            break;
        default:
            // Handle unknown message type
            break;
    }
}
