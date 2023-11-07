/*
 * CustomProtocol.c
 *
 *  Created on: Nov 3, 2023
 *      Author: karlvoigt
 */


#include "CustomProtocol.h"
#include <stdio.h>
#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart2;


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

void sendLightSensorData(uint16_t sensorData) {
	uint32_t curTime;
	curTime = HAL_GetTick();
	sendMessage(0x02, curTime, sensorData);
}
