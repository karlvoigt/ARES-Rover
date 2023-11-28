/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define UART_BUFFER_SIZE 128  // Adjust the size as needed
#define INSTRUCTION_BUFFER_SIZE 5 //Size of the buffer for the navigation instructions


typedef enum {
    WAIT_FOR_START_DELIMITER,
    WAIT_FOR_DATA_LENGTH_HIGH,
    WAIT_FOR_DATA_LENGTH_LOW,
    WAIT_FOR_DATA
} ReceiveState;

typedef enum {
    WAIT_FOR_GYRO_X_HIGH,
    WAIT_FOR_GYRO_X_LOW,
    WAIT_FOR_GYRO_Y_HIGH,
    WAIT_FOR_GYRO_Y_LOW,
    WAIT_FOR_GYRO_Z_HIGH,
    WAIT_FOR_GYRO_Z_LOW,
    WAIT_FOR_ACCEL_X_HIGH,
    WAIT_FOR_ACCEL_X_LOW,
    WAIT_FOR_ACCEL_Y_HIGH,
    WAIT_FOR_ACCEL_Y_LOW,
    WAIT_FOR_ACCEL_Z_HIGH,
    WAIT_FOR_ACCEL_Z_LOW
} MEMS_Data_State;

typedef enum uint8_t {
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

typedef enum {
    MEASURE_NO = 0b10,
    MEASURE_YES = 0b01,
    MEASURE_OPTIONAL = 0b11
} MeasureState;

typedef struct __attribute__((packed)) {
    MeasureState temperature : 2;
    MeasureState humidity : 2;
    MeasureState visibleLight : 2;
    MeasureState infrared : 2;
} SensorMeasurements;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
