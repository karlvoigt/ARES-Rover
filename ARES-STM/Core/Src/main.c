/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "queue.h"
#include "LTR329.h"
#include "CustomProtocol.h"
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART2Task */
osThreadId_t UART2TaskHandle;
const osThreadAttr_t UART2Task_attributes = {
  .name = "UART2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UART1Task */
osThreadId_t UART1TaskHandle;
const osThreadAttr_t UART1Task_attributes = {
  .name = "UART1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LightSensor_Tas */
osThreadId_t LightSensor_TasHandle;
const osThreadAttr_t LightSensor_Tas_attributes = {
  .name = "LightSensor_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for uart1Queue */
osMessageQueueId_t uart1QueueHandle;
const osMessageQueueAttr_t uart1Queue_attributes = {
  .name = "uart1Queue"
};
/* Definitions for uart2Queue */
osMessageQueueId_t uart2QueueHandle;
const osMessageQueueAttr_t uart2Queue_attributes = {
  .name = "uart2Queue"
};
/* USER CODE BEGIN PV */

uint8_t uart1_buffer[1], uart2_buffer[1];
uint8_t uart1_accumulate_buffer[UART_BUFFER_SIZE],uart2_accumulate_buffer[UART_BUFFER_SIZE];;
uint8_t uart1_accumulate_pos = 0;
uint8_t uart2_accumulate_pos = 0;
uint16_t light_ch0, light_ch1;
uint8_t receivedData[100]; // Adjust size as needed
const uint8_t verbose=1;

ReceiveState UART1receiveState = WAIT_FOR_START_DELIMITER;
uint16_t dataLength = 0;
uint8_t dataReceived = 0;
uint8_t* dma_buffer = NULL;
float curX = 0;
float curY = 0;
float curAngle = 0;
float targetX = 0;
float targetY = 0;
float targetAngle = 0;
float plannedAngle = 0;
SensorMeasurements targetMeasurements;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void UART2_Task(void *argument);
void UART1_Task(void *argument);
void LightSensorTask(void *argument);

/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void transmitInstructions(navigationInstruction* instructions, uint8_t instructionCnt);
uint8_t receiveInstructions(uint8_t* data, uint16_t dataLength);
void printInstructions(navigationInstruction* instructions, uint16_t numInstructions);
uint8_t calculatePath(navigationInstruction* instructions);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t calculatePath(navigationInstruction* instructions){
    uint8_t instructionCount = 0;
    float deltaX = targetX - curX;
    float deltaY = targetY - curY;

    // Calculate the angle to the target point
    float targetAngle = atan2(deltaY, deltaX) * (180.0 / M_PI);
    targetAngle = fmod(targetAngle + 360.0, 360.0); // Ensure the angle is between 0 and 360

    // Calculate the shortest turn to the target angle
    float turnAngle = targetAngle - curAngle;
    if (turnAngle < -180.0) turnAngle += 360.0;
    if (turnAngle > 180.0) turnAngle -= 360.0;

    // Turn to face the target point
    if (turnAngle != 0.0) {
        instructions[instructionCount].instructionType = turnAngle > 0.0 ? CLOCKWISE : COUNTERCLOCKWISE;
        instructions[instructionCount].instructionValue = fabs(turnAngle);
        instructionCount++;
        plannedAngle = targetAngle;
    }

    // Move to the target point
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    instructions[instructionCount].instructionType = turnAngle >= -90.0 && turnAngle <= 90.0 ? FORWARD : BACKWARD;
    instructions[instructionCount].instructionValue = distance;
    instructionCount++;

    // Add a stop instruction
    instructions[instructionCount].instructionType = STOP;
    instructions[instructionCount].instructionValue = 0.0;
    instructionCount++;

    return instructionCount;
}

void transmitInstructions(navigationInstruction* instructions, uint8_t instructionCnt){
    /* Transmit the instructions to the line bot
      Transmissions consists of a START_DELIMITER, Instruction Count, payload and END_DELIMITER
      Instruction Count is a 1 byte value
      Payload consists of the instructions which are 5 bytes each
      The data is transmitted on USART2 using HAL_UART_Transmit
    */
    uint8_t transmission[instructionCnt*sizeof(navigationInstruction) + 3];

    transmission[0] = START_DELIMITER;
    transmission[1] = instructionCnt;
    for (int i = 0; i < instructionCnt; i++) {
        transmission[2 + i*sizeof(navigationInstruction)] = instructions[i].instructionType;
        memcpy(&transmission[3 + i*sizeof(navigationInstruction)], &instructions[i].instructionValue, 4);
    }
    transmission[instructionCnt*sizeof(navigationInstruction) + 2] = END_DELIMITER;
    // HAL_UART_Transmit(&huart2, transmission, instructionCnt*sizeof(navigationInstruction) + 3, HAL_MAX_DELAY);
    receiveInstructions(transmission, instructionCnt*sizeof(navigationInstruction) + 3);
}

// Function to receive and decode instructions
uint8_t receiveInstructions(uint8_t* data, uint16_t dataLength) {
    // Check the start and end delimiters
    if (data[0] != START_DELIMITER || data[dataLength - 1] != END_DELIMITER) {
        return 0; // Invalid data
    }

    // Calculate the number of instructions
    uint16_t numInstructions = (dataLength - 2) / sizeof(navigationInstruction);
    //check that the calculated number of instructions matches the transmitted number
    if (numInstructions != data[1]) {
        return 0; // Invalid data
    }
    navigationInstruction instructions[numInstructions];

    // Decode the instructions
    for (uint16_t i = 0; i < numInstructions; i++) {
        memcpy(&instructions[i], &data[i * sizeof(navigationInstruction) + 1], sizeof(navigationInstruction));
    }

    // Print the instructions
    printInstructions(instructions, numInstructions);
    return numInstructions;
}

// Function to print the instructions
void printInstructions(navigationInstruction* instructions, uint16_t numInstructions) {
  for (uint16_t i = 0; i < numInstructions; i++) {
    printf("Instruction %d: ", i);
    switch (instructions[i].instructionType) {
      case LEFT:
        printf("Turn left and drive %f mm\n", instructions[i].instructionValue);
        break;
      case RIGHT:
        printf("Turn right and drive %f mm\n", instructions[i].instructionValue);
        break;
      case FORWARD:
        printf("Drive %f mm forwards\n", instructions[i].instructionValue);
        break;
      case BACKWARD:
        printf("Drive %f mm backwards\n", instructions[i].instructionValue);
        break;
      case CLOCKWISE:
        printf("Turn clockwise %f degrees\n", instructions[i].instructionValue);
        break;
      case COUNTERCLOCKWISE:
        printf("Turn counterclockwise %f degrees\n", instructions[i].instructionValue);
        break;
      case STOP:
        printf("Stop\n");
        break;
      default:
        printf("Unknown instruction type\n");
        break;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, uart2_buffer, 1);
  HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
  printf("Setup complete\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart1Queue */
  uart1QueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &uart1Queue_attributes);

  /* creation of uart2Queue */
  uart2QueueHandle = osMessageQueueNew (2, sizeof(Dash7ToSTM32Message), &uart2Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART2Task */
  UART2TaskHandle = osThreadNew(UART2_Task, NULL, &UART2Task_attributes);

  /* creation of UART1Task */
  UART1TaskHandle = osThreadNew(UART1_Task, NULL, &UART1Task_attributes);

  /* creation of LightSensor_Tas */
  LightSensor_TasHandle = osThreadNew(LightSensorTask, NULL, &LightSensor_Tas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
    {
        switch (UART1receiveState) {
            case WAIT_FOR_START_DELIMITER:
                if (uart1_buffer[0] == START_DELIMITER) {
                    UART1receiveState = WAIT_FOR_DATA_LENGTH_HIGH;
                }
                HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
                break;

            case WAIT_FOR_DATA_LENGTH_HIGH:
                dataLength = uart1_buffer[0] << 8;
                UART1receiveState = WAIT_FOR_DATA_LENGTH_LOW;
                HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
                break;

            case WAIT_FOR_DATA_LENGTH_LOW:
                dataLength |= uart1_buffer[0];
                UART1receiveState = WAIT_FOR_DATA;
                dma_buffer = (uint8_t*)pvPortMalloc((dataLength*12 + 1) * sizeof(uint8_t)); // Allocate buffer for DMA transfer
                HAL_UART_Receive_DMA(&huart1, dma_buffer, dataLength*12 + 1);
                break;

            case WAIT_FOR_DATA:
                // DMA transfer is complete
                HAL_UART_DMAStop(&huart1); // Stop the DMA transfer

                // Check if the last byte is the end delimiter
                if (dma_buffer[dataLength*12] == END_DELIMITER) {
                    // Process the received data here
                    // You can send the data to a FreeRTOS task using a queue
                    xQueueSendFromISR(uart1QueueHandle, dma_buffer, NULL);
                }

                // Free the DMA buffer
                vPortFree(dma_buffer);
                dma_buffer = NULL;

                // Restart the interrupt-based reception
                UART1receiveState = WAIT_FOR_START_DELIMITER;
                HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
                break;
        }
    }
	// if (huart->Instance == USART1)
	// {
	// 	xQueueSendFromISR(uart1QueueHandle, uart1_buffer, NULL);
	// 	HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
	// }
	else if (huart->Instance == USART2)
	{
		HAL_UART_Receive_IT(&huart2, uart2_buffer, 1);
		xQueueSendFromISR(uart2QueueHandle, uart2_buffer, NULL);
=======
//		HAL_UART_Receive_IT(&huart2, uart2_buffer, 1);
      newInstruction.startDelimiter = uart2_buffer[11];

      memcpy(&newInstruction.xCoord, &uart2_buffer[0], sizeof(float));
      memcpy(&newInstruction.yCoord, &uart2_buffer[4], sizeof(float));

      newInstruction.sensorControl = uart2_buffer[8];
      newInstruction.endDelimiter = uart2_buffer[9];
//		xQueueSendFromISR(uart2QueueHandle, &msg, NULL);

		xTaskResumeFromISR(navigationTaskHandle);
>>>>>>> Stashed changes

		// Prepare to receive the next character
//		HAL_UART_Receive_DMA(&huart2, uart2_buffer, sizeof(uart2_buffer));

	}
}



void printSensorMeasurements(SensorMeasurements measurements) {
    const char* measurementStrings[] = {"?","No", "Yes", "Optional"};

    printf("\nTemperature: %s\n", measurementStrings[measurements.temperature]);
    printf("Humidity: %s\n", measurementStrings[measurements.humidity]);
    printf("Visible Light: %s\n", measurementStrings[measurements.visibleLight]);
    printf("Infrared: %s\n", measurementStrings[measurements.infrared]);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
<<<<<<< Updated upstream
=======
   HAL_UART_Receive_IT(&huart2, uart2_buffer, 1);
//  HAL_UART_Receive_DMA(&huart2, uart2_buffer, sizeof(uart2_buffer));
  HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
  printf("Setup complete\n");
>>>>>>> Stashed changes
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UART2_Task */
/**
* @brief Function implementing the UART2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART2_Task */
void UART2_Task(void *argument)
{
  /* USER CODE BEGIN UART2_Task */
<<<<<<< Updated upstream
	Dash7ToSTM32Message receivedMessage;
=======
//	uint8_t data[sizeof(Dash7ToSTM32Message)];
	Dash7ToSTM32Message msg;
//	MessageUnion receivedMessage;
>>>>>>> Stashed changes
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(uart2QueueHandle, &receivedMessage, portMAX_DELAY) == pdPASS)
    {
      //Check for message structure
      if (receivedMessage.startDelimiter != START_DELIMITER || receivedMessage.endDelimiter != END_DELIMITER)
      {
        // Invalid message structure
        continue;
      } else {
        targetX = receivedMessage.xCoord;
        targetY = receivedMessage.yCoord;
        targetMeasurements = *(SensorMeasurements*)&receivedMessage.sensorControl;
        printSensorMeasurements(targetMeasurements);
        printf("Received message with x: %f, y: %f\n", targetX, targetY);
      }
    }
  }
  /* USER CODE END UART2_Task */
}

/* USER CODE BEGIN Header_UART1_Task */
/**
* @brief Function implementing the UART1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART1_Task */
void UART1_Task(void *argument)
{
  /* USER CODE BEGIN UART1_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART1_Task */
}

/* USER CODE BEGIN Header_LightSensorTask */
/**
* @brief Function implementing the LightSensor_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LightSensorTask */
void LightSensorTask(void *argument)
{
  /* USER CODE BEGIN LightSensorTask */
	LTR329_Init();
	/* Infinite loop */
	  for(;;)
	  {
		// Read light data
		LTR329_Read_Light(&light_ch0, &light_ch1);
//		if (verbose) printf("Light Ch0: %d \n Light Ch1: %d\n________\n\n",light_ch0,light_ch1);
		sendLightSensorData(light_ch0);
		osDelay(2000);
	  }
	  // In case we accidentally exit from task loop
	osThreadTerminate(NULL);
  /* USER CODE END LightSensorTask */
}

<<<<<<< Updated upstream
=======
/* USER CODE BEGIN Header_TempTask */
/**
* @brief Function implementing the Temp_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TempTask */
void TempTask(void *argument)
{
  /* USER CODE BEGIN TempTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TempTask */
}

/* USER CODE BEGIN Header_StartNavigationTask */
/**
* @brief Function implementing the navigationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNavigationTask */
void StartNavigationTask(void *argument)
{
  /* USER CODE BEGIN StartNavigationTask */
  /* Infinite loop */
  for(;;)
  {
	  if (newInstruction.startDelimiter == START_DELIMITER && newInstruction.endDelimiter == END_DELIMITER)
		{
		  targetX = newInstruction.xCoord;
		  targetY = newInstruction.yCoord;
		  targetMeasurements = *(SensorMeasurements*)&newInstruction.sensorControl;
		  printSensorMeasurements(targetMeasurements);
		  printf("Received message with x: %f, y: %f\n", targetX, targetY);
      navigationInstruction instructions[INSTRUCTION_BUFFER_SIZE];
      uint8_t instructionCnt = calculatePath(instructions);
      transmitInstructions(instructions, instructionCnt);
		}
		vTaskSuspend(NULL);
  }
  /* USER CODE END StartNavigationTask */
}

>>>>>>> Stashed changes
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
