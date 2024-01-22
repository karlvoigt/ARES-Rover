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
#include "SHT40.h"
#include "CustomProtocol.h"
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UID 0x0001
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for NavigationTask */
osThreadId_t NavigationTaskHandle;
const osThreadAttr_t NavigationTask_attributes = {
  .name = "NavigationTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for DemoTask */
osThreadId_t DemoTaskHandle;
const osThreadAttr_t DemoTask_attributes = {
  .name = "DemoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for SensingTask */
osThreadId_t SensingTaskHandle;
const osThreadAttr_t SensingTask_attributes = {
  .name = "SensingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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

uint8_t uart1_buffer[sizeof(LineBotToSTM32Message)], uart2_buffer[sizeof(BLERoverMessage)];
uint8_t uart1_accumulate_buffer[UART_BUFFER_SIZE],uart2_accumulate_buffer[UART_BUFFER_SIZE];;
uint8_t uart1_accumulate_pos = 0;
uint8_t uart2_accumulate_pos = 0;
uint16_t light_ch0, light_ch1,raw_temp,raw_humidity;
uint8_t receivedData[100]; // Adjust size as needed
const uint8_t verbose=0;

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
Dash7ToSTM32Message newInstruction;
BLERoverMessage bleMessage;
uint8_t BLEstarted;
uint8_t shouldStop;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartNavigationTask(void *argument);
void StartDemoTask(void *argument);
void StartSensingTask(void *argument);

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
//void printInstructions(navigationInstruction* instructions, uint16_t numInstructions);
uint8_t calculatePath(navigationInstruction* instructions);

void Toggle_LED(void) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}
void Enter_Stop2_Mode(void) {
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

    // Enter Stop 2 mode
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    printf("ISR");
//    if (GPIO_Pin == GPIO_PIN_4) {
//        // This function will be called by the HAL interrupt handler which should clear the pending bit
//        // Perform tasks upon EXTI interrupt from PA4
//        // For example, signaling a task or setting a flag
////        SystemClock_Config();
//        printf("ISR");
//        // Set a flag to indicate that a wakeup event has occurred
//        justWokenUpFromStop2 = 1;
//        shouldStop = 0;
//    }
//}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_5) {
        // This function will be called by the HAL interrupt handler which should clear the pending bit
        // Perform tasks upon EXTI interrupt from PA4
        // For example, signaling a task or setting a flag
//        SystemClock_Config();
        // Set a flag to indicate that a wakeup event has occurred
        shouldStop = 0;
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
}

uint8_t calculatePath(navigationInstruction* instructions){
    uint8_t instructionCount = 0;
    float deltaX = targetX - curX;
    float deltaY = targetY - curY;

    // Calculate the angle to the target point
    float targetAngle = atan2(deltaX, deltaY) * (180.0 / M_PI);
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
    instructions[instructionCount].instructionType =  FORWARD ;
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

	//Wakeup LineBot
	HAL_GPIO_WritePin(LinebotTrigger_GPIO_Port, LinebotTrigger_Pin, GPIO_PIN_RESET);

    uint8_t transmission[instructionCnt*sizeof(navigationInstruction) + 3];

    transmission[0] = START_DELIMITER;
    transmission[1] = instructionCnt;
    for (int i = 0; i < instructionCnt; i++) {
        transmission[2 + i*sizeof(navigationInstruction)] = instructions[i].instructionType;
        memcpy(&transmission[3 + i*sizeof(navigationInstruction)], &instructions[i].instructionValue, 4);
    }
    transmission[instructionCnt*sizeof(navigationInstruction) + 2] = END_DELIMITER;
    HAL_UART_Transmit(&huart1, transmission, instructionCnt*sizeof(navigationInstruction) + 3, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(LinebotTrigger_GPIO_Port, LinebotTrigger_Pin, GPIO_PIN_SET);
    //TODO: wait for transmission to complete and go to sleep
    vTaskDelay(50);
//    Enter_Stop2_Mode();
    shouldStop =1;
//    receiveInstructions(transmission, instructionCnt*sizeof(navigationInstruction) + 3);
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
//  HAL_UART_Receive_IT(&huart2, uart2_buffer, 1);
//  HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
  if (verbose) printf("Starting Setup\n");
  shouldStop = 0;

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

  /* creation of NavigationTask */
  NavigationTaskHandle = osThreadNew(StartNavigationTask, NULL, &NavigationTask_attributes);

  /* creation of DemoTask */
  DemoTaskHandle = osThreadNew(StartDemoTask, NULL, &DemoTask_attributes);

  /* creation of SensingTask */
  SensingTaskHandle = osThreadNew(StartSensingTask, NULL, &SensingTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  vTaskSuspend(NavigationTaskHandle);
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
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, LinebotTrigger_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LinebotIntPin_Pin Dash7IntPin_Pin */
  GPIO_InitStruct.Pin = LinebotIntPin_Pin|Dash7IntPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LinebotTrigger_Pin */
  GPIO_InitStruct.Pin = LinebotTrigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LinebotTrigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (uart1_buffer[0]== START_DELIMITER && uart1_buffer[13]==END_DELIMITER) {
		  memcpy(&curX, &uart1_buffer[1], sizeof(float));
		  memcpy(&curY, &uart1_buffer[5], sizeof(float));
		  memcpy(&curAngle, &uart1_buffer[9], sizeof(float));

		  vTaskNotifyGiveFromISR(SensingTaskHandle, pdFALSE);
		}
		HAL_UART_Receive_DMA(&huart1, uart1_buffer, 14);
	}
	else if (huart->Instance == USART2)
	{
		//Can either be a BLE message or a new instruction
		//Case of new instruction
		if (uart2_buffer[0] == START_DELIMITER && uart2_buffer[10] == END_DELIMITER && uart2_buffer[11] == 0x00) {
		  // Received a new instruction
		  // Copy the instruction into the newInstruction variable
		  newInstruction.startDelimiter = uart2_buffer[0];

		  memcpy(&newInstruction.xCoord, &uart2_buffer[1], sizeof(float));
		  memcpy(&newInstruction.yCoord, &uart2_buffer[5], sizeof(float));

		  newInstruction.sensorControl = uart2_buffer[9];
		  newInstruction.endDelimiter = uart2_buffer[10];

		  // HAL_UART_Transmit(&huart2, uart2_buffer, 11, HAL_MAX_DELAY);

		  // Notify the StartNavigationTask
		  vTaskNotifyGiveFromISR(NavigationTaskHandle, pdFALSE);
		}
		//case of BLE message
		else if (uart2_buffer[0] == START_DELIMITER && uart2_buffer[26] == END_DELIMITER) {
		  // Received a BLE message
		  // Copy the message into the receivedData variable
		  memcpy(&bleMessage, uart2_buffer, 27);
		  // printf("BLE Message Received\n");
		  // printf("BLE Message: ");
		  // HAL_UART_Transmit(&huart2, &bleMessage, 27, HAL_MAX_DELAY);
		}

		// Prepare to receive the next message
		HAL_UART_Receive_DMA(&huart2, uart2_buffer, 27);
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
  BLEstarted = 0;
  HAL_UART_Receive_DMA(&huart2, uart2_buffer,27);
  HAL_UART_Receive_DMA(&huart1, uart1_buffer,14);
  LTR329_Init();
  if (verbose) printf("Setup complete\n");
  vTaskSuspend(NULL);
//  Enter_Stop2_Mode();
  /* Infinite loop */
  for(;;)
  {
	  Toggle_LED();
	  osDelay(500);
  }
  /* USER CODE END 5 */
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
    uint32_t ulNotificationValue;
    BaseType_t xResult = xTaskNotifyWait( pdFALSE, 0, &ulNotificationValue, portMAX_DELAY );

    if( xResult == pdPASS )
    {
      /* A notification was received. */
      if (newInstruction.startDelimiter == START_DELIMITER && newInstruction.endDelimiter == END_DELIMITER)
      {
        targetX = newInstruction.xCoord;
        targetY = newInstruction.yCoord;
        memcpy(&targetMeasurements, &newInstruction.sensorControl, sizeof(targetMeasurements));
        navigationInstruction instructions[INSTRUCTION_BUFFER_SIZE];
        uint8_t instructionCnt = calculatePath(instructions);
        transmitInstructions(instructions, instructionCnt);
      } 
    }
  }
  /* USER CODE END StartNavigationTask */
}

/* USER CODE BEGIN Header_StartDemoTask */
/**
* @brief Function implementing the DemoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDemoTask */
void StartDemoTask(void *argument)
{
  /* USER CODE BEGIN StartDemoTask */
	if (verbose) printf("Start Demo Task");
	vTaskDelay(10); //Gives LineBot time to startup before sending instructions
	targetX = newInstruction.xCoord;
	targetY = newInstruction.yCoord;
	memcpy(&targetMeasurements, &newInstruction.sensorControl, sizeof(targetMeasurements));
	navigationInstruction instructions[10];
  uint8_t instructionCount = 0;

  //Create array of points to move to
  float points[4][2] = {{0,250},{250,250},{0,500},{250,750}};

  //Move to each point
  for (int i = 0; i < 4; i++) {
    targetX = points[i][0];
    targetY = points[i][1];
    float deltaX = targetX - curX;
    float deltaY = targetY - curY;
    curX = targetX;
    curY = targetY;

    // Calculate the angle to the target point
    float targetAngle = atan2(deltaX, deltaY) * (180.0 / M_PI);
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
        curAngle = targetAngle;
    }

    // Move to the target point
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    instructions[instructionCount].instructionType =  FORWARD;
    instructions[instructionCount].instructionValue = distance;
    instructionCount++;
  }

  // Add a stop instruction
  instructions[instructionCount].instructionType = STOP;
  instructions[instructionCount].instructionValue = 0.0;
  instructionCount++;

  transmitInstructions(instructions, instructionCount);

	vTaskSuspend(NULL); //Stop DemoTask
  /* Infinite loop */
  /* USER CODE END StartDemoTask */
}

/* USER CODE BEGIN Header_StartSensingTask */
/**
* @brief Function implementing the SensingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensingTask */
void StartSensingTask(void *argument)
{
  /* USER CODE BEGIN StartSensingTask */
  /* Infinite loop */
  for(;;)
  {
    uint32_t ulNotificationValue;
    BaseType_t xResult = xTaskNotifyWait( pdFALSE, 0, &ulNotificationValue, portMAX_DELAY );
//	 BaseType_t xResult =  pdPASS;
//	vTaskDelay(1000);

    if( xResult == pdPASS )
    {
      /* A notification was received. */
      //Read temp & humidity data
      SHT40_Read_Temp_Hum(&raw_temp, &raw_humidity);
      // Read light data
	  LTR329_Read_Light(&light_ch0, &light_ch1);

      //Send data to Dash7
      STM32ToDash7Message msg;
      msg.startDelimiter = START_DELIMITER;
      msg.xCoord = curX;
      msg.yCoord = curY;
      msg.angle = curAngle;
      msg.light = light_ch0;
      msg.ir = light_ch1;
      msg.temperature = raw_temp;
      msg.humidity = raw_humidity;
      msg.uid = UID;
      msg.battery = 100;
      msg.endDelimiter = END_DELIMITER;

      // Send the message to the Dash7 task
      HAL_UART_Transmit(&huart2, (uint8_t*) &msg, sizeof(msg), HAL_MAX_DELAY);
      vTaskDelay(50);
      shouldStop = 1;
    }
  }
  /* USER CODE END StartSensingTask */
}

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
