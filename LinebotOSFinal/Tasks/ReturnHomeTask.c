#include "ReturnHomeTask.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "NavigationTask.h"
#include "EncoderPositioning.h"
#include "CustomProtocol.h"
#include <math.h>

//Function definitions
void InitReturnHomeTask()
{
	xTaskCreate( WorkerReturnHome, "ReturnHome", 256, NULL, tskIDLE_PRIORITY+3, ReturnHomeTaskHandle );	
    vTaskSuspend(ReturnHomeTaskHandle);
}

static void WorkerReturnHome(void *pvParameters)
{
	while (1)
	{
        //Clear instruction queue
        xQueueReset(InstructionQueue);

        navigationInstruction instructions[3];

        uint8_t instructionCount = 0;
        float deltaX = - getEncoderXCoord();
        float deltaY = - getEncoderYCoord();
        float curAngle = getEncoderYaw();

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
            xQueueSendToBack(InstructionQueue, &instructions[instructionCount], 0);
            instructionCount++;
        }

        // Move to the target point
        float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        instructions[instructionCount].instructionType = turnAngle >= -90.0 && turnAngle <= 90.0 ? FORWARD : BACKWARD;
        instructions[instructionCount].instructionValue = distance;
        xQueueSendToBack(InstructionQueue, &instructions[instructionCount], 0);
        instructionCount++;

        // Add a stop instruction
        instructions[instructionCount].instructionType = STOP;
        instructions[instructionCount].instructionValue = 0.0;
        xQueueSendToBack(InstructionQueue, &instructions[instructionCount], 0);
        instructionCount++;

        vTaskSuspend(NULL);
	}

}
