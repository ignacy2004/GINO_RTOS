#include "system_state.h"
#include "FreeRTOS.h"
#include "semphr.h"

static RobotState_t globalRobotState;
static SemaphoreHandle_t stateMutex = NULL;

void SystemState_Init(void)
{
    stateMutex = xSemaphoreCreateMutex();
    globalRobotState.currentCartesian.x = 0.0f;
    globalRobotState.currentCartesian.y = 0.0f;
    globalRobotState.currentCartesian.z = 0.0f;

    globalRobotState.currentSteps.j1_steps = 0;
    globalRobotState.currentSteps.j2_steps = 0;
    globalRobotState.currentSteps.j3_steps = 0;

    globalRobotState.isHomed = false;
    globalRobotState.pumpState = false;
    globalRobotState.emergencyStop = false;
}

void SystemState_Get(RobotState_t* outState)
{
    if (stateMutex != NULL && outState != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            *outState = globalRobotState;
            xSemaphoreGive(stateMutex);
        }
    }
}

void SystemState_UpdateSteps(int32_t j1, int32_t j2, int32_t j3)
{
    globalRobotState.currentSteps.j1_steps = j1;
    globalRobotState.currentSteps.j2_steps = j2;
    globalRobotState.currentSteps.j3_steps = j3;
}

void SystemState_UpdateCartesian(float x, float y, float z)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.currentCartesian.x = x;
            globalRobotState.currentCartesian.y = y;
            globalRobotState.currentCartesian.z = z;
            xSemaphoreGive(stateMutex);
        }
    }
}

void SystemState_SetPump(bool enable)
{
    globalRobotState.pumpState = enable;
}

void SystemState_SetEstop(bool estop)
{
    globalRobotState.emergencyStop = estop;
}

void SystemState_SetEstopFromISR(bool estop)
{
    globalRobotState.emergencyStop = estop;
}

bool SystemState_IsEstop(void)
{
    return globalRobotState.emergencyStop;
}

void SystemState_SetHomed(bool homed)
{
    globalRobotState.isHomed = homed;
}

bool SystemState_IsHomed(void)
{
    return globalRobotState.isHomed;
}