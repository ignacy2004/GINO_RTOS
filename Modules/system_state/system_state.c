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

    globalRobotState.teachInSteps.j1_steps = 0;
    globalRobotState.teachInSteps.j2_steps = 0;
    globalRobotState.teachInSteps.j3_steps = 0;
    globalRobotState.teachInReady = false;
    globalRobotState.pendingMotionAckCmd = 0;
    globalRobotState.completedMotionAckCmd = 0;
    globalRobotState.pendingDelayMs = 0;
    globalRobotState.pendingDelayAckCmd = 0;

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
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.currentSteps.j1_steps = j1;
            globalRobotState.currentSteps.j2_steps = j2;
            globalRobotState.currentSteps.j3_steps = j3;
            xSemaphoreGive(stateMutex);
        }
    }
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

void SystemState_SetTeachInSteps(int32_t j1, int32_t j2, int32_t j3)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.teachInSteps.j1_steps = j1;
            globalRobotState.teachInSteps.j2_steps = j2;
            globalRobotState.teachInSteps.j3_steps = j3;
            globalRobotState.teachInReady = true;
            xSemaphoreGive(stateMutex);
        }
    }
}

bool SystemState_ConsumeTeachInSteps(JointSteps_t* outSteps)
{
    bool ready = false;
    if (stateMutex != NULL && outSteps != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            if (globalRobotState.teachInReady)
            {
                *outSteps = globalRobotState.teachInSteps;
                globalRobotState.teachInReady = false;
                ready = true;
            }
            xSemaphoreGive(stateMutex);
        }
    }
    return ready;
}

void SystemState_SetPendingMotionAck(uint8_t cmd)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.pendingMotionAckCmd = cmd;
            globalRobotState.completedMotionAckCmd = 0;
            xSemaphoreGive(stateMutex);
        }
    }
}

void SystemState_MarkMotionAckCompleted(void)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            if (globalRobotState.pendingMotionAckCmd != 0)
            {
                globalRobotState.completedMotionAckCmd = globalRobotState.pendingMotionAckCmd;
                globalRobotState.pendingMotionAckCmd = 0;
            }
            xSemaphoreGive(stateMutex);
        }
    }
}

bool SystemState_ConsumeCompletedMotionAck(uint8_t* outCmd)
{
    bool completed = false;
    if (stateMutex != NULL && outCmd != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            if (globalRobotState.completedMotionAckCmd != 0)
            {
                *outCmd = globalRobotState.completedMotionAckCmd;
                globalRobotState.completedMotionAckCmd = 0;
                completed = true;
            }
            xSemaphoreGive(stateMutex);
        }
    }
    return completed;
}

void SystemState_SetPendingDelay(uint8_t cmd, uint32_t delayMs)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.pendingDelayAckCmd = cmd;
            globalRobotState.pendingDelayMs = delayMs;
            xSemaphoreGive(stateMutex);
        }
    }
}

bool SystemState_ConsumePendingDelay(uint8_t* outCmd, uint32_t* outDelayMs)
{
    bool hasDelay = false;
    if (stateMutex != NULL && outCmd != NULL && outDelayMs != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            if (globalRobotState.pendingDelayAckCmd != 0)
            {
                *outCmd = globalRobotState.pendingDelayAckCmd;
                *outDelayMs = globalRobotState.pendingDelayMs;
                globalRobotState.pendingDelayAckCmd = 0;
                globalRobotState.pendingDelayMs = 0;
                hasDelay = true;
            }
            xSemaphoreGive(stateMutex);
        }
    }
    return hasDelay;
}

void SystemState_SetPump(bool enable)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.pumpState = enable;
            xSemaphoreGive(stateMutex);
        }
    }
}

void SystemState_SetEstop(bool estop)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.emergencyStop = estop;
            xSemaphoreGive(stateMutex);
        }
    }
}

bool SystemState_IsEstop(void)
{
    bool estopVal = false;
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            estopVal = globalRobotState.emergencyStop;
            xSemaphoreGive(stateMutex);
        }
    }
    return estopVal;
}

void SystemState_SetHomed(bool homed)
{
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            globalRobotState.isHomed = homed;
            xSemaphoreGive(stateMutex);
        }
    }
}

bool SystemState_IsHomed(void)
{
    bool homedVal = false;
    if (stateMutex != NULL)
    {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
        {
            homedVal = globalRobotState.isHomed;
            xSemaphoreGive(stateMutex);
        }
    }
    return homedVal;
}