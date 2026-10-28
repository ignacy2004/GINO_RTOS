#include "tool_controller.h"
#include "system_state.h"

static GPIO_TypeDef* hardwarePumpPort = NULL;
static uint16_t hardwarePumpPin = 0;
static bool isPumpActive = false;

void ToolController_Init(GPIO_TypeDef* pumpPort, uint16_t pumpPin)
{
    hardwarePumpPort = pumpPort;
    hardwarePumpPin = pumpPin;
    isPumpActive = false;

    if (hardwarePumpPort != NULL)
    {
        HAL_GPIO_WritePin(hardwarePumpPort, hardwarePumpPin, GPIO_PIN_RESET);
    }
    
    SystemState_SetPump(false);
}

void ToolController_EnablePump(bool enable)
{
    isPumpActive = enable;
    
    if (hardwarePumpPort != NULL)
    {
        HAL_GPIO_WritePin(hardwarePumpPort, hardwarePumpPin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    SystemState_SetPump(enable);
}

bool ToolController_IsPumpEnabled(void)
{
    return isPumpActive;
}