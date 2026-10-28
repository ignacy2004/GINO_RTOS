#ifndef TOOL_CONTROLLER_H
#define TOOL_CONTROLLER_H

#include "main.h"
#include <stdbool.h>

void ToolController_Init(GPIO_TypeDef* pumpPort, uint16_t pumpPin);
void ToolController_EnablePump(bool enable);
bool ToolController_IsPumpEnabled(void);

#endif