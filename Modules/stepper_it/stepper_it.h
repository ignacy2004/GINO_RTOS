#ifndef STEPPER_IT_H
#define STEPPER_IT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

void StepperTimer_Init(void);
void StepperTimer_ISR(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Stepper_ProcessISRFlags(void);

#endif

