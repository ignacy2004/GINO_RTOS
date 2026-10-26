#ifndef TMC2209_STEPPER_H
#define TMC2209_STEPPER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STATE_IDLE,
    STATE_MOVING,
    STATE_HOMING,
    STATE_TEACH_IN,
    STATE_TEACH_IN_RETURN
} StepperState_t;

typedef struct {

    GPIO_TypeDef* StepPort;
    uint16_t      StepPin;
    GPIO_TypeDef* DirPort;
    uint16_t      DirPin;
    GPIO_TypeDef* EnPort;
    uint16_t      EnPin;
    GPIO_TypeDef* EndstopPort;
    uint16_t      EndstopPin;
    
    StepperState_t currentState;
    int32_t       teachInStepsCount;
    int32_t currentPosition;
    int32_t targetPosition;
    int32_t stepsToMove;
    int8_t direction;
} StepperMotor_t;

void Stepper_Init(StepperMotor_t* motor, GPIO_TypeDef* stepPort, uint16_t stepPin, GPIO_TypeDef* dirPort, uint16_t dirPin, GPIO_TypeDef* enPort, uint16_t enPin);
void Stepper_MoveTo(StepperMotor_t* motor, int32_t targetSteps);
void Stepper_EnableTeachIn(StepperMotor_t* motor);
void Stepper_UpdateStateMachine(StepperMotor_t* motor);

#endif