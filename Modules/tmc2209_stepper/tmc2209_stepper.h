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
    
    volatile StepperState_t currentState;
    int32_t teachInStepsCount;
    volatile int32_t currentPosition;
    volatile int32_t targetPosition;
    volatile int32_t stepsToMove;
    volatile int8_t direction;
    volatile bool isHomed;
    uint8_t endstopDebounceCount;
    int32_t maxSteps;
    bool invertDir;
} StepperMotor_t;

void Stepper_Init(StepperMotor_t* motor, GPIO_TypeDef* stepPort, uint16_t stepPin, GPIO_TypeDef* dirPort, uint16_t dirPin, GPIO_TypeDef* enPort, uint16_t enPin);
void Stepper_SetEndstop(StepperMotor_t* motor, GPIO_TypeDef* endstopPort, uint16_t endstopPin);
void Stepper_SetInvertDir(StepperMotor_t* motor, bool invert);
void Stepper_SetMaxSteps(StepperMotor_t* motor, int32_t maxSteps);
void Stepper_MoveTo(StepperMotor_t* motor, int32_t targetSteps);
void Stepper_EnableTeachIn(StepperMotor_t* motor);
void Stepper_StartHoming(StepperMotor_t* motor);
void Stepper_Stop(StepperMotor_t* motor);
void Stepper_OnEndstopTriggered(StepperMotor_t* motor);
void Stepper_UpdateStateMachine(StepperMotor_t* motor);

#endif