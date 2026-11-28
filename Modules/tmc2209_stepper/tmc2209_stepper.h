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
    GPIO_TypeDef* Limit_SwitchPort;
    uint16_t      Limit_SwitchPin;
    
    StepperState_t currentState;
    bool           isHomed;
    bool           isTeachInDone;
    uint8_t        stepDivider;
    uint8_t        stepTickCounter;
    int32_t teachInStepsCount;
    int32_t currentPosition;
    int32_t targetPosition;
    int32_t stepsToMove;
    int8_t direction;
} StepperMotor_t;

extern StepperMotor_t motorJ1;
extern StepperMotor_t motorJ2;
extern StepperMotor_t motorZ;

extern volatile bool pendingHomeJ1J2;
extern volatile bool pendingTeachReturnJ1J2;

void Stepper_Init(StepperMotor_t* motor, GPIO_TypeDef* stepPort, uint16_t stepPin, GPIO_TypeDef* dirPort, uint16_t dirPin, GPIO_TypeDef* enPort, uint16_t enPin);
void Stepper_SetSpeedDivider(StepperMotor_t* motor, uint8_t divider);
void Limit_Switch_Init(StepperMotor_t* motor, GPIO_TypeDef* Limit_SwitchPort, uint16_t Limit_SwitchPin);
void Stepper_MoveTo(StepperMotor_t* motor, int32_t targetSteps);
void Stepper_EnableTeachIn(StepperMotor_t* motor);
void Stepper_DisableTeachIn(StepperMotor_t* motor);
void Stepper_StartTeachInReturn(StepperMotor_t* motor);
void Stepper_StartHoming(StepperMotor_t* motor);
void Stepper_Stop(StepperMotor_t* motor);
void Stepper_PullStepLow(StepperMotor_t* motor);
void Stepper_OnLimitSwitchTriggered(StepperMotor_t* motor);
void Stepper_CheckSoftLimits(void);
void Stepper_HandleHomingSequence(void);
void Stepper_HandleTeachInSequence(void);
void Stepper_SyncPositionToSystemState(void);
void Stepper_CheckMotionCompletedAck(void);
void Stepper_UpdateStateMachine(StepperMotor_t* motor);

#endif