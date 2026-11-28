#include "tmc2209_stepper.h"
#include "config.h"
#include "system_state.h"
#include <stdio.h>

#define DIR_TOWARDS_HOME    (-1)
#define DIR_AWAY_FROM_HOME  (1)

// ============================================================================
// LOW-LEVEL HARDWARE HELPERS (5 uniwersalnych funkcji zamiast powtarzania HAL)
// ============================================================================

static inline void enable_driver(StepperMotor_t* motor, bool enable)
{
    HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline void set_direction(StepperMotor_t* motor, int8_t dir)
{
    motor->direction = dir;
    HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, (dir == DIR_AWAY_FROM_HOME) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline void pulse_step_high(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
}

static inline bool is_limit_switch_pressed(StepperMotor_t* motor)
{
    return (motor->Limit_SwitchPort != NULL) &&
           (HAL_GPIO_ReadPin(motor->Limit_SwitchPort, motor->Limit_SwitchPin) == GPIO_PIN_RESET);
}

static inline bool is_step_tick_ready(StepperMotor_t* motor)
{
    motor->stepTickCounter++;
    if (motor->stepTickCounter < motor->stepDivider)
    {
        return false;
    }
    motor->stepTickCounter = 0;
    return true;
}

static inline void finish_at_home_position(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
    motor->currentPosition = 0;
    motor->targetPosition = 0;
    motor->stepsToMove = 0;
    motor->isHomed = true;
    motor->currentState = STATE_IDLE;
}

// ============================================================================
// PUBLIC API
// ============================================================================

void Stepper_Init(StepperMotor_t* motor, GPIO_TypeDef* stepPort, uint16_t stepPin, GPIO_TypeDef* dirPort, uint16_t dirPin, GPIO_TypeDef* enPort, uint16_t enPin)
{
    motor->StepPort = stepPort;
    motor->StepPin = stepPin;
    motor->DirPort = dirPort;
    motor->DirPin = dirPin;
    motor->EnPort = enPort;
    motor->EnPin = enPin;

    motor->Limit_SwitchPort = NULL;
    motor->Limit_SwitchPin = 0;

    motor->currentState = STATE_IDLE;
    motor->isHomed = false;
    motor->isTeachInDone = false;
    motor->stepDivider = 1;
    motor->stepTickCounter = 0;
    motor->teachInStepsCount = 0;
    motor->currentPosition = 0;
    motor->targetPosition = 0;
    motor->stepsToMove = 0;
    motor->direction = DIR_AWAY_FROM_HOME;

    enable_driver(motor, true);
}

void Stepper_SetSpeedDivider(StepperMotor_t* motor, uint8_t divider)
{
    motor->stepDivider = (divider > 0) ? divider : 1;
    motor->stepTickCounter = 0;
}

void Limit_Switch_Init(StepperMotor_t* motor, GPIO_TypeDef* Limit_SwitchPort, uint16_t Limit_SwitchPin)
{
    motor->Limit_SwitchPort = Limit_SwitchPort;
    motor->Limit_SwitchPin = Limit_SwitchPin;

    if (Limit_SwitchPort != NULL)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = Limit_SwitchPin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(Limit_SwitchPort, &GPIO_InitStruct);
    }
}

void Stepper_OnLimitSwitchTriggered(StepperMotor_t* motor)
{
    if (!is_limit_switch_pressed(motor)) return;

    if (motor->currentState == STATE_HOMING)
    {
        finish_at_home_position(motor);
    }
    else if (motor->currentState == STATE_TEACH_IN_RETURN)
    {
        finish_at_home_position(motor);
        motor->isTeachInDone = true;
    }
    else if (motor->currentState == STATE_MOVING && motor->direction == DIR_TOWARDS_HOME)
    {
        finish_at_home_position(motor);
    }
}

void Stepper_StartHoming(StepperMotor_t* motor)
{
    motor->isHomed = false;
    motor->stepTickCounter = 0;
    enable_driver(motor, true);
    set_direction(motor, DIR_TOWARDS_HOME);
    motor->currentState = STATE_HOMING;
}

void Stepper_Stop(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
    motor->stepsToMove = 0;
    motor->currentState = STATE_IDLE;
}

void Stepper_PullStepLow(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
}

void Stepper_MoveTo(StepperMotor_t* motor, int32_t targetSteps)
{
    motor->currentState = STATE_IDLE;
    motor->stepTickCounter = 0;
    motor->targetPosition = targetSteps;

    int32_t delta = targetSteps - motor->currentPosition;
    if (delta == 0)
    {
        motor->stepsToMove = 0;
        return;
    }

    enable_driver(motor, true);
    set_direction(motor, (delta > 0) ? DIR_AWAY_FROM_HOME : DIR_TOWARDS_HOME);
    motor->stepsToMove = (delta > 0) ? delta : -delta;
    motor->currentState = STATE_MOVING;
}

void Stepper_EnableTeachIn(StepperMotor_t* motor)
{
    enable_driver(motor, false);
    motor->currentState = STATE_TEACH_IN;
}

void Stepper_DisableTeachIn(StepperMotor_t* motor)
{
    enable_driver(motor, true);
    motor->currentState = STATE_IDLE;
}

void Stepper_StartTeachInReturn(StepperMotor_t* motor)
{
    motor->teachInStepsCount = 0;
    motor->isTeachInDone = false;
    motor->isHomed = false;
    motor->stepTickCounter = 0;
    enable_driver(motor, true);
    set_direction(motor, DIR_TOWARDS_HOME);
    motor->currentState = STATE_TEACH_IN_RETURN;
}

static inline bool are_all_motors_idle(void)
{
    return (motorJ1.currentState == STATE_IDLE) &&
           (motorJ2.currentState == STATE_IDLE) &&
           (motorZ.currentState  == STATE_IDLE);
}

void Stepper_CheckSoftLimits(void)
{
    if (motorJ1.currentPosition > J1_MAX_STEPS && motorJ1.direction > 0)
    {
        Stepper_Stop(&motorJ1);
    }
    if (motorJ2.currentPosition > J2_MAX_STEPS && motorJ2.direction > 0)
    {
        Stepper_Stop(&motorJ2);
    }
    if (motorZ.currentPosition > Z_MAX_STEPS && motorZ.direction > 0)
    {
        Stepper_Stop(&motorZ);
    }
}

void Stepper_HandleHomingSequence(void)
{
    // Etap 2 bazowania: gdy os Z skonczy bazowanie, uruchom bazowanie osi J1 i J2
    if (pendingHomeJ1J2 && motorZ.isHomed && motorZ.currentState == STATE_IDLE)
    {
        pendingHomeJ1J2 = false;
        Stepper_StartHoming(&motorJ1);
        Stepper_StartHoming(&motorJ2);
    }

    // Gdy wszystkie 3 osie sa zbazowane i stoja w IDLE -> ustaw flage SystemState_SetHomed(true)
    if (!SystemState_IsHomed() && motorJ1.isHomed && motorJ2.isHomed && motorZ.isHomed && are_all_motors_idle())
    {
        SystemState_SetHomed(true);
        printf("[HOMING] All axes homed! Position set to (0, 0, 0)\r\n");
    }
}

void Stepper_HandleTeachInSequence(void)
{
    // Etap 2 powrotu Teach-In: gdy os Z skonczy powrot, uruchom powrot osi J1 i J2
    if (pendingTeachReturnJ1J2 && motorZ.isTeachInDone && motorZ.currentState == STATE_IDLE)
    {
        pendingTeachReturnJ1J2 = false;
        Stepper_StartTeachInReturn(&motorJ1);
        Stepper_StartTeachInReturn(&motorJ2);
    }

    // Gdy wszystkie 3 osie policzyly kroki do krancowek -> zapisz wynik i wyzeruj liczniki
    if (motorJ1.isTeachInDone && motorJ2.isTeachInDone && motorZ.isTeachInDone && are_all_motors_idle())
    {
        SystemState_SetTeachInSteps(motorJ1.teachInStepsCount,
                                    motorJ2.teachInStepsCount,
                                    motorZ.teachInStepsCount);

        motorJ1.teachInStepsCount = 0;
        motorJ2.teachInStepsCount = 0;
        motorZ.teachInStepsCount  = 0;

        motorJ1.isTeachInDone = false;
        motorJ2.isTeachInDone = false;
        motorZ.isTeachInDone  = false;
    }
}

void Stepper_SyncPositionToSystemState(void)
{
    SystemState_UpdateSteps(motorJ1.currentPosition, motorJ2.currentPosition, motorZ.currentPosition);
}

void Stepper_CheckMotionCompletedAck(void)
{
    if (!pendingHomeJ1J2 && !pendingTeachReturnJ1J2 && are_all_motors_idle())
    {
        SystemState_MarkMotionAckCompleted();
    }
}

// ============================================================================
// STATE MACHINE (Czytelna jak zdanie po angielsku)
// ============================================================================

void Stepper_UpdateStateMachine(StepperMotor_t* motor)
{
    switch (motor->currentState)
    {
        case STATE_IDLE:
            enable_driver(motor, true);
            break;

        case STATE_TEACH_IN:
            enable_driver(motor, false);
            break;

        case STATE_MOVING:
            if (!is_step_tick_ready(motor)) return;

            if (is_limit_switch_pressed(motor) && motor->direction == DIR_TOWARDS_HOME)
            {
                finish_at_home_position(motor);
                break;
            }

            if (motor->stepsToMove > 0)
            {
                pulse_step_high(motor);
                motor->currentPosition += motor->direction;
                motor->stepsToMove--;
            }
            else
            {
                motor->currentState = STATE_IDLE;
            }
            break;

        case STATE_HOMING:
            if (!is_step_tick_ready(motor)) return;

            if (!is_limit_switch_pressed(motor))
            {
                pulse_step_high(motor);
            }
            else
            {
                finish_at_home_position(motor);
            }
            break;

        case STATE_TEACH_IN_RETURN:
            if (!is_step_tick_ready(motor)) return;

            if (!is_limit_switch_pressed(motor))
            {
                pulse_step_high(motor);
                motor->teachInStepsCount++;
            }
            else
            {
                finish_at_home_position(motor);
                motor->isTeachInDone = true;
            }
            break;
    }
}