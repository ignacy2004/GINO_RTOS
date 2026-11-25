#include "tmc2209_stepper.h"
#include "config.h"


void Stepper_Init(StepperMotor_t* motor, GPIO_TypeDef* stepPort, uint16_t stepPin, GPIO_TypeDef* dirPort, uint16_t dirPin, GPIO_TypeDef* enPort, uint16_t enPin)
{   //TMC2209 PINS
    motor->StepPort = stepPort;
    motor->StepPin = stepPin;
    motor->DirPort = dirPort;
    motor->DirPin = dirPin;
    motor->EnPort = enPort;
    motor->EnPin = enPin;
    
    //LIMIT SWITCH PINS
    motor->Limit_SwitchPort = NULL;
    motor->Limit_SwitchPin = 0;

    // Initialize state variables
    motor->currentState = STATE_IDLE;
    motor->isHomed = false;
    motor->isTeachInDone = false;
    motor->stepDivider = 1;
    motor->stepTickCounter = 0;
    motor->teachInStepsCount = 0;
    motor->currentPosition = 0;
    motor->targetPosition = 0;
    motor->stepsToMove = 0;
    motor->direction = 1;

    HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
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
}

void Stepper_StartHoming(StepperMotor_t* motor)
{
    motor->direction = -1;
    motor->isHomed = false;
    motor->currentState = STATE_HOMING;
}

void Stepper_Stop(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
    motor->stepsToMove = 0;
    motor->currentState = STATE_IDLE;
}

void Stepper_MoveTo(StepperMotor_t* motor, int32_t targetSteps)
{
    motor->currentState = STATE_IDLE;
    motor->targetPosition = targetSteps;
    int32_t delta = targetSteps - motor->currentPosition;
    if (delta > 0) {
        HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_RESET);
        motor->direction = 1;
        motor->stepsToMove = delta;
    } else if (delta < 0) {
        HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET);
        motor->direction = -1;
        motor->stepsToMove = -delta;
    } else {
        motor->stepsToMove = 0;
        return;
    }
    motor->currentState = STATE_MOVING;
}

void Stepper_EnableTeachIn(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_SET);

    motor->currentState = STATE_TEACH_IN;
}

void Stepper_DisableTeachIn(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);

    motor->currentState = STATE_IDLE;
}

void Stepper_StartTeachInReturn(StepperMotor_t* motor)
{
    motor->teachInStepsCount = 0;
    motor->isTeachInDone = false;
    motor->isHomed = false;
    motor->direction = -1;
    motor->currentState = STATE_TEACH_IN_RETURN;
}

void Stepper_UpdateStateMachine(StepperMotor_t* motor)
{
    if (motor->currentState == STATE_MOVING ||
        motor->currentState == STATE_HOMING ||
        motor->currentState == STATE_TEACH_IN_RETURN)
    {
        motor->stepTickCounter++;
        if (motor->stepTickCounter < motor->stepDivider)
        {
            return;
        }
        motor->stepTickCounter = 0;
    }

    switch (motor->currentState)
    {
        case STATE_IDLE:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            break;

        case STATE_MOVING:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            
            // Krańcówka wciśnięta = GPIO_PIN_RESET (zwarta do GND)
            // Blokuj jazdę w lewo (direction == 1, DIR = SET), pozwól jechać w prawo (direction == -1, DIR = RESET)
            if (limit_switch_pressed(motor)) {
                if (motor->direction == -1)
                {
                    set_axis_homed(motor);
                    break;
                }
            }
            
            if (motor->direction == 1) {
                HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET);
            }

            if (motor->stepsToMove > 0)
            {
                move_one_step(motor);

                motor->currentPosition += motor->direction;
                motor->stepsToMove--;
            }
            else
            {
                motor->currentState = STATE_IDLE;
            }
            break;

        case STATE_HOMING:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET);

            if (limit_switch_not_pressed(motor))
            {
                move_one_step(motor);
            }
            else
            {
                set_axis_homed(motor);
            }
            break;

        case STATE_TEACH_IN:
        
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_SET);    
            break;

        case STATE_TEACH_IN_RETURN:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET);

            if (limit_switch_not_pressed(motor))
            {
                move_one_step(motor);
                motor->teachInStepsCount++;
            }
            else
            {
                set_axis_homed(motor);
                motor->isTeachInDone = true;
            }
            break;
        
    }
}


bool limit_switch_not_pressed(StepperMotor_t* motor) {
    return motor->Limit_SwitchPort != NULL && HAL_GPIO_ReadPin(motor->Limit_SwitchPort, motor->Limit_SwitchPin) == GPIO_PIN_SET;
}

bool limit_switch_pressed(StepperMotor_t* motor) {
    return motor->Limit_SwitchPort != NULL && HAL_GPIO_ReadPin(motor->Limit_SwitchPort, motor->Limit_SwitchPin) == GPIO_PIN_RESET;
}

void set_axis_homed(StepperMotor_t* motor) {
    HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
    motor->currentPosition = 0;
    motor->targetPosition = 0;
    motor->stepsToMove = 0;
    motor->isHomed = true;
    motor->currentState = STATE_IDLE;
}

void move_one_step(StepperMotor_t* motor) {
    // Tylko zbocze narastające (HIGH) - zbocze opadające (LOW) generuje sprzętowy Timer TIM2 w następnym półokresie!
    HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
}