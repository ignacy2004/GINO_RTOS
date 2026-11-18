#include "tmc2209_stepper.h"

void Stepper_Init(StepperMotor_t* motor, GPIO_TypeDef* stepPort, uint16_t stepPin, GPIO_TypeDef* dirPort, uint16_t dirPin, GPIO_TypeDef* enPort, uint16_t enPin)
{
    motor->StepPort = stepPort;
    motor->StepPin = stepPin;
    motor->DirPort = dirPort;
    motor->DirPin = dirPin;
    motor->EnPort = enPort;
    motor->EnPin = enPin;
    motor->EndstopPort = NULL;
    motor->EndstopPin = 0;
    motor->teachInStepsCount = 0;

    motor->currentState = STATE_IDLE;
    motor->isHomed = false;
    motor->currentPosition = 0;
    motor->targetPosition = 0;
    motor->stepsToMove = 0;
    motor->direction = 1;

    HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
}

void Stepper_SetEndstop(StepperMotor_t* motor, GPIO_TypeDef* endstopPort, uint16_t endstopPin)
{
    motor->EndstopPort = endstopPort;
    motor->EndstopPin = endstopPin;
}

void Stepper_StartHoming(StepperMotor_t* motor)
{
    motor->isHomed = false;
    motor->currentState = STATE_HOMING;
}

void Stepper_Stop(StepperMotor_t* motor)
{
    motor->stepsToMove = 0;
    motor->currentState = STATE_IDLE;
}

void Stepper_MoveTo(StepperMotor_t* motor, int32_t targetSteps)
{
    motor->targetPosition = targetSteps;
    motor->stepsToMove = targetSteps - motor->currentPosition;

    if (motor->stepsToMove > 0) {
        HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET);
        motor->direction = 1;
    } else {
        HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_RESET);
        motor->direction = -1;
        motor->stepsToMove = -motor->stepsToMove;
    }

    motor->currentState = STATE_MOVING;
}

void Stepper_EnableTeachIn(StepperMotor_t* motor)
{
    HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_SET);

    motor->currentState = STATE_TEACH_IN;
}

void Stepper_UpdateStateMachine(StepperMotor_t* motor)
{
    switch (motor->currentState)
    {
        case STATE_IDLE:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            break;

        case STATE_MOVING:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);

            // Krańcówka wciśnięta = GPIO_PIN_RESET (zwarta do GND)
            // Blokuj jazdę w lewo (direction == 1, DIR = SET), pozwól jechać w prawo (direction == -1, DIR = RESET)
            if (motor->EndstopPort != NULL && HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_RESET)
            {
                if (motor->direction == 1)
                {
                    motor->stepsToMove = 0;
                    motor->currentPosition = 0;
                    motor->targetPosition = 0;
                    motor->currentState = STATE_IDLE;
                    break;
                }
            }
            
            if (motor->direction == 1) {
                HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_RESET);
            }

            if (motor->stepsToMove > 0)
            {
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
                for (volatile int i = 0; i < 10; i++);
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);

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
            HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET); // Kierunek do bazy (w lewo, direction == 1)

            // Dopóki krańcówka NIE jest wciśnięta (GPIO_PIN_SET z Pull-Up), jedź do bazy
            if (motor->EndstopPort != NULL && HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
                for (volatile int i = 0; i < 10; i++);
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
            }
            else
            {
                // Krańcówka wciśnięta (GPIO_PIN_RESET) -> STOP i pozycja = 0
                motor->currentPosition = 0;
                motor->targetPosition = 0;
                motor->stepsToMove = 0;
                motor->isHomed = true;
                motor->currentState = STATE_IDLE;
            }
            break;

        case STATE_TEACH_IN:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_SET);
            break;

        case STATE_TEACH_IN_RETURN:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_SET);

            if (motor->EndstopPort != NULL && HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
                for (volatile int i = 0; i < 10; i++);
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);

                motor->teachInStepsCount++;
            }
            else
            {
                motor->currentPosition = 0;
                motor->targetPosition = 0;
                motor->stepsToMove = 0;
                motor->currentState = STATE_IDLE;
            }
            break;
        
    }
}