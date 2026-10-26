#include "tmc2209_stepper.h"

void Stepper_Init(StepperMotor_t* motor, GPIO_TypeDef* stepPort, uint16_t stepPin, GPIO_TypeDef* dirPort, uint16_t dirPin, GPIO_TypeDef* enPort, uint16_t enPin)
{
    motor->StepPort = stepPort;
    motor->StepPin = stepPin;
    motor->DirPort = dirPort;
    motor->DirPin = dirPin;
    motor->EnPort = enPort;
    motor->EnPin = enPin;

    motor->currentState = STATE_IDLE;
    motor->currentPosition = 0;
    motor->targetPosition = 0;
    motor->stepsToMove = 0;
    motor->direction = 1;

    HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
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
            HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_RESET); // Kierunek do bazy

            if (HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_RESET)
            {
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
                for (volatile int i = 0; i < 10; i++);
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
            }
            else
            {
                motor->currentPosition = 0;
                motor->currentState = STATE_IDLE;
            }
            break;

        case STATE_TEACH_IN:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_SET);
            break;

        case STATE_TEACH_IN_RETURN:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_RESET);

            if (HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_RESET)
            {
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
                for (volatile int i = 0; i < 10; i++);
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);

                motor->teachInStepsCount++;
            }
            else
            {
                motor->currentPosition = 0;
                motor->currentState = STATE_IDLE;
            }
            break;
        
    }
}