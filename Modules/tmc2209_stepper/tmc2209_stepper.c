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
    motor->maxSteps = 0;
    motor->invertDir = false;
    motor->isHomed = false;
    motor->endstopDebounceCount = 0;

    motor->currentState = STATE_IDLE;
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

void Stepper_SetInvertDir(StepperMotor_t* motor, bool invert)
{
    motor->invertDir = invert;
}

void Stepper_SetMaxSteps(StepperMotor_t* motor, int32_t maxSteps)
{
    motor->maxSteps = maxSteps;
}

static void Stepper_ApplyDirPin(StepperMotor_t* motor, int8_t dir)
{
    // dir: +1 = ruch na plus (od krancowki w przestrzen robocza)
    //      -1 = ruch na minus (do krancowki / bazy)
    // Na tej maszynie fizycznie:
    // +1 (od krancowki) -> GPIO_PIN_RESET
    // -1 (do krancowki)  -> GPIO_PIN_SET
    GPIO_PinState pinState;
    if (dir > 0) {
        pinState = motor->invertDir ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else {
        pinState = motor->invertDir ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, pinState);
}

static bool Stepper_IsEndstopPressed(StepperMotor_t* motor)
{
    if (motor->EndstopPort == NULL) return false;

    if (HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_RESET)
    {
        if (motor->endstopDebounceCount < 5)
        {
            motor->endstopDebounceCount++;
        }
    }
    else
    {
        motor->endstopDebounceCount = 0;
    }

    return (motor->endstopDebounceCount >= 5);
}

void Stepper_StartHoming(StepperMotor_t* motor)
{
    if (motor == NULL) return;
    if (motor->currentState == STATE_HOMING) return;

    // Jesli os juz stoi na wcisnietej krancowce, od razu zerujemy i konczymy bazowanie
    if (motor->EndstopPort != NULL && HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_RESET)
    {
        motor->endstopDebounceCount = 5;
        motor->currentState = STATE_IDLE;
        motor->stepsToMove = 0;
        motor->currentPosition = 0;
        motor->targetPosition = 0;
        motor->isHomed = true;
        return;
    }

    motor->currentState = STATE_IDLE;
    motor->endstopDebounceCount = 0;
    motor->isHomed = false;
    motor->stepsToMove = 0;
    motor->direction = -1;
    Stepper_ApplyDirPin(motor, -1);
    motor->currentState = STATE_HOMING;
}

void Stepper_Stop(StepperMotor_t* motor)
{
    if (motor == NULL) return;
    motor->currentState = STATE_IDLE;
    motor->stepsToMove = 0;
}

void Stepper_OnEndstopTriggered(StepperMotor_t* motor)
{
    if (motor == NULL) return;

    if (motor->currentState == STATE_HOMING)
    {
        motor->currentState = STATE_IDLE;
        motor->stepsToMove = 0;
        motor->currentPosition = 0;
        motor->targetPosition = 0;
        motor->isHomed = true;
    }
    else if (motor->currentState == STATE_MOVING)
    {
        // Zatrzymaj i wyzeruj os TYLKO jesli jechala w strone krancowki (direction == -1).
        // Gdy odjezdza z krancowki na plus (direction == 1), drganie puszczanej blaszki jest ignorowane!
        if (motor->direction == -1)
        {
            motor->currentState = STATE_IDLE;
            motor->stepsToMove = 0;
            motor->currentPosition = 0;
            motor->targetPosition = 0;
            motor->isHomed = true;
        }
    }
}

void Stepper_MoveTo(StepperMotor_t* motor, int32_t targetSteps)
{
    if (motor == NULL) return;

    // Ograniczenie programowe gornego zakresu (Soft Limit) od krancowki
    if (motor->maxSteps > 0 && targetSteps > motor->maxSteps)
    {
        targetSteps = motor->maxSteps;
    }

    // Jesli krancowka jest fizycznie wcisnieta, zablokuj dalszy ruch w jej strone, ale pozwol odjechac
    if (motor->EndstopPort != NULL && HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_RESET)
    {
        if (motor->currentPosition < 0)
        {
            motor->currentPosition = 0;
        }

        if (targetSteps <= motor->currentPosition)
        {
            motor->currentState = STATE_IDLE;
            motor->stepsToMove = 0;
            return;
        }
    }

    // Bezpieczne zatrzymanie maszyny stanow na czas aktualizacji parametrow (ochrona przed wywlaszczeniem przez MotorTask)
    motor->currentState = STATE_IDLE;
    motor->targetPosition = targetSteps;
    int32_t delta = targetSteps - motor->currentPosition;

    if (delta > 0) {
        motor->direction = 1;
        motor->stepsToMove = delta;
    } else if (delta < 0) {
        motor->direction = -1;
        motor->stepsToMove = -delta;
    } else {
        motor->stepsToMove = 0;
        return;
    }

    Stepper_ApplyDirPin(motor, motor->direction);
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
            Stepper_ApplyDirPin(motor, motor->direction);

            // Jesli jedzie w strone krancowki i jest ona wcisnieta (z filtrem 5 ms) -> zatrzymaj tylko te os
            if (motor->direction == -1 && Stepper_IsEndstopPressed(motor))
            {
                motor->currentState = STATE_IDLE;
                motor->stepsToMove = 0;
                motor->currentPosition = 0;
                motor->targetPosition = 0;
                motor->isHomed = true;
                break;
            }
            else if (motor->direction == 1)
            {
                motor->endstopDebounceCount = 0;
            }

            // Ochrona gornego limitu krokow (Soft Limit)
            if (motor->direction == 1 && motor->maxSteps > 0 && motor->currentPosition >= motor->maxSteps)
            {
                motor->currentState = STATE_IDLE;
                motor->stepsToMove = 0;
                break;
            }

            if (motor->stepsToMove > 0)
            {
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
                for (volatile int i = 0; i < 50; i++);
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);

                if (motor->currentState == STATE_MOVING)
                {
                    motor->currentPosition += motor->direction;
                    motor->stepsToMove--;
                    if (motor->stepsToMove <= 0)
                    {
                        motor->currentState = STATE_IDLE;
                    }
                }
            }
            else
            {
                motor->currentState = STATE_IDLE;
            }
            break;

        case STATE_HOMING:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            motor->direction = -1;
            Stepper_ApplyDirPin(motor, -1); // Kierunek do bazy (krańcówki)

            if (Stepper_IsEndstopPressed(motor))
            {
                // Krancowka osiagnieta
                motor->currentState = STATE_IDLE;
                motor->currentPosition = 0;
                motor->targetPosition = 0;
                motor->stepsToMove = 0;
                motor->isHomed = true;
            }
            else
            {
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_SET);
                for (volatile int i = 0; i < 50; i++);
                HAL_GPIO_WritePin(motor->StepPort, motor->StepPin, GPIO_PIN_RESET);
            }
            break;

        case STATE_TEACH_IN:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_SET);
            break;

        case STATE_TEACH_IN_RETURN:
            
            HAL_GPIO_WritePin(motor->EnPort, motor->EnPin, GPIO_PIN_RESET);
            
            HAL_GPIO_WritePin(motor->DirPort, motor->DirPin, GPIO_PIN_RESET);

            if (motor->EndstopPort != NULL && HAL_GPIO_ReadPin(motor->EndstopPort, motor->EndstopPin) == GPIO_PIN_RESET)
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