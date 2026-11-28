#include "stepper_it.h"
#include "tmc2209_stepper.h"
#include "config.h"

static volatile bool stepPinHighPhase = false;
static volatile bool stepperTickFlag  = false;
static volatile bool limitJ1Flag      = false;
static volatile bool limitJ2Flag      = false;
static volatile bool limitZFlag       = false;

void StepperTimer_Init(void)
{
    // Konfiguracja przerwan zewnetrznych EXTI dla krancowek:
    // J1_LIMIT (PA3 -> EXTI3), J2_LIMIT (PA7 -> EXTI9_5), Z_LIMIT (PB12 -> EXTI15_10)
    HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    // Konfiguracja Timera TIM2 do generacji impulsow STEP (3000 Hz baza / divider)
    __HAL_RCC_TIM2_CLK_ENABLE();

    // APB1 Timer Clock = 84 MHz. Prescaler = 84 - 1 -> Timer tick = 1 MHz (1 us)
    TIM2->PSC  = 84U - 1U;
    // Timer wywoluje przerwanie 2x na okres kroku najszybszej osi (faza HIGH i faza LOW)
    TIM2->ARR  = (1000000U / (STEPPER_BASE_FREQ_HZ * 2U)) - 1U;
    TIM2->CNT  = 0U;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0U;
    TIM2->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(TIM2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

void StepperTimer_ISR(void)
{
    stepperTickFlag = true;
    Stepper_ProcessISRFlags();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == J1_LIMIT_Pin)
    {
        limitJ1Flag = true;
    }
    else if (GPIO_Pin == J2_LIMIT_Pin)
    {
        limitJ2Flag = true;
    }
    else if (GPIO_Pin == Z_LIMIT_Pin)
    {
        limitZFlag = true;
    }
    Stepper_ProcessISRFlags();
}

void Stepper_ProcessISRFlags(void)
{
    if (limitJ1Flag)
    {
        limitJ1Flag = false;
        Stepper_OnLimitSwitchTriggered(&motorJ1);
    }
    if (limitJ2Flag)
    {
        limitJ2Flag = false;
        Stepper_OnLimitSwitchTriggered(&motorJ2);
    }
    if (limitZFlag)
    {
        limitZFlag = false;
        Stepper_OnLimitSwitchTriggered(&motorZ);
    }

    if (stepperTickFlag)
    {
        stepperTickFlag = false;

        if (!stepPinHighPhase)
        {
            Stepper_UpdateStateMachine(&motorJ1);
            Stepper_UpdateStateMachine(&motorJ2);
            Stepper_UpdateStateMachine(&motorZ);
            stepPinHighPhase = true;
        }
        else
        {
            Stepper_PullStepLow(&motorJ1);
            Stepper_PullStepLow(&motorJ2);
            Stepper_PullStepLow(&motorZ);
            stepPinHighPhase = false;
        }
    }
}

