/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_state.h"
#include "tmc2209_stepper.h"
#include "tool_controller.h"
#include "serial_interface.h"
#include "dispatcher.h"
#include "inverse_kinematics.h"
#include <stdio.h>
#include "config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
StepperMotor_t motorJ1;
StepperMotor_t motorJ2;
StepperMotor_t motorZ;

static volatile bool stepPinHighPhase = false;
static volatile bool pendingHomeJ1J2 = false;
static volatile bool pendingTeachReturnJ1J2 = false;

static void StepperTimer_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    // APB1 Timer Clock = 84 MHz. Prescaler = 84 - 1 -> Timer tick = 1 MHz (1 us)
    TIM2->PSC = 84U - 1U;
    // Timer wywołuje przerwanie 2x na okres kroku najszybszej osi (faza HIGH i faza LOW)
    TIM2->ARR = (1000000U / (STEPPER_BASE_FREQ_HZ * 2U)) - 1U;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR  = 0U;
    TIM2->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(TIM2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

void StepperTimer_ISR(void)
{
    if (!stepPinHighPhase)
    {
        // Faza 1: Aktualizacja maszyny stanów i wystawienie zbocza narastającego (STEP = HIGH)
        Stepper_UpdateStateMachine(&motorJ1);
        Stepper_UpdateStateMachine(&motorJ2);
        Stepper_UpdateStateMachine(&motorZ);

        // Sekwencja bazowania: gdy oś Z skończy bazowanie, uruchom bazowanie osi J1 i J2
        if (pendingHomeJ1J2 && motorZ.isHomed && motorZ.currentState == STATE_IDLE)
        {
            pendingHomeJ1J2 = false;
            Stepper_StartHoming(&motorJ1);
            Stepper_StartHoming(&motorJ2);
        }

        // Sekwencja powrotu Teach-In: gdy oś Z skończy powrót, uruchom powrót osi J1 i J2
        if (pendingTeachReturnJ1J2 && motorZ.isTeachInDone && motorZ.currentState == STATE_IDLE)
        {
            pendingTeachReturnJ1J2 = false;
            Stepper_StartTeachInReturn(&motorJ1);
            Stepper_StartTeachInReturn(&motorJ2);
        }

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

        stepPinHighPhase = true;
    }
    else
    {
        // Faza 2: Zbocze opadające (STEP = LOW) na wszystkich osiach po połowie okresu timera
        HAL_GPIO_WritePin(motorJ1.StepPort, motorJ1.StepPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motorJ2.StepPort, motorJ2.StepPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motorZ.StepPort,  motorZ.StepPin,  GPIO_PIN_RESET);

        stepPinHighPhase = false;
    }
}

void Dispatcher_MoveJoints(int32_t j1_steps, int32_t j2_steps, int32_t j3_steps)
{
    __disable_irq();
    pendingHomeJ1J2 = false;
    pendingTeachReturnJ1J2 = false;
    Stepper_MoveTo(&motorJ1, j1_steps);
    Stepper_MoveTo(&motorJ2, j2_steps);
    Stepper_MoveTo(&motorZ, j3_steps);
    __enable_irq();
}

void Dispatcher_HomeAllAxes(void)
{
    // Etap 1: Zatrzymaj J1 i J2, wyzeruj ich flagę isHomed i zbazuj najpierw oś Z!
    __disable_irq();
    pendingTeachReturnJ1J2 = false;
    Stepper_Stop(&motorJ1);
    Stepper_Stop(&motorJ2);
    motorJ1.isHomed = false;
    motorJ2.isHomed = false;

    pendingHomeJ1J2 = true;
    Stepper_StartHoming(&motorZ);
    __enable_irq();
}

void Dispatcher_SetTeachMode(uint8_t axisId, bool releaseDrive)
{
    if (axisId == TEACH_AXIS_ALL || axisId == TEACH_AXIS_J1) {
        if (releaseDrive) Stepper_EnableTeachIn(&motorJ1);
        else Stepper_DisableTeachIn(&motorJ1);
    }
    if (axisId == TEACH_AXIS_ALL || axisId == TEACH_AXIS_J2) {
        if (releaseDrive) Stepper_EnableTeachIn(&motorJ2);
        else Stepper_DisableTeachIn(&motorJ2);
    }
    if (axisId == TEACH_AXIS_ALL || axisId == TEACH_AXIS_J3) {
        if (releaseDrive) Stepper_EnableTeachIn(&motorZ);
        else Stepper_DisableTeachIn(&motorZ);
    }
}

void Dispatcher_StartTeachInReturnAll(void)
{
    // Etap 1: Najpierw oś Z wraca na krańcówkę ze zliczaniem kroków, a potem J1 i J2
    pendingHomeJ1J2 = false;
    motorJ1.isTeachInDone = false;
    motorJ2.isTeachInDone = false;
    motorJ1.teachInStepsCount = 0;
    motorJ2.teachInStepsCount = 0;

    pendingTeachReturnJ1J2 = true;
    Stepper_StartTeachInReturn(&motorZ);
}

void Dispatcher_StopAllMotors(void)
{
    pendingHomeJ1J2 = false;
    pendingTeachReturnJ1J2 = false;
    Stepper_Stop(&motorJ1);
    Stepper_Stop(&motorJ2);
    Stepper_Stop(&motorZ);
}
/* USER CODE END Variables */
/* Definitions for DispatcherTask */
osThreadId_t DispatcherTaskHandle;
const osThreadAttr_t DispatcherTask_attributes = {
  .name = "DispatcherTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for KinematicsTask */
osThreadId_t KinematicsTaskHandle;
const osThreadAttr_t KinematicsTask_attributes = {
  .name = "KinematicsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ToolTask */
osThreadId_t ToolTaskHandle;
const osThreadAttr_t ToolTask_attributes = {
  .name = "ToolTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorTask(void *argument);
void StartKinematicsTask(void *argument);
void StartCommTask(void *argument);
void StartToolTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  SystemState_Init();
  ToolController_Init(PUMP_PIN_GPIO_Port, PUMP_PIN_Pin);
  Serial_Init();

  Stepper_Init(&motorJ1, J1_STEP_GPIO_Port, J1_STEP_Pin, J1_DIR_GPIO_Port, J1_DIR_Pin, J1_ENABLE_GPIO_Port, J1_ENABLE_Pin);
  Stepper_SetSpeedDivider(&motorJ1, J1_J2_STEP_DIVIDER);
  Limit_Switch_Init(&motorJ1, J1_LIMIT_GPIO_Port, J1_LIMIT_Pin);

  Stepper_Init(&motorJ2, J2_STEP_GPIO_Port, J2_STEP_Pin, J2_DIR_GPIO_Port, J2_DIR_Pin, J2_ENABLE_GPIO_Port, J2_ENABLE_Pin);
  Stepper_SetSpeedDivider(&motorJ2, J1_J2_STEP_DIVIDER);
  Limit_Switch_Init(&motorJ2, J2_LIMIT_GPIO_Port, J2_LIMIT_Pin);

  Stepper_Init(&motorZ, Z_STEP_GPIO_Port, Z_STEP_Pin, Z_DIR_GPIO_Port, Z_DIR_Pin, Z_ENABLE_GPIO_Port, Z_ENABLE_Pin);
  Stepper_SetSpeedDivider(&motorZ, Z_STEP_DIVIDER);
  Limit_Switch_Init(&motorZ, Z_LIMIT_GPIO_Port, Z_LIMIT_Pin);

  StepperTimer_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (10, sizeof(CommandFrame_t), &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DispatcherTask */
  DispatcherTaskHandle = osThreadNew(StartDefaultTask, NULL, &DispatcherTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of KinematicsTask */
  KinematicsTaskHandle = osThreadNew(StartKinematicsTask, NULL, &KinematicsTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* creation of ToolTask */
  ToolTaskHandle = osThreadNew(StartToolTask, NULL, &ToolTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  CommandFrame_t frame;
  /* Infinite loop */
  for(;;)
  {
    if (HAL_GPIO_ReadPin(ESTOP_PIN_GPIO_Port, ESTOP_PIN_Pin) == GPIO_PIN_RESET)
    {
      if (!SystemState_IsEstop())
      {
        SystemState_SetEstop(true);
        SystemState_SetPendingMotionAck(0);
        ToolController_EnablePump(false);
        Dispatcher_StopAllMotors();
        printf("[ESTOP] Hardware button pressed!\r\n");
      }
    }

    if (osMessageQueueGet(CommandQueueHandle, &frame, NULL, 10) == osOK)
    {
      Dispatcher_ProcessCommand(&frame);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    if (!SystemState_IsEstop())
    {
      SystemState_UpdateSteps(motorJ1.currentPosition, motorJ2.currentPosition, motorZ.currentPosition);

      if (!SystemState_IsHomed() &&
          motorJ1.isHomed && motorJ2.isHomed && motorZ.isHomed &&
          motorJ1.currentState == STATE_IDLE &&
          motorJ2.currentState == STATE_IDLE &&
          motorZ.currentState == STATE_IDLE)
      {
        SystemState_SetHomed(true);
        printf("[HOMING] All axes homed! Position set to (0, 0, 0)\r\n");
      }

      if (motorJ1.isTeachInDone && motorJ2.isTeachInDone && motorZ.isTeachInDone &&
          motorJ1.currentState == STATE_IDLE &&
          motorJ2.currentState == STATE_IDLE &&
          motorZ.currentState == STATE_IDLE)
      {
        SystemState_SetTeachInSteps(motorJ1.teachInStepsCount,
                                    motorJ2.teachInStepsCount,
                                    motorZ.teachInStepsCount);

        motorJ1.teachInStepsCount = 0;
        motorJ2.teachInStepsCount = 0;
        motorZ.teachInStepsCount = 0;

        motorJ1.isTeachInDone = false;
        motorJ2.isTeachInDone = false;
        motorZ.isTeachInDone = false;
      }

      if (!pendingHomeJ1J2 && !pendingTeachReturnJ1J2 &&
          motorJ1.currentState == STATE_IDLE &&
          motorJ2.currentState == STATE_IDLE &&
          motorZ.currentState == STATE_IDLE)
      {
        SystemState_MarkMotionAckCompleted();
      }
    }
    else
    {
      Stepper_Stop(&motorJ1);
      Stepper_Stop(&motorJ2);
      Stepper_Stop(&motorZ);
    }

    osDelay(5);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartKinematicsTask */
/**
* @brief Function implementing the KinematicsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKinematicsTask */
void StartKinematicsTask(void *argument)
{
  /* USER CODE BEGIN StartKinematicsTask */
  RobotState_t state;
  Point3D_t cartesian;
  JointSteps_t teachSteps;
  Point3D_t teachCartesian;
  uint8_t completedCmd = 0;
  /* Infinite loop */
  for(;;)
  {
    SystemState_Get(&state);
    Kinematics_StepsToCartesian(state.currentSteps, &cartesian);
    SystemState_UpdateCartesian(cartesian.x, cartesian.y, cartesian.z);

    if (SystemState_ConsumeTeachInSteps(&teachSteps))
    {
      Kinematics_StepsToCartesian(teachSteps, &teachCartesian);
      Serial_SendFrame(CMD_TEACH_RESPONSE, teachCartesian.x, teachCartesian.y, teachCartesian.z);
    }

    if (SystemState_ConsumeCompletedMotionAck(&completedCmd))
    {
      Serial_SendFrame(CMD_ACK, (float)completedCmd, 1.0f, 0.0f);
    }

    osDelay(20);
  }
  /* USER CODE END StartKinematicsTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(COMMS_LED_GPIO_Port, COMMS_LED_Pin);
    osDelay(500);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartToolTask */
/**
* @brief Function implementing the ToolTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartToolTask */
void StartToolTask(void *argument)
{
  /* USER CODE BEGIN StartToolTask */
  uint8_t activeDelayCmd = 0;
  uint32_t delayStartTick = 0;
  uint32_t delayDurationMs = 0;
  bool isDelayActive = false;

  uint8_t newCmd = 0;
  uint32_t newDelayMs = 0;
  /* Infinite loop */
  for(;;)
  {
    // 1. Sprawdź czy przyszło nowe zlecenie CMD_DELAY (bez blokowania pętli!)
    if (SystemState_ConsumePendingDelay(&newCmd, &newDelayMs))
    {
      activeDelayCmd = newCmd;
      delayDurationMs = newDelayMs;
      delayStartTick = osKernelGetTickCount();
      isDelayActive = true;
    }

    // 2. Nieblokujące sprawdzanie upływu czasu na podstawie licznika ticków RTOS
    if (isDelayActive)
    {
      if (SystemState_IsEstop())
      {
        isDelayActive = false;
      }
      else if ((osKernelGetTickCount() - delayStartTick) >= delayDurationMs)
      {
        isDelayActive = false;
        Serial_SendFrame(CMD_ACK, (float)activeDelayCmd, 1.0f, 0.0f);
      }
    }

    osDelay(10);
  }
  /* USER CODE END StartToolTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

