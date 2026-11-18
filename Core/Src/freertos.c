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
#include "robot_config.h"
#include "system_state.h"
#include "tmc2209_stepper.h"
#include "tool_controller.h"
#include "serial_interface.h"
#include "dispatcher.h"
#include "inverse_kinematics.h"
#include <stdio.h>

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

static volatile bool irq_estop_event = false;
static volatile bool irq_j1_limit_event = false;
static volatile bool irq_j2_limit_event = false;
static volatile bool irq_z_limit_event = false;

void Dispatcher_MoveJoints(int32_t j1_steps, int32_t j2_steps, int32_t j3_steps)
{
    Stepper_MoveTo(&motorJ1, j1_steps);
    Stepper_MoveTo(&motorJ2, j2_steps);
    Stepper_MoveTo(&motorZ, j3_steps);
}

void Dispatcher_HomeMotors(uint8_t axisId)
{
    if (axisId == 0 || axisId == 1) {
        Stepper_StartHoming(&motorJ1);
    }
    if (axisId == 0 || axisId == 2) {
        Stepper_StartHoming(&motorJ2);
    }
    if (axisId == 0 || axisId == 3) {
        Stepper_StartHoming(&motorZ);
    }
}

void Dispatcher_StopAllMotors(void)
{
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
  Stepper_SetEndstop(&motorJ1, J1_LIMIT_GPIO_Port, J1_LIMIT_Pin);
  Stepper_SetMaxSteps(&motorJ1, J1_MAX_STEPS);

  Stepper_Init(&motorJ2, J2_STEP_GPIO_Port, J2_STEP_Pin, J2_DIR_GPIO_Port, J2_DIR_Pin, J2_ENABLE_GPIO_Port, J2_ENABLE_Pin);
  Stepper_SetEndstop(&motorJ2, J2_LIMIT_GPIO_Port, J2_LIMIT_Pin);
  Stepper_SetMaxSteps(&motorJ2, J2_MAX_STEPS);

  Stepper_Init(&motorZ, Z_STEP_GPIO_Port, Z_STEP_Pin, Z_DIR_GPIO_Port, Z_DIR_Pin, Z_ENABLE_GPIO_Port, Z_ENABLE_Pin);
  Stepper_SetEndstop(&motorZ, Z_LIMIT_GPIO_Port, Z_LIMIT_Pin);
  Stepper_SetMaxSteps(&motorZ, Z_MAX_STEPS);
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
  CommandQueueHandle = osMessageQueueNew (10, 20, &CommandQueue_attributes);

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
  uint8_t estop_filter_count = 0;
  /* Infinite loop */
  for(;;)
  {
    if (HAL_GPIO_ReadPin(ESTOP_PIN_GPIO_Port, ESTOP_PIN_Pin) == GPIO_PIN_RESET)
    {
      if (estop_filter_count < 3)
      {
        estop_filter_count++;
      }
      if (estop_filter_count >= 3 && !SystemState_IsEstop())
      {
        SystemState_SetEstop(true);
        ToolController_EnablePump(false);
        Dispatcher_StopAllMotors();
        printf("[ESTOP] Hardware button pressed!\r\n");
      }
    }
    else
    {
      estop_filter_count = 0;
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
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    tick += 1;
    osDelayUntil(tick);

    if (!SystemState_IsEstop())
    {
      Stepper_UpdateStateMachine(&motorJ1);
      Stepper_UpdateStateMachine(&motorJ2);
      Stepper_UpdateStateMachine(&motorZ);
      SystemState_UpdateSteps(motorJ1.currentPosition, motorJ2.currentPosition, motorZ.currentPosition);
      SystemState_SetHomed(motorJ1.isHomed && motorJ2.isHomed && motorZ.isHomed);
    }
    else
    {
      Stepper_Stop(&motorJ1);
      Stepper_Stop(&motorJ2);
      Stepper_Stop(&motorZ);
    }
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
  /* Infinite loop */
  for(;;)
  {
    SystemState_Get(&state);
    Kinematics_StepsToCartesian(state.currentSteps, &cartesian);
    SystemState_UpdateCartesian(cartesian.x, cartesian.y, cartesian.z);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
  }
  /* USER CODE END StartToolTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_tick_estop = 0;
  static uint32_t last_tick_j1 = 0;
  static uint32_t last_tick_j2 = 0;
  static uint32_t last_tick_z = 0;

  uint32_t current_tick = HAL_GetTick();

  if (GPIO_Pin == ESTOP_PIN_Pin)
  {
    if (HAL_GPIO_ReadPin(ESTOP_PIN_GPIO_Port, ESTOP_PIN_Pin) == GPIO_PIN_RESET)
    {
      if ((current_tick - last_tick_estop) >= 20)
      {
        last_tick_estop = current_tick;
        HAL_GPIO_WritePin(PUMP_PIN_GPIO_Port, PUMP_PIN_Pin, GPIO_PIN_RESET);
        Dispatcher_StopAllMotors();
        SystemState_SetEstopFromISR(true);
        irq_estop_event = true;
      }
    }
  }
  else if (GPIO_Pin == J1_LIMIT_Pin)
  {
    if (HAL_GPIO_ReadPin(J1_LIMIT_GPIO_Port, J1_LIMIT_Pin) == GPIO_PIN_RESET)
    {
      if ((current_tick - last_tick_j1) >= 20)
      {
        last_tick_j1 = current_tick;
        Stepper_OnEndstopTriggered(&motorJ1);
        irq_j1_limit_event = true;
      }
    }
  }
  else if (GPIO_Pin == J2_LIMIT_Pin)
  {
    if (HAL_GPIO_ReadPin(J2_LIMIT_GPIO_Port, J2_LIMIT_Pin) == GPIO_PIN_RESET)
    {
      if ((current_tick - last_tick_j2) >= 20)
      {
        last_tick_j2 = current_tick;
        Stepper_OnEndstopTriggered(&motorJ2);
        irq_j2_limit_event = true;
      }
    }
  }
  else if (GPIO_Pin == Z_LIMIT_Pin)
  {
    if (HAL_GPIO_ReadPin(Z_LIMIT_GPIO_Port, Z_LIMIT_Pin) == GPIO_PIN_RESET)
    {
      if ((current_tick - last_tick_z) >= 20)
      {
        last_tick_z = current_tick;
        Stepper_OnEndstopTriggered(&motorZ);
        irq_z_limit_event = true;
      }
    }
  }
}
/* USER CODE END Application */

