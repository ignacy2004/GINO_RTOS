#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float x;
    float y;
    float z;
} Point3D_t;

typedef struct {
    int32_t j1_steps;
    int32_t j2_steps;
    int32_t j3_steps;
} JointSteps_t;

typedef struct {
    Point3D_t    currentCartesian;
    JointSteps_t currentSteps;
    bool         isHomed;
    bool         pumpState;
    bool         emergencyStop;
} RobotState_t;

void SystemState_Init(void);
void SystemState_Get(RobotState_t* outState);
void SystemState_UpdateSteps(int32_t j1, int32_t j2, int32_t j3);
void SystemState_UpdateCartesian(float x, float y, float z);
void SystemState_SetPump(bool enable);
void SystemState_SetEstop(bool estop);
bool SystemState_IsEstop(void);

#endif