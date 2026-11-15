#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <stdbool.h>
#include "system_state.h"

#define L1_LENGTH           200.0f
#define L2_LENGTH           150.0f

#define STEPS_PER_DEG_J1    35.55f
#define STEPS_PER_DEG_J2    35.55f
#define STEPS_PER_MM_Z      100.0f

bool Kinematics_CartesianToSteps(Point3D_t target, JointSteps_t* outSteps);
void Kinematics_StepsToCartesian(JointSteps_t steps, Point3D_t* outCartesian);

#endif

