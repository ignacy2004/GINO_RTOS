#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <stdbool.h>
#include "system_state.h"
#include "robot_config.h"

void Kinematics_AnglesToSteps(float j1_deg, float j2_deg, float z_mm, JointSteps_t* outSteps);
void Kinematics_StepsToAngles(JointSteps_t steps, float* j1_deg, float* j2_deg, float* z_mm);
bool Kinematics_AreStepsWithinLimits(JointSteps_t steps);

bool Kinematics_CartesianToSteps(Point3D_t target, JointSteps_t* outSteps);
void Kinematics_StepsToCartesian(JointSteps_t steps, Point3D_t* outCartesian);

#endif
