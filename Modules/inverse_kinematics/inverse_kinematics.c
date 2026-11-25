#include "inverse_kinematics.h"
#include <math.h>
#include "config.h"

#define PI_F            3.14159265f
#define RAD_TO_DEG      (180.0f / PI_F)
#define DEG_TO_RAD      (PI_F / 180.0f)

bool Kinematics_CartesianToSteps(Point3D_t target, JointSteps_t* outSteps)
{
    if (outSteps == 0) return false;

    float r_sq = target.x * target.x + target.y * target.y;
    float r = sqrtf(r_sq);

    // Sprawdzenie osiagalnosci
    if (r > (L1_LENGTH + L2_LENGTH) || r < fabsf(L1_LENGTH - L2_LENGTH) || r < 1e-4f)
    {
        return false;
    }

    // Twierdzenie cosinusow dla J2
    float c2 = (r_sq - (L1_LENGTH * L1_LENGTH) - (L2_LENGTH * L2_LENGTH)) / (2.0f * L1_LENGTH * L2_LENGTH);
    if (c2 > 1.0f) c2 = 1.0f;
    if (c2 < -1.0f) c2 = -1.0f;

    float s2 = sqrtf(1.0f - c2 * c2);
    float theta2 = atan2f(s2, c2);

    // Kat J1
    float k1 = L1_LENGTH + L2_LENGTH * c2;
    float k2 = L2_LENGTH * s2;
    float theta1 = atan2f(target.y * k1 - target.x * k2, target.x * k1 + target.y * k2);

    // Przeliczenie na kroki (z uwzglednieniem offsetow katowych krancowek J1_HOME_ANGLE_DEG i J2_HOME_ANGLE_DEG!)
    float deg1 = (theta1 * RAD_TO_DEG) - J1_HOME_ANGLE_DEG;
    float deg2 = (theta2 * RAD_TO_DEG) - J2_HOME_ANGLE_DEG;
    float posZ = target.z - Z_HOME_POS_MM;

    if (deg1 < 0.0f) deg1 = 0.0f;
    if (deg2 < 0.0f) deg2 = 0.0f;
    if (posZ < 0.0f) posZ = 0.0f;

    outSteps->j1_steps = (int32_t)lroundf(deg1 * STEPS_PER_DEG_J1);
    outSteps->j2_steps = (int32_t)lroundf(deg2 * STEPS_PER_DEG_J2);
    outSteps->j3_steps = (int32_t)lroundf(posZ * STEPS_PER_MM_Z);

    if (outSteps->j1_steps > J1_MAX_STEPS) outSteps->j1_steps = J1_MAX_STEPS;
    if (outSteps->j2_steps > J2_MAX_STEPS) outSteps->j2_steps = J2_MAX_STEPS;
    if (outSteps->j3_steps > Z_MAX_STEPS)  outSteps->j3_steps = Z_MAX_STEPS;

    return true;
}

void Kinematics_StepsToCartesian(JointSteps_t steps, Point3D_t* outCartesian)
{
    if (outCartesian == 0) return;

    float a1 = (((float)steps.j1_steps / STEPS_PER_DEG_J1) + J1_HOME_ANGLE_DEG) * DEG_TO_RAD;
    float a2 = (((float)steps.j2_steps / STEPS_PER_DEG_J2) + J2_HOME_ANGLE_DEG) * DEG_TO_RAD;

    outCartesian->x = L1_LENGTH * cosf(a1) + L2_LENGTH * cosf(a1 + a2);
    outCartesian->y = L1_LENGTH * sinf(a1) + L2_LENGTH * sinf(a1 + a2);
    outCartesian->z = ((float)steps.j3_steps / STEPS_PER_MM_Z) + Z_HOME_POS_MM;
}

