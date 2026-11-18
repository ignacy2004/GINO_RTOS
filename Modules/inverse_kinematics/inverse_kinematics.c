#include "inverse_kinematics.h"
#include <math.h>
#include <stdlib.h>

#define PI_F            3.14159265f
#define RAD_TO_DEG      (180.0f / PI_F)
#define DEG_TO_RAD      (PI_F / 180.0f)

void Kinematics_AnglesToSteps(float j1_deg, float j2_deg, float z_mm, JointSteps_t* outSteps)
{
    if (outSteps == NULL) return;

    float travel_j1_deg = j1_deg - J1_HOME_ANGLE_DEG;
    float travel_j2_deg = j2_deg - J2_HOME_ANGLE_DEG;
    float travel_z_mm   = z_mm   - Z_HOME_POS_MM;

    if (travel_j1_deg < 0.0f) travel_j1_deg += 360.0f;
    if (travel_j2_deg < 0.0f) travel_j2_deg += 360.0f;

    outSteps->j1_steps = (int32_t)lroundf(travel_j1_deg * STEPS_PER_DEG_J1);
    outSteps->j2_steps = (int32_t)lroundf(travel_j2_deg * STEPS_PER_DEG_J2);
    outSteps->j3_steps = (int32_t)lroundf(travel_z_mm   * STEPS_PER_MM_Z);
}

void Kinematics_StepsToAngles(JointSteps_t steps, float* j1_deg, float* j2_deg, float* z_mm)
{
    if (j1_deg != NULL) {
        *j1_deg = J1_HOME_ANGLE_DEG + ((float)steps.j1_steps / STEPS_PER_DEG_J1);
    }
    if (j2_deg != NULL) {
        *j2_deg = J2_HOME_ANGLE_DEG + ((float)steps.j2_steps / STEPS_PER_DEG_J2);
    }
    if (z_mm != NULL) {
        *z_mm = Z_HOME_POS_MM + ((float)steps.j3_steps / STEPS_PER_MM_Z);
    }
}

bool Kinematics_AreStepsWithinLimits(JointSteps_t steps)
{
    if (steps.j1_steps < 0 || steps.j1_steps > J1_MAX_STEPS) return false;
    if (steps.j2_steps < 0 || steps.j2_steps > J2_MAX_STEPS) return false;
    if (steps.j3_steps < 0 || steps.j3_steps > Z_MAX_STEPS)  return false;
    return true;
}

bool Kinematics_CartesianToSteps(Point3D_t target, JointSteps_t* outSteps)
{
    if (outSteps == NULL) return false;

    float r_sq = target.x * target.x + target.y * target.y;
    float r = sqrtf(r_sq);

    // Minimalny promien osiagalnosci (geometria ramion vs kolumna bazy)
    float min_r = fabsf(L1_LENGTH - L2_LENGTH);
    if (MIN_WORKSPACE_RADIUS_MM > min_r) {
        min_r = MIN_WORKSPACE_RADIUS_MM;
    }

    // Sprawdzenie osiagalnosci promieniowej
    if (r > (L1_LENGTH + L2_LENGTH) || r < min_r || r < 1e-4f)
    {
        return false;
    }

    // Twierdzenie cosinusow dla J2
    float c2 = (r_sq - (L1_LENGTH * L1_LENGTH) - (L2_LENGTH * L2_LENGTH)) / (2.0f * L1_LENGTH * L2_LENGTH);
    if (c2 > 1.0f) c2 = 1.0f;
    if (c2 < -1.0f) c2 = -1.0f;

    float s2_abs = sqrtf(1.0f - c2 * c2);
    float k1 = L1_LENGTH + L2_LENGTH * c2;

    // --- Konfiguracja A: Lokiec Prawy / Dolny (s2 >= 0) ---
    float s2_a = s2_abs;
    float k2_a = L2_LENGTH * s2_a;
    float theta2_a_deg = atan2f(s2_a, c2) * RAD_TO_DEG;
    float theta1_a_deg = atan2f(target.y * k1 - target.x * k2_a, target.x * k1 + target.y * k2_a) * RAD_TO_DEG;

    JointSteps_t steps_a;
    Kinematics_AnglesToSteps(theta1_a_deg, theta2_a_deg, target.z, &steps_a);
    bool valid_a = Kinematics_AreStepsWithinLimits(steps_a);

    // --- Konfiguracja B: Lokiec Lewy / Gorny (s2 <= 0) ---
    float s2_b = -s2_abs;
    float k2_b = L2_LENGTH * s2_b;
    float theta2_b_deg = atan2f(s2_b, c2) * RAD_TO_DEG;
    float theta1_b_deg = atan2f(target.y * k1 - target.x * k2_b, target.x * k1 + target.y * k2_b) * RAD_TO_DEG;

    JointSteps_t steps_b;
    Kinematics_AnglesToSteps(theta1_b_deg, theta2_b_deg, target.z, &steps_b);
    bool valid_b = Kinematics_AreStepsWithinLimits(steps_b);

    // Wybor najlepszej osiagalnej konfiguracji
    if (valid_a && valid_b)
    {
        RobotState_t currentState;
        SystemState_Get(&currentState);

        int32_t dist_a = labs(steps_a.j1_steps - currentState.currentSteps.j1_steps) +
                         labs(steps_a.j2_steps - currentState.currentSteps.j2_steps);
        int32_t dist_b = labs(steps_b.j1_steps - currentState.currentSteps.j1_steps) +
                         labs(steps_b.j2_steps - currentState.currentSteps.j2_steps);

        *outSteps = (dist_a <= dist_b) ? steps_a : steps_b;
        return true;
    }
    else if (valid_a)
    {
        *outSteps = steps_a;
        return true;
    }
    else if (valid_b)
    {
        *outSteps = steps_b;
        return true;
    }

    return false;
}

void Kinematics_StepsToCartesian(JointSteps_t steps, Point3D_t* outCartesian)
{
    if (outCartesian == NULL) return;

    float j1_deg, j2_deg, z_mm;
    Kinematics_StepsToAngles(steps, &j1_deg, &j2_deg, &z_mm);

    float a1 = j1_deg * DEG_TO_RAD;
    float a2 = j2_deg * DEG_TO_RAD;

    outCartesian->x = L1_LENGTH * cosf(a1) + L2_LENGTH * cosf(a1 + a2);
    outCartesian->y = L1_LENGTH * sinf(a1) + L2_LENGTH * sinf(a1 + a2);
    outCartesian->z = z_mm;
}
