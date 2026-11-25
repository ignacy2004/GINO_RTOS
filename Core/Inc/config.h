#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

// ============================================================================
// 1. PARAMETRY SILNIKOW KROKOWYCH I STEROWNIKOW TMC2209
// ============================================================================
#define MOTOR_STEPS_PER_REV         200.0f  

#define MICROSTEPS_J1               8.0f   
#define MICROSTEPS_J2               8.0f   
#define MICROSTEPS_Z                8.0f   

// ============================================================================
// 2. PRZELOZENIA MECHANICZNE (PRZEKLADNIE PASKOWE / SRUBOWE)
// ============================================================================

#define GEAR_RATIO_J1               3.84f
#define GEAR_RATIO_J2               3.84f
#define GEAR_RATIO_Z                1.0f
#define Z_SCREW_LEAD_MM             8.0f


#define STEPS_PER_DEG_J1            ((MOTOR_STEPS_PER_REV * MICROSTEPS_J1 * GEAR_RATIO_J1) / 360.0f)
#define STEPS_PER_DEG_J2            ((MOTOR_STEPS_PER_REV * MICROSTEPS_J2 * GEAR_RATIO_J2) / 360.0f)
#define STEPS_PER_MM_Z              ((MOTOR_STEPS_PER_REV * MICROSTEPS_Z  * GEAR_RATIO_Z)  / Z_SCREW_LEAD_MM)

// ============================================================================
// 3. GEOMETRIA RAMION SCARA [mm]
// ============================================================================

#define L1_LENGTH                   100.0f 
#define L2_LENGTH                   100.0f  
#define MIN_WORKSPACE_RADIUS_MM     50.0f  

// ============================================================================
// 4. POZYCJA KRANCOWEK (HOME OFFSETS)
// ============================================================================
// Kat matematyczny osi (w stopniach), gdy dana os dotyka swojej krancowki (0 krokow):
// 0 deg dla J1 = ramie wzdluz osi +X; 0 deg dla J2 = przedramie wyprostowane wzgledem J1.
// Poniewaz krancowka jest na poczatku zakresu (0 krokow) i silniki jezdza na '+',
// ujemny kat bazowy pozwala osiagnac kat 0 deg (wyprostowane ramie) na dodatnich krokach.
#define J1_HOME_ANGLE_DEG           (-75.0f)
#define J2_HOME_ANGLE_DEG           (-77.0f)
#define Z_HOME_POS_MM               0.0f

// ============================================================================
// 5. LIMITY PROGRAMOWE OD KRANCOWKI (SOFT LIMITS)
// ============================================================================

#define J1_MAX_TRAVEL_DEG           170.0f
#define J2_MAX_TRAVEL_DEG           170.0f
#define Z_MAX_TRAVEL_MM             100.0f


#define J1_MAX_STEPS                ((int32_t)(J1_MAX_TRAVEL_DEG * STEPS_PER_DEG_J1))
#define J2_MAX_STEPS                ((int32_t)(J2_MAX_TRAVEL_DEG * STEPS_PER_DEG_J2))
#define Z_MAX_STEPS                 ((int32_t)(Z_MAX_TRAVEL_MM  * STEPS_PER_MM_Z))

#endif
