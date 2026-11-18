#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

// ============================================================================
// 1. PARAMETRY SILNIKOW KROKOWYCH I STEROWNIKOW TMC2209
// ============================================================================
#define MOTOR_STEPS_PER_REV         200.0f  // Silnik 1.8 deg = 200 pelnych krokow/obrot

#define MICROSTEPS_J1               16.0f   // Podzial mikrokrokowy osi J1
#define MICROSTEPS_J2               16.0f   // Podzial mikrokrokowy osi J2
#define MICROSTEPS_Z                16.0f   // Podzial mikrokrokowy osi Z

// ============================================================================
// 2. PRZELOZENIA MECHANICZNE (PRZEKLADNIE PASKOWE / SRUBOWE)
// ============================================================================
// Przelozenie = (liczba zebow kola odbiorczego) / (liczba zebow kola na silniku)
#define GEAR_RATIO_J1               16.66f  // Przelozenie osi J1 (np. 50:3 = 16.66)
#define GEAR_RATIO_J2               16.66f  // Przelozenie osi J2 (np. 50:3 = 16.66)
#define GEAR_RATIO_Z                1.0f    // 1.0 jesli silnik napedza srube bezposrednio
#define Z_SCREW_LEAD_MM             8.0f    // Sruba fi8: Tr8x8 (standard drukarek 3D) = 8.0f mm/obr (Tr8x4 = 4.0f, Tr8x2 = 2.0f, M8 = 1.25f)

// Automatyczne wyliczenie krokow na stopien / milimetr:
// J1/J2: 200 * 16 * 16.66 / 360 = ~148.09 krokow/deg
// Z:     200 * 16 * 1.0   / 8.0 = 400.0 krokow/mm
#define STEPS_PER_DEG_J1            ((MOTOR_STEPS_PER_REV * MICROSTEPS_J1 * GEAR_RATIO_J1) / 360.0f)
#define STEPS_PER_DEG_J2            ((MOTOR_STEPS_PER_REV * MICROSTEPS_J2 * GEAR_RATIO_J2) / 360.0f)
#define STEPS_PER_MM_Z              ((MOTOR_STEPS_PER_REV * MICROSTEPS_Z  * GEAR_RATIO_Z)  / Z_SCREW_LEAD_MM)

// ============================================================================
// 3. GEOMETRIA RAMION SCARA [mm]
// ============================================================================
#define L1_LENGTH                   100.0f  // Dlugosc ramienia glownego J1 -> J2 [mm]
#define L2_LENGTH                   100.0f  // Dlugosc przedramienia J2 -> efektor [mm]
#define MIN_WORKSPACE_RADIUS_MM     50.0f   // Minimalny promien od osi bazy J1 [mm]

// ============================================================================
// 4. POZYCJA KRANCOWEK (HOME OFFSETS)
// ============================================================================
// Kat matematyczny osi (w stopniach), gdy dana os dotyka swojej krancowki (0 krokow):
// 0 deg dla J1 = ramie wzdluz osi +X; 0 deg dla J2 = przedramie wyprostowane wzgledem J1.
// Poniewaz krancowka jest na poczatku zakresu (0 krokow) i silniki jezdza na '+',
// ujemny kat bazowy pozwala osiagnac kat 0 deg (wyprostowane ramie) na dodatnich krokach.
#define J1_HOME_ANGLE_DEG           (0.0f)
#define J2_HOME_ANGLE_DEG           (0.0f)
#define Z_HOME_POS_MM               0.0f

// ============================================================================
// 5. LIMITY PROGRAMOWE OD KRANCOWKI (SOFT LIMITS)
// ============================================================================
// Maksymalny dozwolony ruch od krancowki (0) w stopniach / mm:
#define J1_MAX_TRAVEL_DEG           180.0f
#define J2_MAX_TRAVEL_DEG           240.0f
#define Z_MAX_TRAVEL_MM             100.0f

// Maksymalna liczba krokow od krancowki (mozna tez wpisac zmierzona liczbe krokow wprost):
#define J1_MAX_STEPS                ((int32_t)(J1_MAX_TRAVEL_DEG * STEPS_PER_DEG_J1))
#define J2_MAX_STEPS                ((int32_t)(J2_MAX_TRAVEL_DEG * STEPS_PER_DEG_J2))
#define Z_MAX_STEPS                 ((int32_t)(Z_MAX_TRAVEL_MM  * STEPS_PER_MM_Z))

#endif // ROBOT_CONFIG_H
