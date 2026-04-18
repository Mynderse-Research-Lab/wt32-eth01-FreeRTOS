/**
 * @file axis_drivetrain_params.h
 * @brief Per-axis MECHANICAL tuning (drivetrain, travel envelope, motion caps).
 *
 * Each axis picks its own DrivetrainType and fills in the type-specific
 * fields. Axes are configured INDEPENDENTLY - mix and match ballscrew, belt,
 * rack-pinion, or rotary-direct as the hardware demands. There is no profile
 * macro; each AXIS_*_DRIVETRAIN macro alone dictates the axis topology.
 *
 * Scope: ballscrew lead, belt lead-per-rev, rotary output gear ratio, soft
 * travel limits, motion caps, position tolerance, kinematic offsets, gripper
 * timing. Motor/driver/gearbox ELECTRICAL values live in
 * include/axis_pulse_motor_params.h.
 *
 * Verified target actuators:
 *   - X: SCHUNK Beta 100-ZRS toothed-belt linear actuator (200 mm/rev, 550 mm stroke, +/-0.08 mm repeatability)
 *   - Y: SCHUNK Beta 80-SRS ballscrew linear actuator (20 mm pitch, 150 mm stroke, +/-0.03 mm repeatability, 3000 rpm critical)
 *   - Theta: SCHUNK ERD 04-40-D-H-N miniature rotary unit (unlimited rotation, 600 rpm max, +/-0.01 deg)
 */

#ifndef AXIS_DRIVETRAIN_PARAMS_H
#define AXIS_DRIVETRAIN_PARAMS_H

#include "axis_pulse_motor_params.h"

/* ---------------------------------------------------------------------------
 * DrivetrainType enum values. Mirrors PulseMotor::DrivetrainType so the macro
 * assignments below remain readable even in C-preprocessed contexts.
 * --------------------------------------------------------------------------- */
#define DRIVETRAIN_BALLSCREW                1
#define DRIVETRAIN_BELT                     2
#define DRIVETRAIN_RACKPINION               3
#define DRIVETRAIN_ROTARY_DIRECT            4

/* ===========================================================================
 * X AXIS  -  SCHUNK Beta 100-ZRS toothed-belt linear actuator
 * ===========================================================================
 * Datasheet: "Beta 100-ZRS-40AT10-200-550-1050-AK-AZ2-2EO10-1ES10-6BL2-1-MGK2-GS24"
 *   stroke per round: 200 mm  -> AXIS_X_BELT_LEAD_MM_PER_REV
 *   mech. stroke:     550 mm  -> AXIS_X_TRAVEL_MAX_MM
 *   repeatability:    +/-0.08 mm */
#define AXIS_X_DRIVETRAIN                   DRIVETRAIN_BELT
#define AXIS_X_BELT_LEAD_MM_PER_REV         200.0f
#define AXIS_X_BELT_PULLEY_TEETH                0u  /* metadata only; lead_per_rev is authoritative */
#define AXIS_X_BELT_PITCH_MM                  0.0f
#define AXIS_X_TRAVEL_MIN_MM                  0.0f
#define AXIS_X_TRAVEL_MAX_MM                550.0f
#define AXIS_X_MAX_SPEED_MM_PER_S           500.0f  /* placeholder; refine from Trap Moves */
#define AXIS_X_ACCEL_MM_PER_S2             3000.0f  /* placeholder */
#define AXIS_X_DECEL_MM_PER_S2             3000.0f  /* placeholder */
#define AXIS_X_POSITION_TOLERANCE_MM         0.08f  /* per datasheet repeatability */

/* ===========================================================================
 * Y AXIS  -  SCHUNK Beta 80-SRS ballscrew linear actuator
 * ===========================================================================
 * Datasheet: "Beta 80-SRS-M-2020-150-530-2EO10-1ES10-4NS3-1-MGK1-GS19"
 *   pitch 20 mm, thread dia 20 mm, critical speed 3000 rpm
 *   mech. stroke:  150 mm
 *   axial tolerance: 0.08 mm; repeat accuracy: +/-0.03 mm */
#define AXIS_Y_DRIVETRAIN                   DRIVETRAIN_BALLSCREW
#define AXIS_Y_BALLSCREW_LEAD_MM             20.0f
#define AXIS_Y_BALLSCREW_SHAFT_DIA_MM        20.0f  /* informational */
#define AXIS_Y_BALLSCREW_CRITICAL_RPM        3000u
#define AXIS_Y_TRAVEL_MIN_MM                  0.0f
#define AXIS_Y_TRAVEL_MAX_MM                150.0f
#define AXIS_Y_MAX_SPEED_MM_PER_S           500.0f  /* clamped below by critical-speed derived cap */
#define AXIS_Y_ACCEL_MM_PER_S2             5000.0f  /* placeholder */
#define AXIS_Y_DECEL_MM_PER_S2             5000.0f  /* placeholder */
#define AXIS_Y_POSITION_TOLERANCE_MM         0.03f

/* ===========================================================================
 * THETA AXIS  -  SCHUNK ERD 04-40-D-H-N miniature rotary unit
 * ===========================================================================
 * Datasheet (product page 000000000000331220):
 *   angle of rotation: > 360 deg (unlimited)
 *   nominal torque: 0.4 Nm; peak torque: 1.2 Nm
 *   max rotational speed: 600 rpm = 3600 deg/s
 *   repeat accuracy: +/-0.01 deg
 *   max permissible mass moment of inertia: 0.008 kg*m^2
 *   max Fx/Fz: 150 N; max My: 2.5 Nm
 * The rotary hardware allows unlimited rotation; the firmware clamps to the
 * soft-limit range below to keep gripper cabling unwound. */
#define AXIS_THETA_DRIVETRAIN               DRIVETRAIN_ROTARY_DIRECT
#define AXIS_THETA_OUTPUT_GEAR_RATIO          1.0f  /* output_rev / motor_rev; 1.0 = direct drive */
#define AXIS_THETA_TRAVEL_MIN_DEG          -180.0f
#define AXIS_THETA_TRAVEL_MAX_DEG           180.0f
#define AXIS_THETA_MAX_SPEED_DEG_PER_S     3600.0f  /* 600 rpm datasheet cap */
#define AXIS_THETA_ACCEL_DEG_PER_S2       18000.0f  /* placeholder; respect inertia limit */
#define AXIS_THETA_DECEL_DEG_PER_S2       18000.0f  /* placeholder */
#define AXIS_THETA_POSITION_TOLERANCE_DEG    0.01f

/* Datasheet envelope - not consumed by firmware, retained for commissioning. */
#define AXIS_THETA_NOMINAL_TORQUE_NM          0.4f
#define AXIS_THETA_PEAK_TORQUE_NM             1.2f
#define AXIS_THETA_MAX_INERTIA_KGM2         0.008f
#define AXIS_THETA_MAX_AXIAL_FORCE_N        150.0f
#define AXIS_THETA_MAX_MOMENT_NM              2.5f

/* ===========================================================================
 * Gantry-level geometry and end-effector timing
 * ===========================================================================
 * Kinematic offsets (end-effector geometry). Adjust per deployment to match
 * the installed adapter plates. */
#define GANTRY_Y_AXIS_Z_OFFSET_MM            80.0f
#define GANTRY_THETA_X_OFFSET_MM            -55.0f
#define GANTRY_GRIPPER_Y_OFFSET_MM          385.0f
#define GANTRY_GRIPPER_Z_OFFSET_MM           80.0f
#define GANTRY_SAFE_Y_HEIGHT_MM             150.0f

/* Pneumatic gripper (SCHUNK KGG 100-80). Open/close times from datasheet. */
#define GANTRY_GRIPPER_OPEN_TIME_MS            190u
#define GANTRY_GRIPPER_CLOSE_TIME_MS           150u

/* ---------------------------------------------------------------------------
 * Derived helpers.
 * These are macros so they may be consumed where only the preprocessor runs.
 * Runtime code typically prefers PulseMotor::pulsesPerMm / pulsesPerDeg
 * applied to a DrivetrainConfig.
 * --------------------------------------------------------------------------- */

/* X: belt encoder pulses per mm */
#define AXIS_X_PULSES_PER_MM \
    (((double)AXIS_X_ENCODER_PPR * (double)AXIS_X_MOTOR_REDUCER_RATIO) / (double)AXIS_X_BELT_LEAD_MM_PER_REV)

/* Y: ballscrew encoder pulses per mm */
#define AXIS_Y_PULSES_PER_MM \
    (((double)AXIS_Y_ENCODER_PPR * (double)AXIS_Y_MOTOR_REDUCER_RATIO) / (double)AXIS_Y_BALLSCREW_LEAD_MM)

/* Y: ballscrew speed cap derived from critical RPM. Never command a linear
 * speed that would put the screw above this RPM. */
#define AXIS_Y_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S \
    (((double)AXIS_Y_BALLSCREW_CRITICAL_RPM / 60.0) * (double)AXIS_Y_BALLSCREW_LEAD_MM)

/* Theta: rotary encoder pulses per degree */
#define AXIS_THETA_PULSES_PER_DEG \
    (((double)AXIS_THETA_ENCODER_PPR * (double)AXIS_THETA_MOTOR_REDUCER_RATIO * (double)AXIS_THETA_OUTPUT_GEAR_RATIO) / 360.0)

/* ---------------------------------------------------------------------------
 * Reserved commissioning slots.
 * Filled in when the SCHUNK Trap Move calculation attachments are located.
 * Not consumed by firmware; kept here for commissioning traceability.
 * ---------------------------------------------------------------------------
 *   #define AXIS_X_REFLECTED_INERTIA_KGM2      ...
 *   #define AXIS_X_REQUIRED_TORQUE_NM          ...
 *   #define AXIS_Y_REFLECTED_INERTIA_KGM2      ...
 *   #define AXIS_Y_REQUIRED_TORQUE_NM          ...
 *   #define AXIS_THETA_REFLECTED_INERTIA_KGM2  ...
 *   #define AXIS_THETA_REQUIRED_TORQUE_NM      ...
 *   #define GANTRY_CYCLE_TIME_TARGET_S         2.04
 * --------------------------------------------------------------------------- */

#endif /* AXIS_DRIVETRAIN_PARAMS_H */
