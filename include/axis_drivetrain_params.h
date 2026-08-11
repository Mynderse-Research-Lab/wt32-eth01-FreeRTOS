/**
 * @file axis_drivetrain_params.h
 * @brief Per-axis MECHANICAL tuning (drivetrain, hard-limit envelope, motion caps).
 *
 * Each axis picks its own DrivetrainType and fills in the kinematic parameters
 * appropriate to that topology. Axes are configured INDEPENDENTLY - mix and
 * match ballscrew, belt, rack-pinion, or rotary-direct as the hardware
 * demands. There is no profile macro; each AXIS_*_DRIVETRAIN macro alone
 * dictates the axis topology.
 *
 * Naming convention: macros here are drivetrain-AGNOSTIC. For any linear
 * drivetrain (belt, ballscrew, rack-pinion) the primary kinematic constant is
 * the linear travel produced by one motor-side revolution, expressed as
 *   AXIS_<X>_LEAD_MM_PER_REV
 * regardless of whether that travel comes from belt pitch x pulley teeth,
 * ballscrew pitch, or pinion circumference. Drivetrain-specific bookkeeping
 * (belt pulley teeth, belt tooth pitch, ballscrew shaft diameter, etc.) lives
 * in the commissioning notes below and - if the runtime driver needs it - in
 * the corresponding PulseMotor::DrivetrainConfig field inside src/main.cpp.
 *
 * Limit semantics:
 *   AXIS_*_HARD_LIMIT_MIN/MAX_*  = mechanical envelope the axis must never
 *                                  exceed. For linear axes these come from
 *                                  the actuator datasheet stroke. For the
 *                                  rotary axis - whose hardware allows
 *                                  unlimited rotation - these are firmware
 *                                  clamps that protect gripper cabling from
 *                                  winding up and are therefore the effective
 *                                  hard limit in practice.
 *   SOFT LIMITS                  = NOT defined here. They are derived at
 *                                  boot-time homing / calibration from the
 *                                  actual limit-switch positions and may be
 *                                  tighter than the hard envelope. See
 *                                  Gantry::calibrate() and the homing task in
 *                                  src/gantry_test_console.cpp.
 *
 * Scope of this header: kinematic lead, hard-limit envelope, motion caps,
 * position tolerance, kinematic offsets, gripper timing, default profile.
 * Firmware builds override via idf.py menuconfig "Gantry kinematics". Motor/driver/gearbox
 * ELECTRICAL values live in include/axis_pulse_motor_params.h.
 *
 * Coordinate convention:
 *   - X: horizontal traverse along the gantry beam (across the conveyor belt).
 *   - Y: along-belt direction; the gantry has NO Y actuator. Conveyor downstream
 *        is the -Y direction. Used by the pick scheduler to describe battery
 *        positions on the belt; not consumed by the motion layer.
 *   - Z: vertical (gantry descent axis). +Z = up (away from belt). Firmware
 *        joint Z=0 is the homing datum (limit switch), which is generally
 *        NOT exactly on the belt touch plane; see GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM.
 *        Z_max is the top / home / safe-retracted position.
 *
 * Verified target actuators:
 *   - X: SCHUNK Beta 100-ZRS toothed-belt linear actuator (200 mm/rev, 550 mm stroke, +/-0.08 mm repeatability)
 *   - Z: SCHUNK Beta 80-SRS ballscrew linear actuator (20 mm pitch, 150 mm stroke, +/-0.03 mm repeatability, 3000 rpm critical)
 *   - Theta: SCHUNK ERD 04-40-D-H-N miniature rotary unit (unlimited rotation, 600 rpm max, +/-0.01 deg)
 */

#ifndef AXIS_DRIVETRAIN_PARAMS_H
#define AXIS_DRIVETRAIN_PARAMS_H

#include "axis_pulse_motor_params.h"
#include "gantry_kconfig.h"

/* Kinematic values: idf.py menuconfig "Gantry kinematics" when building
 * firmware (CONFIG_*). Host tests without sdkconfig.h use the defaults. */

/* ---------------------------------------------------------------------------
 * Preprocessor mirrors of PulseMotor::DrivetrainType. A DT_ prefix is used to
 * avoid colliding with the enumerator names in PulseMotor.h (the enum is the
 * source of truth; these macros just make the AXIS_*_DRIVETRAIN selectors
 * readable at preprocessor time).
 * --------------------------------------------------------------------------- */
#define DT_BALLSCREW                        1
#define DT_BELT                             2
#define DT_RACKPINION                       3
#define DT_ROTARY_DIRECT                    4

/* ===========================================================================
 * X AXIS  -  SCHUNK Beta 100-ZRS toothed-belt linear actuator
 * ===========================================================================
 * Datasheet: "Beta 100-ZRS-40AT10-200-550-1050-AK-AZ2-2EO10-1ES10-6BL2-1-MGK2-GS24"
 *   stroke per round: 200 mm  -> AXIS_X_LEAD_MM_PER_REV
 *   mech. stroke:     550 mm  -> AXIS_X_HARD_LIMIT_MAX_MM
 *   repeatability:    +/-0.08 mm
 * Belt geometry breakdown (informational; lead_per_rev is authoritative at
 * runtime): AT10 profile, 10 mm tooth pitch, 20-tooth pulley -> 200 mm/rev. */
#define AXIS_X_DRIVETRAIN                   DT_BELT
#if defined(CONFIG_AXIS_X_LEAD_MM_PER_REV)
#define AXIS_X_LEAD_MM_PER_REV             ((float)CONFIG_AXIS_X_LEAD_MM_PER_REV)
#else
#define AXIS_X_LEAD_MM_PER_REV             200.0f
#endif
#if defined(CONFIG_AXIS_X_HARD_LIMIT_MIN_MM)
#define AXIS_X_HARD_LIMIT_MIN_MM           ((float)CONFIG_AXIS_X_HARD_LIMIT_MIN_MM)
#else
#define AXIS_X_HARD_LIMIT_MIN_MM           0.0f
#endif
#if defined(CONFIG_AXIS_X_HARD_LIMIT_MAX_MM)
#define AXIS_X_HARD_LIMIT_MAX_MM           ((float)CONFIG_AXIS_X_HARD_LIMIT_MAX_MM)
#else
#define AXIS_X_HARD_LIMIT_MAX_MM           550.0f
#endif
/* Commissioning documentation only — not motion-tuning knobs. Console /
 * Gantry use GANTRY_PATH_MAX_* for the shared 2-D path envelope; per-axis
 * Absolute components are derived from the path resultant. */
#if defined(CONFIG_AXIS_X_MAX_SPEED_MM_PER_S)
#define AXIS_X_MAX_SPEED_MM_PER_S          ((float)CONFIG_AXIS_X_MAX_SPEED_MM_PER_S)
#else
#define AXIS_X_MAX_SPEED_MM_PER_S          500.0f  /* placeholder; refine from Trap Moves */
#endif
#if defined(CONFIG_AXIS_X_ACCEL_MM_PER_S2)
#define AXIS_X_ACCEL_MM_PER_S2             ((float)CONFIG_AXIS_X_ACCEL_MM_PER_S2)
#else
#define AXIS_X_ACCEL_MM_PER_S2             3000.0f  /* placeholder; not a console ceiling */
#endif
#if defined(CONFIG_AXIS_X_DECEL_MM_PER_S2)
#define AXIS_X_DECEL_MM_PER_S2             ((float)CONFIG_AXIS_X_DECEL_MM_PER_S2)
#else
#define AXIS_X_DECEL_MM_PER_S2             3000.0f  /* placeholder; not a console ceiling */
#endif
#if defined(CONFIG_AXIS_X_POSITION_TOLERANCE_MM)
#define AXIS_X_POSITION_TOLERANCE_MM       ((float)CONFIG_AXIS_X_POSITION_TOLERANCE_MM)
#else
#define AXIS_X_POSITION_TOLERANCE_MM       0.08f  /* per datasheet repeatability */
#endif

/* ===========================================================================
 * Z AXIS  -  SCHUNK Beta 80-SRS ballscrew linear actuator (vertical descent)
 * ===========================================================================
 * Was historically labelled "Y" in pre-2026-05 firmware; the vertical actuator
 * is now named Z to free the Y symbol for the conveyor along-belt direction.
 *
 * Datasheet: "Beta 80-SRS-M-2020-150-530-2EO10-1ES10-4NS3-1-MGK1-GS19"
 *   screw pitch:      20 mm    -> AXIS_Z_LEAD_MM_PER_REV
 *   thread dia:       20 mm    (informational)
 *   critical speed:   3000 rpm -> AXIS_Z_CRITICAL_RPM (whip-speed ceiling)
 *   mech. stroke:     150 mm   -> AXIS_Z_HARD_LIMIT_MAX_MM
 *   axial tolerance:  0.08 mm; repeat accuracy: +/-0.03 mm
 *
 * Sign convention: +Z = up. Joint Z=0 is the homing datum (see
 * GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM for the physical belt/bed plane).
 * Z_HARD_LIMIT_MAX_MM is the top of stroke and the safe-retracted height. */
#define AXIS_Z_DRIVETRAIN                   DT_BALLSCREW
#if defined(CONFIG_AXIS_Z_LEAD_MM_PER_REV)
#define AXIS_Z_LEAD_MM_PER_REV             ((float)CONFIG_AXIS_Z_LEAD_MM_PER_REV)
#else
#define AXIS_Z_LEAD_MM_PER_REV             20.0f
#endif
#if defined(CONFIG_AXIS_Z_CRITICAL_RPM)
#define AXIS_Z_CRITICAL_RPM                ((uint32_t)CONFIG_AXIS_Z_CRITICAL_RPM)
#else
#define AXIS_Z_CRITICAL_RPM                3000u
#endif
#if defined(CONFIG_AXIS_Z_HARD_LIMIT_MIN_MM)
#define AXIS_Z_HARD_LIMIT_MIN_MM           ((float)CONFIG_AXIS_Z_HARD_LIMIT_MIN_MM)
#else
#define AXIS_Z_HARD_LIMIT_MIN_MM           0.0f
#endif
#if defined(CONFIG_AXIS_Z_HARD_LIMIT_MAX_MM)
#define AXIS_Z_HARD_LIMIT_MAX_MM           ((float)CONFIG_AXIS_Z_HARD_LIMIT_MAX_MM)
#else
#define AXIS_Z_HARD_LIMIT_MAX_MM           150.0f
#endif
/* Commissioning documentation only — not motion-tuning knobs. See
 * GANTRY_PATH_MAX_* for the shared 2-D path envelope. Z critical-RPM
 * derived speed cap is still applied as a protective clamp. */
#if defined(CONFIG_AXIS_Z_MAX_SPEED_MM_PER_S)
#define AXIS_Z_MAX_SPEED_MM_PER_S          ((float)CONFIG_AXIS_Z_MAX_SPEED_MM_PER_S)
#else
#define AXIS_Z_MAX_SPEED_MM_PER_S          500.0f  /* clamped below by critical-speed derived cap */
#endif
#if defined(CONFIG_AXIS_Z_ACCEL_MM_PER_S2)
#define AXIS_Z_ACCEL_MM_PER_S2             ((float)CONFIG_AXIS_Z_ACCEL_MM_PER_S2)
#else
#define AXIS_Z_ACCEL_MM_PER_S2             5000.0f  /* placeholder; not a console ceiling */
#endif
#if defined(CONFIG_AXIS_Z_DECEL_MM_PER_S2)
#define AXIS_Z_DECEL_MM_PER_S2             ((float)CONFIG_AXIS_Z_DECEL_MM_PER_S2)
#else
#define AXIS_Z_DECEL_MM_PER_S2             5000.0f  /* placeholder; not a console ceiling */
#endif
#if defined(CONFIG_AXIS_Z_POSITION_TOLERANCE_MM)
#define AXIS_Z_POSITION_TOLERANCE_MM       ((float)CONFIG_AXIS_Z_POSITION_TOLERANCE_MM)
#else
#define AXIS_Z_POSITION_TOLERANCE_MM       0.03f
#endif

/*
 * Physical belt / bed vs firmware Z datum (mm, +Z up).
 *
 * After homing, joint Z=0 sits at a mechanical reference (e.g. MIN limit),
 * which is usually offset from the actual belt or bed contact plane.
 *   GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM = distance from the belt/bed contact
 *   plane to the firmware joint Z datum, measured along +Z (positive means
 *   the Z=0 datum lies above the bed; the bed is at joint z = -this value).
 *
 * Forward kinematics sets pose.z = joint.z + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM
 * so pose.z reads as TCP height above the physical bed when this is non-zero.
 * Default 0 keeps legacy behaviour (datum coincident with the bed plane).
 */
#if defined(CONFIG_GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM)
#define GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM  ((float)CONFIG_GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM)
#elif !defined(GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM)
#define GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM  0.0f
#endif

/* ===========================================================================
 * THETA AXIS  -  SCHUNK ERD 04-40-D-H-N miniature rotary unit
 * ===========================================================================
 * Datasheet (product page 000000000000331220):
 *   angle of rotation: > 360 deg (unlimited at the hardware level)
 *   nominal torque: 0.4 Nm; peak torque: 1.2 Nm
 *   max rotational speed: 600 rpm = 3600 deg/s
 *   repeat accuracy: +/-0.01 deg
 *   max permissible mass moment of inertia: 0.008 kg*m^2
 *   max Fx/Fz: 150 N; max My: 2.5 Nm
 * The rotary hardware allows unlimited rotation; the HARD_LIMIT values below
 * are firmware clamps that keep the gripper cabling from winding up and are
 * therefore treated as the axis's effective hard envelope. */
#define AXIS_THETA_DRIVETRAIN               DT_ROTARY_DIRECT
#if defined(CONFIG_AXIS_THETA_OUTPUT_GEAR_RATIO)
#define AXIS_THETA_OUTPUT_GEAR_RATIO       ((float)CONFIG_AXIS_THETA_OUTPUT_GEAR_RATIO)
#else
#define AXIS_THETA_OUTPUT_GEAR_RATIO       1.0f  /* output_rev / motor_rev; 1.0 = direct drive */
#endif
#if defined(CONFIG_AXIS_THETA_HARD_LIMIT_MIN_DEG)
#define AXIS_THETA_HARD_LIMIT_MIN_DEG      ((float)CONFIG_AXIS_THETA_HARD_LIMIT_MIN_DEG)
#else
#define AXIS_THETA_HARD_LIMIT_MIN_DEG      -180.0f
#endif
#if defined(CONFIG_AXIS_THETA_HARD_LIMIT_MAX_DEG)
#define AXIS_THETA_HARD_LIMIT_MAX_DEG      ((float)CONFIG_AXIS_THETA_HARD_LIMIT_MAX_DEG)
#else
#define AXIS_THETA_HARD_LIMIT_MAX_DEG      180.0f
#endif
#if defined(CONFIG_AXIS_THETA_MAX_SPEED_DEG_PER_S)
#define AXIS_THETA_MAX_SPEED_DEG_PER_S     ((float)CONFIG_AXIS_THETA_MAX_SPEED_DEG_PER_S)
#else
#define AXIS_THETA_MAX_SPEED_DEG_PER_S     3600.0f  /* 600 rpm datasheet cap */
#endif
#if defined(CONFIG_AXIS_THETA_ACCEL_DEG_PER_S2)
#define AXIS_THETA_ACCEL_DEG_PER_S2        ((float)CONFIG_AXIS_THETA_ACCEL_DEG_PER_S2)
#else
#define AXIS_THETA_ACCEL_DEG_PER_S2        18000.0f  /* placeholder; respect inertia limit */
#endif
#if defined(CONFIG_AXIS_THETA_DECEL_DEG_PER_S2)
#define AXIS_THETA_DECEL_DEG_PER_S2        ((float)CONFIG_AXIS_THETA_DECEL_DEG_PER_S2)
#else
#define AXIS_THETA_DECEL_DEG_PER_S2        18000.0f  /* placeholder */
#endif
#if defined(CONFIG_AXIS_THETA_POSITION_TOLERANCE_DEG)
#define AXIS_THETA_POSITION_TOLERANCE_DEG  ((float)CONFIG_AXIS_THETA_POSITION_TOLERANCE_DEG)
#else
#define AXIS_THETA_POSITION_TOLERANCE_DEG  0.01f
#endif

/* Datasheet envelope - not consumed by firmware, retained for commissioning. */
#define AXIS_THETA_NOMINAL_TORQUE_NM          0.4f
#define AXIS_THETA_PEAK_TORQUE_NM             1.2f
#define AXIS_THETA_MAX_INERTIA_KGM2         0.008f
#define AXIS_THETA_MAX_AXIAL_FORCE_N        150.0f
#define AXIS_THETA_MAX_MOMENT_NM              2.5f

/* ===========================================================================
 * Gantry-level geometry and end-effector timing
 * ===========================================================================
 * Kinematic offsets (end-effector geometry). THE VALUES BELOW ARE THE
 * DEVELOPMENT RIG ASSEMBLY - they MUST be re-measured and overwritten against
 * the frozen production design before the firmware can command correct TCP
 * poses. See the "Geometry freeze gate" block a few lines down for the
 * attestation macro that silences the reminder once the design is frozen. */
/* Kinematic offsets in the new X / Y(=along-belt, -Y downstream) / Z(=up)
 * frame. Semantics (unchanged numbers from the development rig):
 *   GANTRY_Z_AXIS_Y_OFFSET_MM     - along-belt offset of the Z-column from
 *                                   the X beam datum (formerly Y_AXIS_Z_OFFSET).
 *   GANTRY_THETA_X_OFFSET_MM      - X offset of the theta module from the
 *                                   Z column.
 *   GANTRY_GRIPPER_X_OFFSET_MM    - X offset of the gripper TCP from the
 *                                   theta module (formerly GRIPPER_Y_OFFSET).
 *   GANTRY_GRIPPER_Z_OFFSET_MM    - Z offset of the gripper TCP from the
 *                                   theta module mounting flange.
 *   GANTRY_SAFE_Z_HEIGHT_MM       - margin below Z+ (mm) for coordinated X+Z
 *                                   (band floor = z_max - this).
 *   GANTRY_CAL_X_PARK_MM          - X park before Z+ seek during bring-up. */
#if defined(CONFIG_GANTRY_Z_AXIS_Y_OFFSET_MM)
#define GANTRY_Z_AXIS_Y_OFFSET_MM          ((float)CONFIG_GANTRY_Z_AXIS_Y_OFFSET_MM)
#else
#define GANTRY_Z_AXIS_Y_OFFSET_MM          80.0f
#endif
#if defined(CONFIG_GANTRY_THETA_X_OFFSET_MM)
#define GANTRY_THETA_X_OFFSET_MM           ((float)CONFIG_GANTRY_THETA_X_OFFSET_MM)
#else
#define GANTRY_THETA_X_OFFSET_MM           -55.0f
#endif
#if defined(CONFIG_GANTRY_GRIPPER_X_OFFSET_MM)
#define GANTRY_GRIPPER_X_OFFSET_MM         ((float)CONFIG_GANTRY_GRIPPER_X_OFFSET_MM)
#else
#define GANTRY_GRIPPER_X_OFFSET_MM         385.0f
#endif
#if defined(CONFIG_GANTRY_GRIPPER_Z_OFFSET_MM)
#define GANTRY_GRIPPER_Z_OFFSET_MM         ((float)CONFIG_GANTRY_GRIPPER_Z_OFFSET_MM)
#else
#define GANTRY_GRIPPER_Z_OFFSET_MM         80.0f
#endif
/* Traverse clearance margin above the Z− endstop / A015 (joint mm).
 * Coordinated X+Z Absolute is allowed only while Z is within this margin of
 * Z− (joint Z <= z_min + this). Above that bottom band Z moves alone with X
 * held. Limit codes: X A014=min/A015=max; Z A015=min (−Z)/A014=max (+Z).
 * Z Absolute joint sense is inverted so joint − seeks A015. */
#if defined(CONFIG_GANTRY_SAFE_Z_HEIGHT_MM)
#define GANTRY_SAFE_Z_HEIGHT_MM            ((float)CONFIG_GANTRY_SAFE_Z_HEIGHT_MM)
#else
#define GANTRY_SAFE_Z_HEIGHT_MM            30.0f
#endif

/* After X home/cal during bring-up, park here before seeking Z+ (joint mm). */
#if defined(CONFIG_GANTRY_CAL_X_PARK_MM)
#define GANTRY_CAL_X_PARK_MM               ((float)CONFIG_GANTRY_CAL_X_PARK_MM)
#else
#define GANTRY_CAL_X_PARK_MM               35.0f
#endif

/* Pneumatic gripper (SCHUNK KGG 100-80). Open/close times from datasheet. */
#if defined(CONFIG_GANTRY_GRIPPER_OPEN_TIME_MS)
#define GANTRY_GRIPPER_OPEN_TIME_MS        ((uint32_t)CONFIG_GANTRY_GRIPPER_OPEN_TIME_MS)
#else
#define GANTRY_GRIPPER_OPEN_TIME_MS        190u
#endif
#if defined(CONFIG_GANTRY_GRIPPER_CLOSE_TIME_MS)
#define GANTRY_GRIPPER_CLOSE_TIME_MS       ((uint32_t)CONFIG_GANTRY_GRIPPER_CLOSE_TIME_MS)
#else
#define GANTRY_GRIPPER_CLOSE_TIME_MS       150u
#endif


/* Default 2-D path motion profile (console / Gantry boot).
 * speed / accel / decel are the RESULTANT along the X-Z path; per-axis
 * Absolute components are derived from path direction. Menuconfig: Gantry kinematics. */
#if defined(CONFIG_GANTRY_DEFAULT_SPEED_MM_PER_S)
#define GANTRY_DEFAULT_SPEED_MM_PER_S          ((uint32_t)CONFIG_GANTRY_DEFAULT_SPEED_MM_PER_S)
#else
#define GANTRY_DEFAULT_SPEED_MM_PER_S          50u
#endif
#if defined(CONFIG_GANTRY_DEFAULT_SPEED_DEG_PER_S)
#define GANTRY_DEFAULT_SPEED_DEG_PER_S         ((uint32_t)CONFIG_GANTRY_DEFAULT_SPEED_DEG_PER_S)
#else
#define GANTRY_DEFAULT_SPEED_DEG_PER_S         30u
#endif
#if defined(CONFIG_GANTRY_DEFAULT_ACCEL_MM_PER_S2)
#define GANTRY_DEFAULT_ACCEL_MM_PER_S2         ((uint32_t)CONFIG_GANTRY_DEFAULT_ACCEL_MM_PER_S2)
#else
#define GANTRY_DEFAULT_ACCEL_MM_PER_S2         3000u
#endif
#if defined(CONFIG_GANTRY_DEFAULT_DECEL_MM_PER_S2)
#define GANTRY_DEFAULT_DECEL_MM_PER_S2         ((uint32_t)CONFIG_GANTRY_DEFAULT_DECEL_MM_PER_S2)
#else
#define GANTRY_DEFAULT_DECEL_MM_PER_S2         3000u
#endif
#if defined(CONFIG_GANTRY_EIP_ASSUMED_STOP_DECEL_MM_S2)
#define GANTRY_EIP_ASSUMED_STOP_DECEL_MM_S2    ((double)CONFIG_GANTRY_EIP_ASSUMED_STOP_DECEL_MM_S2)
#else
#define GANTRY_EIP_ASSUMED_STOP_DECEL_MM_S2    300.0
#endif

/* Shared 2-D path envelope ceilings (console rangelimit). Not per-axis. */
#if defined(CONFIG_GANTRY_PATH_MAX_SPEED_MM_PER_S)
#define GANTRY_PATH_MAX_SPEED_MM_PER_S         ((uint32_t)CONFIG_GANTRY_PATH_MAX_SPEED_MM_PER_S)
#else
#define GANTRY_PATH_MAX_SPEED_MM_PER_S         500u
#endif
#if defined(CONFIG_GANTRY_PATH_MAX_ACCEL_MM_PER_S2)
#define GANTRY_PATH_MAX_ACCEL_MM_PER_S2        ((uint32_t)CONFIG_GANTRY_PATH_MAX_ACCEL_MM_PER_S2)
#else
#define GANTRY_PATH_MAX_ACCEL_MM_PER_S2        3000u
#endif
#if defined(CONFIG_GANTRY_PATH_MAX_DECEL_MM_PER_S2)
#define GANTRY_PATH_MAX_DECEL_MM_PER_S2        ((uint32_t)CONFIG_GANTRY_PATH_MAX_DECEL_MM_PER_S2)
#else
#define GANTRY_PATH_MAX_DECEL_MM_PER_S2        3000u
#endif

/* ---------------------------------------------------------------------------
 * Geometry freeze gate.
 *
 * The five GANTRY_*_OFFSET_*_MM / GANTRY_SAFE_Z_HEIGHT_MM values above are
 * placeholders from the development rig. Before deployment the application
 * engineer must:
 *   1. Measure the as-built adapter-plate offsets from the frozen production
 *      design drawings (or verify them on the physical assembly with a CMM /
 *      calibration fixture).
 *   2. Overwrite the five macros above with the measured values.
 *   3. Define GANTRY_GEOMETRY_FROZEN (as a compile definition, e.g.
 *      target_compile_definitions(... PUBLIC GANTRY_GEOMETRY_FROZEN) in the
 *      deployment CMake file, or as a `-D` flag) to attest the freeze and
 *      silence the reminder emitted below.
 *
 * Defining GANTRY_SUPPRESS_GEOMETRY_WARNING bypasses the reminder WITHOUT
 * asserting the freeze - use it only for CI and bring-up builds that are
 * knowingly running with dev-rig geometry.
 * --------------------------------------------------------------------------- */
/* The reminder is emitted only when AXIS_DRIVETRAIN_PARAMS_EMIT_WARNINGS is
 * defined by the including TU. Exactly one TU - the application entry point
 * src/main.cpp - defines it before including this header, which keeps the
 * build log from carrying a copy of the warning for every source file that
 * transitively pulls axis_drivetrain_params.h in. */
#if !defined(GANTRY_GEOMETRY_FROZEN) && \
    !defined(GANTRY_SUPPRESS_GEOMETRY_WARNING) && \
     defined(AXIS_DRIVETRAIN_PARAMS_EMIT_WARNINGS)
#  if defined(__GNUC__) || defined(__clang__)
#    warning "axis_drivetrain_params.h: gantry geometric offsets (GANTRY_Z_AXIS_Y_OFFSET_MM, GANTRY_THETA_X_OFFSET_MM, GANTRY_GRIPPER_X_OFFSET_MM, GANTRY_GRIPPER_Z_OFFSET_MM, GANTRY_SAFE_Z_HEIGHT_MM) are DEVELOPMENT-RIG placeholders. Re-measure against the frozen production design and define GANTRY_GEOMETRY_FROZEN to silence this reminder (or define GANTRY_SUPPRESS_GEOMETRY_WARNING for CI / bring-up builds)."
#  elif defined(_MSC_VER)
#    pragma message("axis_drivetrain_params.h: GANTRY_*_OFFSET_*_MM and GANTRY_SAFE_Z_HEIGHT_MM are DEVELOPMENT-RIG placeholders. Update per the frozen production design and #define GANTRY_GEOMETRY_FROZEN to silence.")
#  endif
#endif

/* ---------------------------------------------------------------------------
 * Derived helpers.
 * These are macros so they may be consumed where only the preprocessor runs.
 * Runtime code typically prefers PulseMotor::pulsesPerMm / pulsesPerDeg
 * applied to a DrivetrainConfig.
 * --------------------------------------------------------------------------- */

/* X: encoder pulses per mm of linear travel */
#define AXIS_X_PULSES_PER_MM \
    (((double)AXIS_X_ENCODER_PPR * (double)AXIS_X_MOTOR_REDUCER_RATIO) / (double)AXIS_X_LEAD_MM_PER_REV)

/* Z: encoder pulses per mm of linear travel */
#define AXIS_Z_PULSES_PER_MM \
    (((double)AXIS_Z_ENCODER_PPR * (double)AXIS_Z_MOTOR_REDUCER_RATIO) / (double)AXIS_Z_LEAD_MM_PER_REV)

/* Z: ballscrew speed cap derived from critical RPM. Never command a linear
 * speed that would put the screw above this RPM. */
#define AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S \
    (((double)AXIS_Z_CRITICAL_RPM / 60.0) * (double)AXIS_Z_LEAD_MM_PER_REV)

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
 *   #define AXIS_Z_REFLECTED_INERTIA_KGM2      ...
 *   #define AXIS_Z_REQUIRED_TORQUE_NM          ...
 *   #define AXIS_THETA_REFLECTED_INERTIA_KGM2  ...
 *   #define AXIS_THETA_REQUIRED_TORQUE_NM      ...
 *   #define GANTRY_CYCLE_TIME_TARGET_S         2.04
 * --------------------------------------------------------------------------- */

#endif /* AXIS_DRIVETRAIN_PARAMS_H */
