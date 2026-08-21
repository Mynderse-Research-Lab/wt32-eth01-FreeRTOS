/**
 * @file axis_pulse_motor_params.h
 * @brief Per-axis ELECTRICAL tuning for the PulseMotor drivers.
 *
 * Scope: motor + driver + optional gearbox. Values here come from:
 *   - The motor datasheet (resolution, nominal/peak torque, inertia).
 *   - The driver parameter sheet (pulse-input bandwidth, electronic gear, inversion flags).
 *   - The physical gearbox data sheet, if any.
 *   - idf.py menuconfig "Gantry kinematics" (CONFIG_AXIS_* when building firmware).
 *
 * The companion header include/axis_drivetrain_params.h owns the MECHANICAL
 * side (ballscrew lead, belt lead-per-rev, rotary travel, motion envelopes).
 * Pin mapping and hardware-peripheral allocation stay in gantry_app_constants.h.
 */

#ifndef AXIS_PULSE_MOTOR_PARAMS_H
#define AXIS_PULSE_MOTOR_PARAMS_H

#include "gantry_kconfig.h"

/* Phase A bring-up: K5100 buffered encoder outputs not wired (no AM26LV32 yet).
 * Set to 1 after Phase B receiver is installed — encoder path is deferred;
 * production feedback is EtherNet/IP assembly actual position. */
#define AXIS_X_ENCODER_FEEDBACK_ENABLED          0
#define AXIS_Z_ENCODER_FEEDBACK_ENABLED          0

/* =======================================================================
 * X AXIS  -  Allen-Bradley Kinetix 5100 (PTI mode)
 * ======================================================================= */
#if defined(CONFIG_AXIS_X_ENCODER_PPR)
#define AXIS_X_ENCODER_PPR                 ((uint32_t)CONFIG_AXIS_X_ENCODER_PPR)
#else
#define AXIS_X_ENCODER_PPR                 2097152u
#endif
#define AXIS_X_MAX_PULSE_FREQ_HZ          200000u
#define AXIS_X_GEAR_NUMERATOR                 1.0
#define AXIS_X_GEAR_DENOMINATOR               1.0
#if defined(CONFIG_AXIS_X_MOTOR_REDUCER_RATIO)
#define AXIS_X_MOTOR_REDUCER_RATIO            ((float)CONFIG_AXIS_X_MOTOR_REDUCER_RATIO)
#else
#define AXIS_X_MOTOR_REDUCER_RATIO            5.0f
#endif
#define AXIS_X_INVERT_DIR                        1
#define AXIS_X_INVERT_OUTPUT_LOGIC               1
#define AXIS_X_LEDC_RESOLUTION_BITS              2u
#if defined(CONFIG_AXIS_X_HOMING_SPEED_PPS)
#define AXIS_X_HOMING_SPEED_PPS              ((uint32_t)CONFIG_AXIS_X_HOMING_SPEED_PPS)
#else
#define AXIS_X_HOMING_SPEED_PPS              4000u
#endif
#define AXIS_X_LIMIT_DEBOUNCE_CYCLES           10u
#define AXIS_X_LIMIT_SAMPLE_INTERVAL_MS         3u

/* =======================================================================
 * Z AXIS  -  Allen-Bradley Kinetix 5100 (EIP), direct drive, no holding brake
 * ======================================================================= */
#if defined(CONFIG_AXIS_Z_ENCODER_PPR)
#define AXIS_Z_ENCODER_PPR                 ((uint32_t)CONFIG_AXIS_Z_ENCODER_PPR)
#else
#define AXIS_Z_ENCODER_PPR                 2097152u
#endif
#define AXIS_Z_MAX_PULSE_FREQ_HZ          200000u
#define AXIS_Z_GEAR_NUMERATOR                 1.0
#define AXIS_Z_GEAR_DENOMINATOR               1.0
#if defined(CONFIG_AXIS_Z_MOTOR_REDUCER_RATIO)
#define AXIS_Z_MOTOR_REDUCER_RATIO            ((float)CONFIG_AXIS_Z_MOTOR_REDUCER_RATIO)
#else
#define AXIS_Z_MOTOR_REDUCER_RATIO            1.0f  /* direct drive (no gearbox) */
#endif
#if defined(CONFIG_AXIS_Z_HAS_MOTOR_BRAKE)
#define AXIS_Z_HAS_MOTOR_BRAKE                1
#else
#define AXIS_Z_HAS_MOTOR_BRAKE                0
#endif
#define AXIS_Z_INVERT_DIR                        0
#define AXIS_Z_INVERT_OUTPUT_LOGIC               1
#define AXIS_Z_LEDC_RESOLUTION_BITS              2u
#if defined(CONFIG_AXIS_Z_HOMING_SPEED_PPS)
#define AXIS_Z_HOMING_SPEED_PPS              ((uint32_t)CONFIG_AXIS_Z_HOMING_SPEED_PPS)
#else
#define AXIS_Z_HOMING_SPEED_PPS              4000u
#endif
#define AXIS_Z_LIMIT_DEBOUNCE_CYCLES           10u
#define AXIS_Z_LIMIT_SAMPLE_INTERVAL_MS         3u

/* =======================================================================
 * THETA AXIS  -  Custom pulse-train driver + SCHUNK ERD 04-40-D-H-N
 * ======================================================================= */
#if defined(CONFIG_AXIS_THETA_ENCODER_PPR)
#define AXIS_THETA_ENCODER_PPR             ((uint32_t)CONFIG_AXIS_THETA_ENCODER_PPR)
#else
#define AXIS_THETA_ENCODER_PPR             36000u
#endif
#define AXIS_THETA_MAX_PULSE_FREQ_HZ      400000u
#define AXIS_THETA_GEAR_NUMERATOR             1.0
#define AXIS_THETA_GEAR_DENOMINATOR           1.0
#if defined(CONFIG_AXIS_THETA_MOTOR_REDUCER_RATIO)
#define AXIS_THETA_MOTOR_REDUCER_RATIO        ((float)CONFIG_AXIS_THETA_MOTOR_REDUCER_RATIO)
#else
#define AXIS_THETA_MOTOR_REDUCER_RATIO        1.0f
#endif
#define AXIS_THETA_INVERT_DIR                    0
#define AXIS_THETA_INVERT_OUTPUT_LOGIC           1
#define AXIS_THETA_LEDC_RESOLUTION_BITS          2u
#if defined(CONFIG_AXIS_THETA_HOMING_SPEED_PPS)
#define AXIS_THETA_HOMING_SPEED_PPS          ((uint32_t)CONFIG_AXIS_THETA_HOMING_SPEED_PPS)
#else
#define AXIS_THETA_HOMING_SPEED_PPS          3000u
#endif
#define AXIS_THETA_LIMIT_DEBOUNCE_CYCLES       10u
#define AXIS_THETA_LIMIT_SAMPLE_INTERVAL_MS     3u

#endif /* AXIS_PULSE_MOTOR_PARAMS_H */
