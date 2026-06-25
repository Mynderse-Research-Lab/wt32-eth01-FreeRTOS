/**
 * @file axis_pulse_motor_params.h
 * @brief Per-axis ELECTRICAL tuning for the PulseMotor drivers.
 *
 * Scope: motor + driver + optional gearbox. Values here come from:
 *   - The motor datasheet (resolution, nominal/peak torque, inertia).
 *   - The driver parameter sheet (pulse-input bandwidth, electronic gear, inversion flags).
 *   - The physical gearbox data sheet, if any.
 *
 * The companion header include/axis_drivetrain_params.h owns the MECHANICAL
 * side (ballscrew lead, belt lead-per-rev, rotary travel, motion envelopes).
 * Pin mapping and hardware-peripheral allocation stay in gantry_app_constants.h.
 *
 * Coordinate convention (firmware-wide):
 *   - X: horizontal traverse along the gantry beam (across the conveyor belt).
 *   - Y: along-belt direction; the gantry has NO Y actuator. Conveyor downstream
 *        is the -Y direction. Battery detections carry a y_bat_mm coordinate
 *        in this frame, consumed only by the pick scheduler / planner.
 *   - Z: vertical (gantry descent axis). +Z = up (away from belt). Joint Z=0
 *        is the homing datum; physical bed vs datum: GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM
 *        in axis_drivetrain_params.h. Top home / safe-retracted is Z = Z_max.
 *   - Theta: rotation about the vertical (Z) axis.
 *
 * Verified target drivers (see docs/MOTION_IO_INTERFACE.md):
 *   - X: Allen-Bradley Kinetix 5100 (PTI pulse+direction step mode)
 *   - Z: Allen-Bradley Kinetix 5100 (PTI pulse+direction step mode)
 *   - Theta: HCS01.1E-W0005 Step/Dir on X31 + SCHUNK ERD 04-40-D-H-N
 */

#ifndef AXIS_PULSE_MOTOR_PARAMS_H
#define AXIS_PULSE_MOTOR_PARAMS_H

/* Phase A bring-up: K5100 buffered encoder outputs not wired (no AM26LV32 yet).
 * Set to 1 after Phase B receiver is installed — see docs/MOTION_IO_INTERFACE.md */
#define AXIS_X_ENCODER_FEEDBACK_ENABLED          0
#define AXIS_Z_ENCODER_FEEDBACK_ENABLED          0

/* =======================================================================
 * X AXIS  -  Allen-Bradley Kinetix 5100 (PTI mode)
 * =======================================================================
 * Commissioning: set the K5100's electronic gear so AXIS_X_ENCODER_PPR
 * matches the number of pulses you want per motor revolution as seen at the
 * PCNT input. Max pulse frequency is governed by the K5100 PTI bandwidth. */
#define AXIS_X_ENCODER_PPR                 10000u
#define AXIS_X_MAX_PULSE_FREQ_HZ          200000u
#define AXIS_X_GEAR_NUMERATOR                 1.0
#define AXIS_X_GEAR_DENOMINATOR               1.0
#define AXIS_X_MOTOR_REDUCER_RATIO            1.0f
#define AXIS_X_INVERT_DIR                        1
#define AXIS_X_INVERT_OUTPUT_LOGIC               1
#define AXIS_X_LEDC_RESOLUTION_BITS              2u
#define AXIS_X_HOMING_SPEED_PPS              8000u
#define AXIS_X_LIMIT_DEBOUNCE_CYCLES           10u
#define AXIS_X_LIMIT_SAMPLE_INTERVAL_MS         3u

/* =======================================================================
 * Z AXIS  -  Allen-Bradley Kinetix 5100 (PTI mode)
 *
 * Vertical descent axis (was historically labelled "Y" in pre-2026-05
 * firmware). +Z = up; joint Z = 0 at homing datum (see
 * GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM for belt plane); Z_max at top / home /
 * safe-retracted position.
 * ======================================================================= */
#define AXIS_Z_ENCODER_PPR                 10000u
#define AXIS_Z_MAX_PULSE_FREQ_HZ          200000u
#define AXIS_Z_GEAR_NUMERATOR                 1.0
#define AXIS_Z_GEAR_DENOMINATOR               1.0
#define AXIS_Z_MOTOR_REDUCER_RATIO            1.0f
#define AXIS_Z_INVERT_DIR                        0
#define AXIS_Z_INVERT_OUTPUT_LOGIC               1
#define AXIS_Z_LEDC_RESOLUTION_BITS              2u
#define AXIS_Z_HOMING_SPEED_PPS              8000u
#define AXIS_Z_LIMIT_DEBOUNCE_CYCLES           10u
#define AXIS_Z_LIMIT_SAMPLE_INTERVAL_MS         3u

/* =======================================================================
 * THETA AXIS  -  Custom pulse-train driver + SCHUNK ERD 04-40-D-H-N
 * =======================================================================
 * The ERD 04-40-D-H-N is a 3-phase torque motor with HIPERFACE multiturn
 * absolute encoder. The custom driver consumes HIPERFACE internally and
 * presents pulse+direction input to the ESP32. The PPR seen at the PCNT is
 * the commissioning-configured electronic gear on the custom driver, not the
 * native HIPERFACE resolution.
 *
 * Default: 36000 ppr -> 0.01 deg/pulse (matches datasheet repeat accuracy).
 * Max speed: 600 rpm (datasheet) = 600 rev/min * 36000 ppr / 60 s = 360 kHz.
 * Homing speed is kept gentle because nominal torque is only 0.4 Nm. */
#define AXIS_THETA_ENCODER_PPR             36000u
#define AXIS_THETA_MAX_PULSE_FREQ_HZ      400000u
#define AXIS_THETA_GEAR_NUMERATOR             1.0
#define AXIS_THETA_GEAR_DENOMINATOR           1.0
#define AXIS_THETA_MOTOR_REDUCER_RATIO        1.0f
#define AXIS_THETA_INVERT_DIR                    0
#define AXIS_THETA_INVERT_OUTPUT_LOGIC           1
#define AXIS_THETA_LEDC_RESOLUTION_BITS          2u
#define AXIS_THETA_HOMING_SPEED_PPS          6000u
#define AXIS_THETA_LIMIT_DEBOUNCE_CYCLES       10u
#define AXIS_THETA_LIMIT_SAMPLE_INTERVAL_MS     3u

#endif /* AXIS_PULSE_MOTOR_PARAMS_H */
