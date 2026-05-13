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
 * Verified target drivers (see driver_datasheets_and_calculations/INDEX.md):
 *   - X: Allen-Bradley Kinetix 5100 (PTI pulse+direction step mode)
 *   - Y: Allen-Bradley Kinetix 5100 (PTI pulse+direction step mode)
 *   - Theta: Custom pulse-train driver for SCHUNK ERD 04-40-D-H-N
 */

#ifndef AXIS_PULSE_MOTOR_PARAMS_H
#define AXIS_PULSE_MOTOR_PARAMS_H

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
 * Y AXIS  -  Allen-Bradley Kinetix 5100 (PTI mode)
 * ======================================================================= */
#define AXIS_Y_ENCODER_PPR                 10000u
#define AXIS_Y_MAX_PULSE_FREQ_HZ          200000u
#define AXIS_Y_GEAR_NUMERATOR                 1.0
#define AXIS_Y_GEAR_DENOMINATOR               1.0
#define AXIS_Y_MOTOR_REDUCER_RATIO            1.0f
#define AXIS_Y_INVERT_DIR                        0
#define AXIS_Y_INVERT_OUTPUT_LOGIC               1
#define AXIS_Y_LEDC_RESOLUTION_BITS              2u
#define AXIS_Y_HOMING_SPEED_PPS              8000u
#define AXIS_Y_LIMIT_DEBOUNCE_CYCLES           10u
#define AXIS_Y_LIMIT_SAMPLE_INTERVAL_MS         3u

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
