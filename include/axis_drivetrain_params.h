#ifndef AXIS_DRIVETRAIN_PARAMS_H
#define AXIS_DRIVETRAIN_PARAMS_H

#include "axis_pulse_motor_params.h"

/*
 * Per-axis drivetrain parameters consumed by main.cpp when populating a
 * PulseMotor::DrivetrainConfig for each axis. The drivetrain *category*
 * (ballscrew / belt / rack-and-pinion / rotary direct) is selected in code via
 * the PulseMotor::DrivetrainType enum; the numeric values below feed its
 * type-specific fields.
 */

// X axis (SCHUNK Beta 100-ZRS toothed belt, 200 mm/rev)
#define AXIS_X_BELT_LEAD_MM_PER_REV 200.0f
#define AXIS_X_TRAVEL_MIN_MM 0.0f
#define AXIS_X_TRAVEL_MAX_MM 550.0f
#define AXIS_X_MAX_SPEED_MM_PER_S 500.0f
#define AXIS_X_ACCEL_MM_PER_S2 3000.0f
#define AXIS_X_DECEL_MM_PER_S2 3000.0f
#define AXIS_X_POSITION_TOLERANCE_MM 0.08f

// Y axis (SCHUNK Beta 80-SRS ballscrew, 20 mm lead)
#define AXIS_Y_BALLSCREW_LEAD_MM 20.0f
#define AXIS_Y_BALLSCREW_CRITICAL_RPM 3000
#define AXIS_Y_TRAVEL_MIN_MM 0.0f
#define AXIS_Y_TRAVEL_MAX_MM 150.0f
#define AXIS_Y_MAX_SPEED_MM_PER_S 500.0f
#define AXIS_Y_ACCEL_MM_PER_S2 5000.0f
#define AXIS_Y_DECEL_MM_PER_S2 5000.0f
#define AXIS_Y_POSITION_TOLERANCE_MM 0.03f

// Theta axis (SCHUNK ERD 04-40-D-H-N, rotary direct)
#define AXIS_THETA_OUTPUT_GEAR_RATIO 1.0f
#define AXIS_THETA_TRAVEL_MIN_DEG -180.0f
#define AXIS_THETA_TRAVEL_MAX_DEG 180.0f
#define AXIS_THETA_MAX_SPEED_DEG_PER_S 3600.0f
#define AXIS_THETA_ACCEL_DEG_PER_S2 18000.0f
#define AXIS_THETA_DECEL_DEG_PER_S2 18000.0f
#define AXIS_THETA_POSITION_TOLERANCE_DEG 0.01f

// Gantry geometry and actuator timing
#define GANTRY_Y_AXIS_Z_OFFSET_MM 80.0f
#define GANTRY_THETA_X_OFFSET_MM -55.0f
#define GANTRY_GRIPPER_Y_OFFSET_MM 385.0f
#define GANTRY_GRIPPER_Z_OFFSET_MM 80.0f
#define GANTRY_SAFE_Y_HEIGHT_MM 150.0f
#define GANTRY_GRIPPER_OPEN_TIME_MS 190
#define GANTRY_GRIPPER_CLOSE_TIME_MS 150

// Derived helpers
#define AXIS_X_PULSES_PER_MM \
  ((AXIS_X_ENCODER_PPR * AXIS_X_MOTOR_REDUCER_RATIO) / AXIS_X_BELT_LEAD_MM_PER_REV)

#define AXIS_Y_PULSES_PER_MM \
  ((AXIS_Y_ENCODER_PPR * AXIS_Y_MOTOR_REDUCER_RATIO) / AXIS_Y_BALLSCREW_LEAD_MM)

#define AXIS_Y_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S \
  ((AXIS_Y_BALLSCREW_CRITICAL_RPM / 60.0f) * AXIS_Y_BALLSCREW_LEAD_MM)

#define AXIS_THETA_PULSES_PER_DEG \
  ((AXIS_THETA_ENCODER_PPR * AXIS_THETA_MOTOR_REDUCER_RATIO * AXIS_THETA_OUTPUT_GEAR_RATIO) / 360.0f)

#endif  // AXIS_DRIVETRAIN_PARAMS_H
