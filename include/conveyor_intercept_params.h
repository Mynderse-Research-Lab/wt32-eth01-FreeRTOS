#ifndef CONVEYOR_INTERCEPT_PARAMS_H
#define CONVEYOR_INTERCEPT_PARAMS_H

#include "axis_drivetrain_params.h"

// Along-belt coordinates in world Y (+Y = conveyor downstream).
// These are commissioning placeholders and must be validated on hardware.
#if defined(CONFIG_CONVEYOR_Y_CAM_MM)
#define CONVEYOR_Y_CAM_MM                       ((float)CONFIG_CONVEYOR_Y_CAM_MM)
#else
#define CONVEYOR_Y_CAM_MM                       (336.55f)
#endif

#if defined(CONFIG_CONVEYOR_Y_PICK_MM)
#define CONVEYOR_Y_PICK_MM                      ((float)CONFIG_CONVEYOR_Y_PICK_MM)
#else
#define CONVEYOR_Y_PICK_MM                      (1016.0f)
#endif

// Across-belt mapping: gantry joint X = x_across + offset.
#if defined(CONFIG_CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM)
#define CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM ((float)CONFIG_CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM)
#else
#define CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM (0.0f)
#endif

// Joint-space pick heights for Gantry::moveTo(JointConfig).
// SAFE aliases GANTRY_SAFE_Z_HEIGHT_MM = retract/traverse band from Z−.
// PICK is toward the belt (+Z down, near A014) — placeholder, re-measure.
#if defined(CONFIG_CONVEYOR_Z_PICK_JOINT_MM)
#define CONVEYOR_Z_PICK_JOINT_MM                ((float)CONFIG_CONVEYOR_Z_PICK_JOINT_MM)
#else
#define CONVEYOR_Z_PICK_JOINT_MM                (140.0f)
#endif
#define CONVEYOR_Z_SAFE_JOINT_MM                (GANTRY_SAFE_Z_HEIGHT_MM)

// Time-to-pick limits from SRS (vision-scheduled tau_us).
#if defined(CONFIG_TAU_MIN_US)
#define TAU_MIN_US                              ((uint64_t)CONFIG_TAU_MIN_US)
#else
#define TAU_MIN_US                              (100000ULL)
#endif

#if defined(CONFIG_TAU_MAX_US)
#define TAU_MAX_US                              ((uint64_t)CONFIG_TAU_MAX_US)
#else
#define TAU_MAX_US                              (5000000ULL)
#endif

#if !defined(CONVEYOR_INTERCEPT_FROZEN) && defined(AXIS_DRIVETRAIN_PARAMS_EMIT_WARNINGS)
#  if defined(__GNUC__) || defined(__clang__)
#    warning "conveyor_intercept_params.h: CONVEYOR_* and TAU_* values are deployment placeholders; calibrate and define CONVEYOR_INTERCEPT_FROZEN."
#  endif
#endif

#endif  // CONVEYOR_INTERCEPT_PARAMS_H
