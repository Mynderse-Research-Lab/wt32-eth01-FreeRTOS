#ifndef CONVEYOR_INTERCEPT_PARAMS_H
#define CONVEYOR_INTERCEPT_PARAMS_H

#include "axis_drivetrain_params.h"

// Along-belt coordinates in world Y (+Y = conveyor downstream).
// These are commissioning placeholders and must be validated on hardware.
#if defined(CONFIG_CONVEYOR_Y_CAM_MM)
#define CONVEYOR_Y_CAM_MM                       ((float)CONFIG_CONVEYOR_Y_CAM_MM)
#else
#define CONVEYOR_Y_CAM_MM                       (-711.0f)
#endif

#if defined(CONFIG_CONVEYOR_Y_PICK_MM)
#define CONVEYOR_Y_PICK_MM                      ((float)CONFIG_CONVEYOR_Y_PICK_MM)
#else
#define CONVEYOR_Y_PICK_MM                      (0.0f)
#endif

// Across-belt mapping: gantry joint X = x_across + offset.
// The camera is at the belt center: collision edge 95 mm + 150 mm.
#if defined(CONFIG_CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM)
#define CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM ((float)CONFIG_CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM)
#else
#define CONVEYOR_X_ACROSS_TO_GANTRY_X_OFFSET_MM (245.0f)
#endif

#if defined(CONFIG_CONVEYOR_BELT_WIDTH_MM)
#define CONVEYOR_BELT_WIDTH_MM                  ((float)CONFIG_CONVEYOR_BELT_WIDTH_MM)
#else
#define CONVEYOR_BELT_WIDTH_MM                  (300.0f)
#endif

// Joint-space safe height for Gantry::moveTo(JointConfig).
// SAFE aliases GANTRY_SAFE_Z_HEIGHT_MM = retract/traverse band from Z−.
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
