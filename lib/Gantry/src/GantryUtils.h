/**
 * @file GantryUtils.h
 * @brief Utility functions and constants for Gantry library
 * @version 1.0.0
 */

#ifndef GANTRY_UTILS_H
#define GANTRY_UTILS_H

#include <stdint.h>

#include "axis_drivetrain_params.h"

namespace Gantry {

// ============================================================================
// CONSTANTS
// ============================================================================

namespace Constants {
    constexpr float DEFAULT_STEPS_PER_REV = 6000.0f;
    constexpr float DEFAULT_PULSES_PER_MM = 150.0f;
    // Safe retracted Z height (top of stroke; +Z = up). The gantry retracts to
    // this Z value before traversing in X / theta. Was DEFAULT_SAFE_Y_HEIGHT_MM
    // in pre-2026-05 firmware when the vertical axis was called "Y".
    constexpr float DEFAULT_SAFE_Z_HEIGHT_MM = GANTRY_SAFE_Z_HEIGHT_MM;
    constexpr uint32_t DEFAULT_HOMING_SPEED_PPS = 6000;
    constexpr uint32_t DEFAULT_SPEED_MM_PER_S = GANTRY_DEFAULT_SPEED_MM_PER_S;
    constexpr uint32_t DEFAULT_SPEED_DEG_PER_S = GANTRY_DEFAULT_SPEED_DEG_PER_S;
    constexpr uint32_t GRIPPER_ACTUATE_TIME_MS = 100;
    constexpr uint32_t CALIBRATION_TIMEOUT_MS = 30000;
    constexpr uint32_t TRAVEL_MEASUREMENT_TIMEOUT_MS = 90000;
} // namespace Constants

// ============================================================================
// HELPER MACROS
// ============================================================================

#define GANTRY_CHECK_INITIALIZED() \
    if (!initialized_) return

#define GANTRY_CHECK_ENABLED() \
    if (!enabled_) return

#define GANTRY_CHECK_INITIALIZED_RET(ret) \
    if (!initialized_) return ret

#define GANTRY_CHECK_ENABLED_RET(ret) \
    if (!enabled_) return ret

#define GANTRY_CHECK_BUSY_RET(ret) \
    if (isBusy()) return ret

} // namespace Gantry

#endif // GANTRY_UTILS_H
