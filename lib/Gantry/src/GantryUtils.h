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
    // Margin above Z- (mm). Coordinated X+Z / X traverse only while
    // joint Z >= z_max - this margin. Was DEFAULT_SAFE_Y_HEIGHT_MM.
    constexpr float DEFAULT_SAFE_Z_HEIGHT_MM = GANTRY_SAFE_Z_HEIGHT_MM;
    constexpr uint32_t DEFAULT_HOMING_SPEED_PPS = 3000;
    constexpr uint32_t DEFAULT_SPEED_MM_PER_S = GANTRY_DEFAULT_SPEED_MM_PER_S;
    constexpr uint32_t DEFAULT_SPEED_DEG_PER_S = GANTRY_DEFAULT_SPEED_DEG_PER_S;
    constexpr uint32_t GRIPPER_ACTUATE_TIME_MS = GANTRY_GRIPPER_OPEN_TIME_MS;
    constexpr uint32_t CALIBRATION_TIMEOUT_MS = 20000;
    constexpr uint32_t TRAVEL_MEASUREMENT_TIMEOUT_MS = 20000;
    // Whole EIP bring-up (several 20 s phases plus theta at 15 deg/s).
    constexpr uint32_t BRINGUP_SEQUENCE_TIMEOUT_MS = 120000;
    // Home / calibrate / bring-up seek, park, and SAFE_Z return. Independent
    // of console speed/accel. Creep toward switch-clear stays slower.
    constexpr float EIP_HOME_CAL_SPEED_MM_S = 50.0f;
    constexpr float EIP_HOME_THETA_SPEED_DEG_S = 30.0f;
    // Theta pose (mod 360) held during the Z+ stroke measurement in bring-up.
    constexpr float BRINGUP_THETA_ORIENT_DEG = 90.0f;
    // Theta home / bring-up orient counts as arrived within this of target.
    constexpr float EIP_THETA_SETTLED_TOL_DEG = 0.1f;
    constexpr float EIP_HOME_CAL_ACCEL_MM_S2 = 2000.0f;
    constexpr float EIP_CREEP_SPEED_MM_S = 1.0f;
} // namespace Constants

/**
 * @struct RuntimeParams
 * @brief Stored parameters the library applies. gantryRuntimeParams() fills this
 * from AppConfig; these member defaults only serve builds with no parameter store.
 */
struct RuntimeParams {
    float z_speed_cap_mm_s = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
    float z_accel_cap_mm_s2 = 0.0f;
    float z_decel_cap_mm_s2 = 0.0f;
    float theta_max_speed_deg_s = AXIS_THETA_MAX_SPEED_DEG_PER_S;
    float theta_accel_deg_s2 = AXIS_THETA_ACCEL_DEG_PER_S2;
    float theta_decel_deg_s2 = AXIS_THETA_DECEL_DEG_PER_S2;
    uint32_t gripper_open_ms = GANTRY_GRIPPER_OPEN_TIME_MS;
    uint32_t gripper_close_ms = GANTRY_GRIPPER_CLOSE_TIME_MS;
    float x_hard_span_mm = AXIS_X_HARD_LIMIT_MAX_MM - AXIS_X_HARD_LIMIT_MIN_MM;
    float home_speed_mm_s = Constants::EIP_HOME_CAL_SPEED_MM_S;
    float home_accel_mm_s2 = Constants::EIP_HOME_CAL_ACCEL_MM_S2;
    float home_theta_speed_deg_s = Constants::EIP_HOME_THETA_SPEED_DEG_S;
    float creep_speed_mm_s = Constants::EIP_CREEP_SPEED_MM_S;
    float z_axis_y_offset_mm = GANTRY_Z_AXIS_Y_OFFSET_MM;
    float theta_x_offset_mm = GANTRY_THETA_X_OFFSET_MM;
    float gripper_x_offset_mm = GANTRY_GRIPPER_X_OFFSET_MM;
    float gripper_z_offset_mm = GANTRY_GRIPPER_Z_OFFSET_MM;
};

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

/// True when a periodic axislog tick is due. hz=0 never logs. First call
/// (last_us==0) always logs so motion start is visible.
inline bool axisLogDue(uint32_t hz, int64_t now_us, int64_t* last_us) {
    if (hz == 0 || last_us == nullptr) {
        return false;
    }
    const int64_t period_us = 1000000LL / static_cast<int64_t>(hz);
    if (period_us <= 0) {
        *last_us = now_us;
        return true;
    }
    if (*last_us == 0 || (now_us - *last_us) >= period_us) {
        *last_us = now_us;
        return true;
    }
    return false;
}

} // namespace Gantry

#endif // GANTRY_UTILS_H
