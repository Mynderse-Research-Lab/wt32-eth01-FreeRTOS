/**
 * @file Gantry.cpp
 * @brief Gantry class: construction, initialization, enable, configuration and alarm monitoring.
 * @version 2.0.1
 *
 * The Gantry class is implemented across the Gantry*.cpp files listed
 * in lib/Gantry/CMakeLists.txt; GantryInternal.h holds their shared helpers.
 */

#include "Gantry.h"
#include "GantryInternal.h"

#include "GantryThetaOrigin.h"

#include <cmath>
#include <cstring>
#include <mutex>
#include "esp_log.h"

using namespace Gantry::Constants;

namespace Gantry {
static const char* TAG = "Gantry";

#ifndef GANTRY_DIAG_SKIP_AXIS_X_INIT
#define GANTRY_DIAG_SKIP_AXIS_X_INIT 0
#endif
#ifndef GANTRY_DIAG_SKIP_AXIS_Z_INIT
#define GANTRY_DIAG_SKIP_AXIS_Z_INIT 0
#endif
#ifndef GANTRY_DIAG_SKIP_THETA_INIT
#define GANTRY_DIAG_SKIP_THETA_INIT 0
#endif

// ============================================================================
// PulseMotor constructor, factory methods, and preparePinsForBoot - REMOVED
// All drive control is EtherNet/IP over W5500. The DI constructor is the
// only remaining path. See Gantry.h for the updated constructor signature.
// ============================================================================

// Dependency-injection constructor - axisX_, axisZ_, axisTheta_ supplied.
Gantry::Gantry(std::unique_ptr<GantryLinearAxis> xAxis,
               std::unique_ptr<GantryLinearAxis> zAxis,
               std::unique_ptr<GantryRotaryAxis> thetaAxis,
               int gripperPin)
  : axisX_(std::move(xAxis)),
    axisZ_(std::move(zAxis)),
    axisTheta_(std::move(thetaAxis)),
    gripperPin_(gripperPin),
    xMinPin_(-1),
    xMaxPin_(-1),
    initialized_(false),
    enabled_(false),
    abortRequested_(false),
    homingInProgress_(false),
    calibrationInProgress_(false),
    workspaceCalibrated_(false),
    gripperActive_(false),
    currentX_mm_(0.0f),
    currentZ_(0),
    currentTheta_(0),
    targetZ_(0),
    targetTheta_(0),
    axisLength_(0),
    zAxisLength_(0),
    config_(),
    kinematicParams_(),
    stepsPerRev_(DEFAULT_STEPS_PER_REV),
    motionState_(MotionState::IDLE),
    targetX_mm_(0.0f),
    targetZ_mm_(0.0f),
    targetTheta_deg_(0.0f),
    safeZMarginFromMaxMm_(DEFAULT_SAFE_Z_HEIGHT_MM),
    speed_mm_per_s_(DEFAULT_SPEED_MM_PER_S),
    speed_deg_per_s_(DEFAULT_SPEED_DEG_PER_S),
    acceleration_mm_per_s2_(0),
    deceleration_mm_per_s2_(0),
    acceleration_deg_per_s2_(0),
    deceleration_deg_per_s2_(0),
    gripperTargetState_(false),
    gripperActuateStart_ms_(0),
    gripperActuateDurationMs_(GANTRY_GRIPPER_OPEN_TIME_MS),
    lastXPositionCounts_(0),
    xPulsesPerMmOverride_(0.0f),
    jointDirectMove_(false),
    eipLimitPhase_(EipLimitPhase::kIdle),
    eipLimitAxis_(EipLimitAxisRole::kNone),
    eipLimitPhaseStartMs_(0),
    eipLimitSawBusy_(false),
    bringUpPhase_(BringUpPhase::kIdle),
    calXParkMm_(GANTRY_CAL_X_PARK_MM) {
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool Gantry::begin() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (initialized_) {
        return true;
    }
    ESP_LOGI(TAG, "[BEGIN] Enter Gantry::begin()");

    if (gripperPin_ >= 0) {
        ESP_LOGI(TAG, "[BEGIN] Configure end-effector pin=%d", gripperPin_);
        endEffector_.configurePin(gripperPin_, true);
        endEffector_.begin();
        gripperActive_ = false;
        ESP_LOGI(TAG, "[BEGIN] End-effector initialized");
    }

    xMinSwitch_.begin();
    xMaxSwitch_.begin();
    zMinSwitch_.begin();
    zMaxSwitch_.begin();
    ESP_LOGI(TAG, "[BEGIN] Limit switch objects initialized");

    if (GANTRY_DIAG_SKIP_AXIS_X_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping X-axis initialize (diagnostic toggle)");
    } else if (axisX_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing X-axis driver");
        if (!axisX_->begin()) {
            ESP_LOGE(TAG, "[BEGIN] X-axis initialize failed");
            return false;
        }
        ESP_LOGI(TAG, "[BEGIN] X-axis initialize OK");
    } else {
        ESP_LOGE(TAG, "[BEGIN] X-axis is null (misconfigured DrivetrainType)");
        return false;
    }

    if (GANTRY_DIAG_SKIP_AXIS_Z_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping Z-axis initialize (diagnostic toggle)");
    } else if (axisZ_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing Z-axis driver");
        if (!axisZ_->begin()) {
            ESP_LOGE(TAG, "[BEGIN] Z-axis initialize failed");
            return false;
        }
        ESP_LOGI(TAG, "[BEGIN] Z-axis initialize OK");
    } else {
        ESP_LOGW(TAG, "[BEGIN] Z-axis is null (misconfigured DrivetrainType)");
    }

    if (GANTRY_DIAG_SKIP_THETA_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping theta initialize (diagnostic toggle)");
    } else if (axisTheta_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing theta driver");
        if (!axisTheta_->begin()) {
            ESP_LOGW(TAG, "[BEGIN] Theta initialize failed (proceeding without rotary axis)");
        } else {
            ESP_LOGI(TAG, "[BEGIN] Theta initialize OK");
        }
    } else {
        ESP_LOGW(TAG, "[BEGIN] Theta-axis is null (misconfigured DrivetrainType)");
    }

    // Wire per-axis MOVE log tags so the axis classes can emit START/END/MOVE
    // lines tagged with the human-readable axis name. Default periodic rate
    // stays at 0 (off) until the console (or other consumer) calls
    // setAxisLogRateHz(). String literals have static storage duration so the
    // pointers remain valid for the program lifetime.
    if (axisX_)     axisX_->setLogTag("X");
    if (axisZ_)     axisZ_->setLogTag("Z");
    if (axisTheta_) axisTheta_->setLogTag("Theta");

    initialized_ = true;
    ESP_LOGI(TAG, "[BEGIN] Gantry::begin() complete");
    return true;
}

// ============================================================================
// ENABLE/DISABLE
// ============================================================================

void Gantry::enable() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED();
    abortRequested_ = false;
    // Always re-issue axis enable so ServoOn gets a fresh 0->1 edge. Skipping
    // when isEnabled() was already true left Active=0 after boot enable
    // (Class 1 was not up yet when ServoOn was first asserted).
    bool xOk = true;
    bool zOk = true;
    bool tOk = true;
    if (axisX_) {
        xOk = axisX_->enable();
        if (!xOk) {
            ESP_LOGE(TAG, "X enable failed — disabling Z and Theta");
            if (axisZ_) axisZ_->disable();
            if (axisTheta_) axisTheta_->disable();
            enabled_ = false;
            return;
        }
    }
    if (axisZ_) zOk = axisZ_->enable();
    if (axisTheta_) tOk = axisTheta_->enable();
    enabled_ = zOk && tOk;
}

void Gantry::disable() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED();
    if (isBusy()) {
        stopAllMotion();
    }
    if (axisX_)     axisX_->disable();
    if (axisZ_)     axisZ_->disable();
    if (axisTheta_) axisTheta_->disable();
    enabled_ = false;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void Gantry::setLimitPins(int xMinPin, int xMaxPin) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    xMinPin_ = xMinPin;
    xMaxPin_ = xMaxPin;
    xMinSwitch_.configure(xMinPin_, true, true, 6);
    xMaxSwitch_.configure(xMaxPin_, true, true, 6);
}

void Gantry::configureDriveManagedLimits() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    xMinPin_ = -1;
    xMaxPin_ = -1;
    xMinSwitch_.configureExternal();
    xMaxSwitch_.configureExternal();
    zMinSwitch_.configureExternal();
    zMaxSwitch_.configureExternal();
    if (axisX_) {
        axisX_->attachLimitSwitches(&xMinSwitch_, &xMaxSwitch_);
    }
    if (axisZ_) {
        // Bench: joint −Z trips A015/NL; joint +Z trips A014/PL.
        axisZ_->setJointMinWarningA015(true);
        axisZ_->attachLimitSwitches(&zMinSwitch_, &zMaxSwitch_);
    }
    ESP_LOGI(TAG,
             "Drive-managed endstops enabled (EIP; X: A014=min/A015=max; "
             "Z: A015=min/A014=max)");
}

bool Gantry::driveManagedLimitsEnabled() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return xMinSwitch_.isConfigured() &&
           xMinSwitch_.source() == GantryLimitSwitch::Source::kDriveManaged;
}

void Gantry::setZAxisLimits(float minMm, float maxMm) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    config_.limits.z_min = minMm;
    config_.limits.z_max = maxMm;
}

void Gantry::setThetaLimits(float minDeg, float maxDeg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (axisTheta_) {
        axisTheta_->setAngleRange(minDeg, maxDeg);
        config_.limits.theta_min = axisTheta_->getMinDeg();
        config_.limits.theta_max = axisTheta_->getMaxDeg();
        return;
    }
    config_.limits.theta_min = minDeg;
    config_.limits.theta_max = maxDeg;
}

void Gantry::setJointLimits(float xMin, float xMax,
                            float zMin, float zMax,
                            float thetaMin, float thetaMax) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    config_.limits.x_min     = xMin;
    config_.limits.x_max     = xMax;
    config_.limits.z_min     = zMin;
    config_.limits.z_max     = zMax;
    if (xMax > 0.0f) {
        kinematicParams_.x_stroke_mm = xMax;
    }
    if (zMax > 0.0f) {
        kinematicParams_.z_stroke_mm = zMax;
    }
    if (axisTheta_) {
        axisTheta_->setAngleRange(thetaMin, thetaMax);
        config_.limits.theta_min = axisTheta_->getMinDeg();
        config_.limits.theta_max = axisTheta_->getMaxDeg();
    } else {
        config_.limits.theta_min = thetaMin;
        config_.limits.theta_max = thetaMax;
    }
}

void Gantry::setEndEffectorPin(int pin, bool activeHigh) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    gripperPin_ = pin;
    endEffector_.configurePin(pin, activeHigh);
    if (initialized_) {
        endEffector_.begin();
    }
}

void Gantry::setSafeZHeight(float safeHeight_mm) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Argument is the margin above Z− / A015 (not an absolute joint plane).
    if (safeHeight_mm <= 0.0f) {
        return;
    }
    safeZMarginFromMaxMm_ = safeHeight_mm;
}

void Gantry::setRuntimeParams(const RuntimeParams& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    runtime_ = params;
    kinematicParams_.z_axis_y_offset_mm = params.z_axis_y_offset_mm;
    kinematicParams_.theta_x_offset_mm = params.theta_x_offset_mm;
    kinematicParams_.gripper_x_offset_mm = params.gripper_x_offset_mm;
    kinematicParams_.gripper_z_offset_mm = params.gripper_z_offset_mm;
}

float Gantry::zSpeedCapMmS() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return runtime_.z_speed_cap_mm_s;
}

float Gantry::zAccelCapMmS2() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return runtime_.z_accel_cap_mm_s2;
}

float Gantry::zDecelCapMmS2() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return runtime_.z_decel_cap_mm_s2;
}

float Gantry::traverseClearanceZMm() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Bottom-band ceiling: coordinated X+Z only while Z <= z_min + margin.
    return Path::bandCeilingFromZMinus(config_.limits.z_min, safeZMarginFromMaxMm_);
}

bool Gantry::zInTraverseBand() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const float z = axisZ_ ? axisZ_->getCurrentMm() : static_cast<float>(currentZ_);
    return Path::zInTraverseBand(z, traverseClearanceZMm());
}

bool Gantry::requireXTraverseInterlock(const char* what) {
    if (zInTraverseBand()) {
        return true;
    }
    const float z = axisZ_ ? axisZ_->getCurrentMm() : static_cast<float>(currentZ_);
    const float band = traverseClearanceZMm();
    ESP_LOGE(TAG,
             "[INTERLOCK] X blocked (%s): Z=%.3f above band ceiling %.3f "
             "(SAFE_Z=%.1f mm from Z-/A015; lower Z first or use 'calibrate all')",
             what ? what : "X move", z, band, safeZMarginFromMaxMm_);
    return false;
}

bool Gantry::requireThetaTraverseInterlock(const char* what) {
    if (zInTraverseBand()) {
        return true;
    }
    const float z = axisZ_ ? axisZ_->getCurrentMm() : static_cast<float>(currentZ_);
    const float band = traverseClearanceZMm();
    ESP_LOGE(TAG,
             "[INTERLOCK] theta blocked (%s): Z=%.3f above band ceiling %.3f "
             "(SAFE_Z=%.1f mm from Z-/A015; theta only in the retract/traverse band)",
             what ? what : "theta move", z, band, safeZMarginFromMaxMm_);
    return false;
}

// ============================================================================
// ALARM MONITORING
// ============================================================================

bool Gantry::isAlarmActive() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;
    const bool xAlarm = axisX_     && axisX_->isAlarmActive();
    const bool zAlarm = axisZ_     && axisZ_->isAlarmActive();
    const bool tAlarm = axisTheta_ && axisTheta_->isAlarmActive();
    return xAlarm || zAlarm || tAlarm;
}

bool Gantry::drivesOnline() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;
    const bool xOk = !axisX_     || axisX_->isLinkOnline();
    const bool zOk = !axisZ_     || axisZ_->isLinkOnline();
    const bool tOk = !axisTheta_ || axisTheta_->isLinkOnline();
    return xOk && zOk && tOk;
}

const char* Gantry::inactiveServoAxis() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (axisX_ && !axisX_->isServoActive()) return "X";
    if (axisZ_ && !axisZ_->isServoActive()) return "Z";
    if (axisTheta_ && !axisTheta_->isServoActive()) return "theta";
    return nullptr;
}

bool Gantry::clearAlarm() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;
    // Stop sequential motion before FaultReset so we do not switch the
    // mid-move command image out from under an axis (A603 ping-pong).
    if (motionState_ != MotionState::IDLE) {
        stopAllMotion();
    }
    bool ok = false;
    if (axisX_)     ok = axisX_->clearAlarm()     || ok;
    if (axisZ_)     ok = axisZ_->clearAlarm()     || ok;
    if (axisTheta_) ok = axisTheta_->clearAlarm() || ok;
    return ok;
}

void Gantry::logDriveAlarmSummaries() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    char buf[192];
    if (axisX_ && axisX_->getDriveAlarmSummary(buf, sizeof(buf))) {
        if (buf[0] && std::strcmp(buf, "clear") != 0) {
            ESP_LOGW(TAG, "X drive: %s", buf);
        }
    }
    if (axisZ_ && axisZ_->getDriveAlarmSummary(buf, sizeof(buf))) {
        if (buf[0] && std::strcmp(buf, "clear") != 0) {
            ESP_LOGW(TAG, "Z drive: %s", buf);
        }
    }
    if (axisTheta_ && axisTheta_->getDriveAlarmSummary(buf, sizeof(buf))) {
        if (buf[0] && std::strcmp(buf, "clear") != 0) {
            ESP_LOGW(TAG, "Theta drive: %s", buf);
        }
    }
}

bool Gantry::getXDriveAlarmSummary(char* buf, size_t n) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!axisX_) {
        if (buf && n) buf[0] = '\0';
        return false;
    }
    return axisX_->getDriveAlarmSummary(buf, n);
}

bool Gantry::getZDriveAlarmSummary(char* buf, size_t n) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!axisZ_) {
        if (buf && n) buf[0] = '\0';
        return false;
    }
    return axisZ_->getDriveAlarmSummary(buf, n);
}

void Gantry::setHomingSpeed(uint32_t speed_pps) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Homing is not supported in EIP-only mode. The speed is set via
    // the EIP assembly HomeReturnSpeed field (OutputAssembly104 byte 20).
    (void)speed_pps;
}

void Gantry::setAxisLogRateHz(uint32_t hz) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (axisX_)     axisX_->setLogRateHz(hz);
    if (axisZ_)     axisZ_->setLogRateHz(hz);
    if (axisTheta_) axisTheta_->setLogRateHz(hz);
}

} // namespace Gantry
