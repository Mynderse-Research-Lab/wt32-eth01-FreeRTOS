/**
 * @file Gantry.cpp
 * @brief Implementation of Gantry class.
 * @version 2.0.0
 */

#include "Gantry.h"
#include "GantryEipLinearAxis.h"
#include "KinetixFaultCodes.h"
#include <cmath>
#include <cstring>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
// Arduino-compat helpers, pure ESP-IDF implementation.
static inline unsigned long gantry_millis() {
    return (unsigned long)(esp_timer_get_time() / 1000LL);
}
static inline void gantry_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
}  // namespace

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
    gripperTargetState_(false),
    gripperActuateStart_ms_(0),
    gripperActuateDurationMs_(GRIPPER_ACTUATE_TIME_MS),
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
    GANTRY_CHECK_INITIALIZED();
    abortRequested_ = false;
    // Always re-issue axis enable so ServoOn gets a fresh 0->1 edge. Skipping
    // when isEnabled() was already true left Active=0 after boot enable
    // (Class 1 was not up yet when ServoOn was first asserted).
    bool xOk = true;
    bool zOk = true;
    bool tOk = true;
    if (axisX_) xOk = axisX_->enable();
    if (axisZ_) zOk = axisZ_->enable();
    if (axisTheta_) tOk = axisTheta_->enable();
    enabled_ = xOk && zOk && tOk;
}

void Gantry::disable() {
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
    xMinPin_ = xMinPin;
    xMaxPin_ = xMaxPin;
    xMinSwitch_.configure(xMinPin_, true, true, 6);
    xMaxSwitch_.configure(xMaxPin_, true, true, 6);
}

void Gantry::configureDriveManagedLimits() {
    xMinPin_ = -1;
    xMaxPin_ = -1;
    xMinSwitch_.configureExternal();
    xMaxSwitch_.configureExternal();
    zMinSwitch_.configureExternal();
    zMaxSwitch_.configureExternal();
    auto* xEip = dynamic_cast<GantryEipLinearAxis*>(axisX_.get());
    if (xEip != nullptr) {
        xEip->attachLimitSwitches(&xMinSwitch_, &xMaxSwitch_);
    }
    auto* zEip = dynamic_cast<GantryEipLinearAxis*>(axisZ_.get());
    if (zEip != nullptr) {
        // Bench: joint −Z trips A015/NL; joint +Z trips A014/PL.
        zEip->setJointMinWarningA015(true);
        zEip->attachLimitSwitches(&zMinSwitch_, &zMaxSwitch_);
    }
    ESP_LOGI(TAG,
             "Drive-managed endstops enabled (EIP; X: A014=min/A015=max; "
             "Z: A015=min/A014=max)");
}

bool Gantry::driveManagedLimitsEnabled() const {
    return xMinSwitch_.isConfigured() &&
           xMinSwitch_.source() == GantryLimitSwitch::Source::kDriveManaged;
}

void Gantry::setZAxisLimits(float minMm, float maxMm) {
    config_.limits.z_min = minMm;
    config_.limits.z_max = maxMm;
}

void Gantry::setThetaLimits(float minDeg, float maxDeg) {
    config_.limits.theta_min = minDeg;
    config_.limits.theta_max = maxDeg;
    if (axisTheta_) {
        axisTheta_->setAngleRange(minDeg, maxDeg);
    }
}

void Gantry::setJointLimits(float xMin, float xMax,
                            float zMin, float zMax,
                            float thetaMin, float thetaMax) {
    config_.limits.x_min     = xMin;
    config_.limits.x_max     = xMax;
    config_.limits.z_min     = zMin;
    config_.limits.z_max     = zMax;
    config_.limits.theta_min = thetaMin;
    config_.limits.theta_max = thetaMax;
    if (axisTheta_) {
        axisTheta_->setAngleRange(thetaMin, thetaMax);
    }
}

void Gantry::setEndEffectorPin(int pin, bool activeHigh) {
    gripperPin_ = pin;
    endEffector_.configurePin(pin, activeHigh);
    if (initialized_) {
        endEffector_.begin();
    }
}

void Gantry::setSafeZHeight(float safeHeight_mm) {
    // Argument is the margin above Z− / A015 (not an absolute joint plane).
    if (safeHeight_mm <= 0.0f) {
        return;
    }
    safeZMarginFromMaxMm_ = safeHeight_mm;
}

float Gantry::traverseClearanceZMm() const {
    // Bottom-band ceiling: coordinated X+Z only while Z <= z_min + margin.
    return Path::bandCeilingFromZMinus(config_.limits.z_min, safeZMarginFromMaxMm_);
}

bool Gantry::zInTraverseBand() const {
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

// ============================================================================
// MOTION CONTROL
// ============================================================================

void Gantry::moveTo(int32_t x, int32_t z, int32_t theta, uint32_t speed) {
    if (!initialized_ || !enabled_) {
        return;
    }
    if (speed == 0) {
        speed = 5000;
    }

    targetZ_ = z;
    targetTheta_ = theta;

    targetX_mm_       = (float)x;
    targetZ_mm_       = (float)z;
    targetTheta_deg_  = (float)theta;
    // Convert X pulses/s to mm/s using X axis scaling.
    const float xPpm = getPulsesPerMm();
    speed_mm_per_s_        = (uint32_t)((float)speed / xPpm);
    acceleration_mm_per_s2_ = 0;
    deceleration_mm_per_s2_ = 0;
    jointDirectMove_ = true;

    startSequentialMotion();
}

GantryError Gantry::moveTo(const JointConfig& joint,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2) {
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    GANTRY_CHECK_BUSY_RET(GantryError::ALREADY_MOVING);

    if (axisX_ && axisX_->isAlarmActive()) {
        return GantryError::TIMEOUT;
    }

    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }

    targetX_mm_       = joint.x;
    targetZ_mm_       = joint.z;
    targetTheta_deg_  = joint.theta;
    speed_mm_per_s_   = speed_mm_per_s;
    speed_deg_per_s_  = speed_deg_per_s;
    acceleration_mm_per_s2_ = acceleration_mm_per_s2;
    deceleration_mm_per_s2_ = deceleration_mm_per_s2;
    jointDirectMove_ = true;

    startSequentialMotion();
    return GantryError::OK;
}

GantryError Gantry::moveTo(const EndEffectorPose& pose,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2) {
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    GANTRY_CHECK_BUSY_RET(GantryError::ALREADY_MOVING);

    JointConfig joint = inverseKinematics(pose);
    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }

    targetX_mm_       = joint.x;
    targetZ_mm_       = joint.z;
    targetTheta_deg_  = joint.theta;
    speed_mm_per_s_   = speed_mm_per_s;
    speed_deg_per_s_  = speed_deg_per_s;
    acceleration_mm_per_s2_ = acceleration_mm_per_s2;
    deceleration_mm_per_s2_ = deceleration_mm_per_s2;
    jointDirectMove_ = false;

    startSequentialMotion();
    return GantryError::OK;
}

bool Gantry::isBusy() const {
    if (!initialized_) {
        return false;
    }
    const bool xBusy = axisX_ && (axisX_->isMotionActive());
    const bool zBusy = axisZ_ && axisZ_->isBusy();
    const bool tBusy = axisTheta_ && axisTheta_->isMotionActive();
    return motionState_ != MotionState::IDLE || xBusy || zBusy || tBusy ||
           homingInProgress_ || calibrationInProgress_;
}

bool Gantry::isEnabled() const { return initialized_ && enabled_; }

void Gantry::requestAbort() {
    abortRequested_ = true;
    if (initialized_) {
        stopAllMotion();
    }
}

bool Gantry::isAbortRequested() const { return abortRequested_; }

void Gantry::update() {
    GANTRY_CHECK_INITIALIZED();

    // Advance EIP axis state machines first so isBusy() reflects the latest
    // StartMotion preload/pulse phase before sequential sequencing decides.
    updateAxisPositions();

    if (motionState_ != MotionState::IDLE) {
        processSequentialMotion();
    }
}

// ============================================================================
// HOMING AND CALIBRATION
// ============================================================================

void Gantry::home() {
    // Legacy entry point: only X is currently real, so behavior is unchanged.
    // Z and Theta stubs are NOT invoked here so existing callers that depend
    // on `home()` blocking only on the X sweep keep working.
    homeX();
}

void Gantry::homeX() {
    GANTRY_CHECK_INITIALIZED();
    GANTRY_CHECK_ENABLED();
    abortRequested_ = false;
    homingInProgress_ = false;

    if (!axisX_) return;
    if (axisX_->isAlarmActive()) return;

    if (!requireXTraverseInterlock("home x")) {
        return;
    }

    if (driveManagedLimitsEnabled()) {
        startEipPrecisionHome(EipLimitAxisRole::kX);
        return;
    }

    if (!xMinSwitch_.isConfigured() || !xMaxSwitch_.isConfigured()) {
        return;
    }

    xMinSwitch_.update(true);
    xMaxSwitch_.update(true);

    if (xMinSwitch_.isActive()) {
        axisX_->stopMotion();
        axisX_->setCurrentPulses(0);
        return;
    }

    constexpr uint32_t kHomingStartPosition = 100000000;
    const uint32_t     speed                = getHomingSpeed();
    axisX_->setCurrentPulses(kHomingStartPosition);
    // Seed the position tracker to match the synthetic bump. Otherwise the
    // very next updateAxisPositions() tick sees currentXCounts (~100M) >
    // lastXPositionCounts_ (~0), mis-classifies it as "moving toward MAX",
    // and if MAX is active it instantly stop-motions the home before any
    // real pulses are emitted. (Confirmed by runtime evidence: setDirection
    // fires with logical=0, then "Homing did not start" appears 20ms later
    // with gate state max=1.)
    lastXPositionCounts_ = kHomingStartPosition;
    ESP_LOGI(TAG,
             "[HOME] seeded position tracker to %lu before negative-direction sweep",
             (unsigned long)kHomingStartPosition);
    if (!axisX_->moveToPulses(0, speed, speed, speed)) {
        stopAllMotion();
        return;
    }
    homingInProgress_ = true;
}

void Gantry::homeZ() {
    GANTRY_CHECK_INITIALIZED();
    GANTRY_CHECK_ENABLED();
    abortRequested_ = false;
    homingInProgress_ = false;

    if (!axisZ_) return;
    if (axisZ_->isAlarmActive()) return;

    if (driveManagedLimitsEnabled()) {
        startEipPrecisionHome(EipLimitAxisRole::kZ);
        return;
    }

    ESP_LOGW(TAG,
             "[HOME] Z axis home requires drive-managed EIP limits "
             "(GPIO Z switches not integrated)");
}

void Gantry::homeTheta() {
    // Theta currently has no limit switches; pins that used to host theta
    // limits were reassigned to DIR/ENABLE. Log a placeholder until the
    // switches are added to the hardware.
    ESP_LOGW(TAG, "[HOME] Theta axis home not yet wired (no theta limit switches)");
}

void Gantry::softHomeJointDatum() {
    GANTRY_CHECK_INITIALIZED();
    if (axisX_) {
        axisX_->setCurrentPulses(0);
        currentX_mm_ = axisX_->getCurrentMm();
    }
    if (axisZ_) {
        axisZ_->setCurrentPulses(0);
        currentZ_ = (int32_t)axisZ_->getCurrentMm();
    }
}

int Gantry::calibrate() {
    return calibrateX();
}

int Gantry::calibrateZ() {
    GANTRY_CHECK_INITIALIZED_RET(0);
    GANTRY_CHECK_ENABLED_RET(0);
    abortRequested_ = false;

    if (!axisZ_) return 0;
    if (axisZ_->isAlarmActive()) return 0;

    if (!driveManagedLimitsEnabled()) {
        ESP_LOGW(TAG,
                 "[CAL] Z axis calibrate requires drive-managed EIP limits "
                 "(GPIO Z switches not integrated)");
        return 0;
    }

    // Keep calibrationInProgress_ set through home→cal handoff in kHomeSettle.
    // On EIP-LIMIT failure the SM clears the flag; do NOT restart cal from Idle
    // after a failed home (that left active=0 / rejected Absolute).
    calibrationInProgress_ = true;
    zAxisLength_ = 0;
    if (eipLimitPhase_ == EipLimitPhase::kIdle) {
        startEipPrecisionHome(EipLimitAxisRole::kZ);
    }
    unsigned long start_ms = gantry_millis();
    while (calibrationInProgress_ &&
           (gantry_millis() - start_ms) < TRAVEL_MEASUREMENT_TIMEOUT_MS) {
        if (abortRequested_) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
        gantry_delay(10);
    }
    if (calibrationInProgress_) {
        ESP_LOGE(TAG, "[CAL] EIP Z calibrate timed out");
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }
    return zAxisLength_;
}

int Gantry::calibrateTheta() {
    ESP_LOGW(TAG, "[CAL] Theta axis calibrate not yet wired (no theta limit switches)");
    return 0;
}

int Gantry::calibrateX() {
    GANTRY_CHECK_INITIALIZED_RET(0);
    GANTRY_CHECK_ENABLED_RET(0);
    abortRequested_ = false;

    if (!axisX_) return 0;
    if (axisX_->isAlarmActive()) return 0;

    if (!requireXTraverseInterlock("calibrate x")) {
        return 0;
    }

    if (driveManagedLimitsEnabled()) {
        // Keep calibrationInProgress_ set through home→cal handoff in kHomeSettle.
        // On EIP-LIMIT failure the SM clears the flag; do NOT restart cal from Idle
        // after a failed home.
        calibrationInProgress_ = true;
        axisLength_ = 0;
        if (eipLimitPhase_ == EipLimitPhase::kIdle) {
            startEipPrecisionHome(EipLimitAxisRole::kX);
            if (eipLimitPhase_ == EipLimitPhase::kIdle) {
                // Interlock or missing axis aborted before arming.
                calibrationInProgress_ = false;
                return 0;
            }
        }
        unsigned long start_ms = gantry_millis();
        while (calibrationInProgress_ &&
               (gantry_millis() - start_ms) < TRAVEL_MEASUREMENT_TIMEOUT_MS) {
            if (abortRequested_) {
                stopAllMotion();
                calibrationInProgress_ = false;
                return 0;
            }
            gantry_delay(10);
        }
        if (calibrationInProgress_) {
            ESP_LOGE(TAG, "[CAL] EIP calibrate timed out");
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
        return axisLength_;
    }

    calibrationInProgress_ = true;

    if (!xMinSwitch_.isConfigured() || !xMaxSwitch_.isConfigured()) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    homeX();

    unsigned long start_ms = gantry_millis();
    while (homingInProgress_ && (gantry_millis() - start_ms) < CALIBRATION_TIMEOUT_MS) {
        if (abortRequested_) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
        xMinSwitch_.update();
        xMaxSwitch_.update();
        gantry_delay(10);
    }

    if (homingInProgress_ || !xMinSwitch_.isActive()) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    const uint32_t speed = getHomingSpeed();
    constexpr uint32_t kCalibrationTarget = 1000000000UL;
    constexpr uint32_t kMinReleaseTimeoutMs = 3000;
    if (!axisX_->moveToPulses(kCalibrationTarget, speed, speed, speed)) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    start_ms = gantry_millis();
    bool minReleased = false;
    while (axisX_->isMotionActive() &&
           (gantry_millis() - start_ms) < TRAVEL_MEASUREMENT_TIMEOUT_MS) {
        if (abortRequested_) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
        xMinSwitch_.update();
        xMaxSwitch_.update();

        if (!minReleased) {
            if (!xMinSwitch_.isActive()) {
                minReleased = true;
            } else if ((gantry_millis() - start_ms) > kMinReleaseTimeoutMs) {
                ESP_LOGW(TAG,
                         "Calibration abort: MIN limit did not release within %lu ms",
                         (unsigned long)kMinReleaseTimeoutMs);
                stopAllMotion();
                calibrationInProgress_ = false;
                return 0;
            }
        }

        gantry_delay(10);
        if (axisX_->isAlarmActive()) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
    }

    // The inner loop exited because axisX_->isMotionActive() became false.
    // The only place that stops motion mid-sweep is updateAxisPositions()'s
    // movingTowardMax + xMaxSwitch_.isActive() guard, which is also what
    // sets calibrationInProgress_=false. So a non-active stableState here
    // genuinely means we exited for a different reason (timeout, abort,
    // alarm) - those branches have already returned. Reaching this point
    // with stableState=true is the success signal.
    //
    // DO NOT call xMaxSwitch_.update(true) here. Force-overwriting the
    // debounced stableState with the raw level at this instant races
    // against the carriage's residual deceleration overshooting the
    // inductive sensor's detection zone. The debounce that already
    // committed during the inner loop is the correct truth - the carriage
    // really did pass over the MAX sensor for >=60ms of stable sampling.
    // (Confirmed by runtime evidence: every "Calibration failed" event is
    // preceded by max_limit transitioning 0->1 ~25ms earlier.)
    if (!xMaxSwitch_.isActive()) {
        ESP_LOGW(TAG,
                 "[CAL] inner loop exited but xMaxSwitch_ not active "
                 "(unexpected; aborting)");
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    // Use the commanded driver position (LEDC pulse count), NOT
    // axisX_->getCurrentMm(). The latter routes through the encoder
    // accumulator when enable_encoder_feedback=true, and the X-axis
    // encoder is currently not returning pulses (see
    // docs/LOW_LEVEL_GANTRY_CONTROL.md - wiring TODO). We measured the
    // physical travel by commanding pulses from MIN to MAX, so the
    // pulse count between those two events is the authoritative travel
    // distance regardless of encoder availability.
    const double commandedPulses = (double)axisX_->getCurrentPulses();
    const double basePpm         = axisX_->pulsesPerMm();
    if (basePpm <= 0.0) {
        ESP_LOGE(TAG, "[CAL] pulses-per-mm not configured; cannot compute axis length");
        calibrationInProgress_ = false;
        return 0;
    }

    // Re-anchor X scaling to the known physical hard-travel envelope.
    // This keeps dead-reckoning travel accurate even if commanded pulses/rev
    // in the drive do not match the compile-time drivetrain constants yet.
    const double hardSpanMm = (double)AXIS_X_HARD_LIMIT_MAX_MM - (double)AXIS_X_HARD_LIMIT_MIN_MM;
    const double derivedLengthMm = commandedPulses / basePpm;
    if (hardSpanMm > 1.0 && commandedPulses > 0.0) {
        xPulsesPerMmOverride_ = (float)(commandedPulses / hardSpanMm);
        axisLength_ = (int32_t)(hardSpanMm + 0.5);
        ESP_LOGI(TAG,
                 "[CAL] travel measured %.1f mm from base ppm=%.4f; applying corrected ppm=%.4f from hard span=%.1f mm",
                 derivedLengthMm, basePpm, (double)xPulsesPerMmOverride_, hardSpanMm);
    } else {
        xPulsesPerMmOverride_ = 0.0f;
        axisLength_ = (int32_t)(derivedLengthMm + 0.5);
        ESP_LOGW(TAG,
                 "[CAL] hard span unavailable; using base ppm only (axisLength=%ld mm, ppm=%.4f)",
                 (long)axisLength_, basePpm);
    }
    calibrationInProgress_ = false;
    return axisLength_;
}

// ============================================================================
// GRIPPER
// ============================================================================

void Gantry::grip(bool active) {
    GANTRY_CHECK_INITIALIZED();
    if (endEffector_.isConfigured()) {
        endEffector_.setActive(active);
        gripperActive_ = active;
    }
}

// ============================================================================
// ACCESSORS
// ============================================================================

int Gantry::getXEncoder() const {
    if (!initialized_ || !axisX_) return 0;
    if (axisX_->isEncoderFeedbackEnabled()) {
        return axisX_->getEncoderPulses();
    }
    return (int32_t)axisX_->getCurrentPulses();
}

int Gantry::getXEncoderRaw() const {
    if (!initialized_ || !axisX_) return 0;
    return axisX_->getEncoderPulses();
}

int32_t Gantry::getXCommandedPulses() const {
    if (!initialized_ || !axisX_) return 0;
    if (homingInProgress_ || calibrationInProgress_) {
        return 0;
    }
    return (int32_t)axisX_->getCurrentPulses();
}

float Gantry::getXCommandedMm() const {
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)((double)getXCommandedPulses() / (double)ppm);
}

float Gantry::getXEncoderMm() const {
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)((double)getXEncoderRaw() / (double)ppm);
}

int Gantry::getCurrentZ() const {
    return (int)(axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_);
}

float Gantry::getZCommandedMm() const {
    return axisZ_ ? axisZ_->getTargetMm() : targetZ_mm_;
}

float Gantry::getZEncoderMm() const {
    return axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_;
}

int32_t Gantry::getZEncoderPulses() const {
    if (!initialized_ || !axisZ_) return 0;
    return axisZ_->getEncoderPulses();
}

float Gantry::getZPulsesPerMm() const {
    if (axisZ_ && axisZ_->pulsesPerMm() > 0.0) {
        return static_cast<float>(axisZ_->pulsesPerMm());
    }
    return 0.0f;
}

int Gantry::getCurrentTheta() const {
    return axisTheta_ ? (int)axisTheta_->getCurrentDeg() : currentTheta_;
}

// ============================================================================
// ALARM MONITORING
// ============================================================================

bool Gantry::isAlarmActive() const {
    if (!initialized_) return false;
    const bool xAlarm = axisX_     && axisX_->isAlarmActive();
    const bool zAlarm = axisZ_     && axisZ_->isAlarmActive();
    const bool tAlarm = axisTheta_ && axisTheta_->isAlarmActive();
    return xAlarm || zAlarm || tAlarm;
}

bool Gantry::clearAlarm() {
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
}

bool Gantry::getXDriveAlarmSummary(char* buf, size_t n) const {
    if (!axisX_) {
        if (buf && n) buf[0] = '\0';
        return false;
    }
    return axisX_->getDriveAlarmSummary(buf, n);
}

bool Gantry::getZDriveAlarmSummary(char* buf, size_t n) const {
    if (!axisZ_) {
        if (buf && n) buf[0] = '\0';
        return false;
    }
    return axisZ_->getDriveAlarmSummary(buf, n);
}

void Gantry::setHomingSpeed(uint32_t speed_pps) {
    // Homing is not supported in EIP-only mode. The speed is set via
    // the EIP assembly HomeReturnSpeed field (OutputAssembly104 byte 20).
    (void)speed_pps;
}

void Gantry::setAxisLogRateHz(uint32_t hz) {
    if (axisX_)     axisX_->setLogRateHz(hz);
    if (axisZ_)     axisZ_->setLogRateHz(hz);
    if (axisTheta_) axisTheta_->setLogRateHz(hz);
}

// ============================================================================
// KINEMATICS
// ============================================================================

EndEffectorPose Gantry::forwardKinematics(const JointConfig& joint) const {
    return Kinematics::forward(joint, kinematicParams_);
}

JointConfig Gantry::inverseKinematics(const EndEffectorPose& pose) const {
    return Kinematics::inverse(pose, kinematicParams_);
}

JointConfig Gantry::getCurrentJointConfig() const {
    JointConfig joint;
    joint.x     = currentX_mm_;
    joint.z     = axisZ_     ? axisZ_->getCurrentMm()     : (float)currentZ_;
    joint.theta = axisTheta_ ? axisTheta_->getCurrentDeg() : (float)currentTheta_;
    return joint;
}

JointConfig Gantry::getTargetJointConfig() const {
    JointConfig joint;
    joint.x     = targetX_mm_;
    joint.z     = axisZ_ ? axisZ_->getTargetMm() : targetZ_mm_;
    joint.theta = axisTheta_ ? axisTheta_->getTargetDeg() : (float)targetTheta_;
    return joint;
}

EndEffectorPose Gantry::getCurrentEndEffectorPose() const {
    return forwardKinematics(getCurrentJointConfig());
}

EndEffectorPose Gantry::getTargetEndEffectorPose() const {
    return forwardKinematics(getTargetJointConfig());
}

// ============================================================================
// HELPERS
// ============================================================================

void Gantry::setStepsPerRevolution(float steps_per_rev) {
    if (steps_per_rev > 0) {
        stepsPerRev_ = steps_per_rev;
    }
}

float Gantry::getPulsesPerMm() const {
    if (xPulsesPerMmOverride_ > 0.0f) {
        return xPulsesPerMmOverride_;
    }
    if (axisX_ && axisX_->pulsesPerMm() > 0.0) {
        return (float)axisX_->pulsesPerMm();
    }
    return DEFAULT_PULSES_PER_MM;
}

float Gantry::pulsesToMm(int32_t pulses) const {
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)pulses / ppm;
}

int32_t Gantry::mmToPulses(float mm) const {
    return (int32_t)(mm * getPulsesPerMm());
}

uint32_t Gantry::getHomingSpeed() const {
    if (axisX_) {
        const uint32_t s = axisX_->homingSpeedPps();
        if (s > 0) return s;
    }
    return DEFAULT_HOMING_SPEED_PPS;
}

bool Gantry::moveZAxisTo(float targetZ, float speed, float accel, float decel) {
    if (axisZ_) {
        return axisZ_->moveToMm(targetZ, speed, accel, decel);
    }
    currentZ_ = (int32_t)targetZ;
    return true;
}

void Gantry::updateAxisPositions() {
    if (!axisX_) {
        return;
    }

    // Refresh drive warnings -> external switches before limit decisions.
    axisX_->update();
    xMinSwitch_.update();
    xMaxSwitch_.update();
    if (axisZ_) {
        axisZ_->update();
        zMinSwitch_.update();
        zMaxSwitch_.update();
        currentZ_ = (int32_t)axisZ_->getCurrentMm();
    }

    if (bringUpPhase_ != BringUpPhase::kIdle) {
        advanceEipBringUp();
    } else if (eipLimitPhase_ != EipLimitPhase::kIdle) {
        advanceEipLimitSequence();
    } else if (axisX_->isMotionActive()) {
        uint32_t currentXCounts = axisX_->getCurrentPulses();
        bool movingTowardMax = currentXCounts > lastXPositionCounts_;
        bool movingTowardMin = currentXCounts < lastXPositionCounts_;

        if (movingTowardMax && xMaxSwitch_.isConfigured() && xMaxSwitch_.isActive()) {
            axisX_->stopMotion();
            homingInProgress_      = false;
            calibrationInProgress_ = false;
        } else if (movingTowardMin && xMinSwitch_.isConfigured() && xMinSwitch_.isActive()) {
            axisX_->stopMotion();
            axisX_->setCurrentPulses(0);
            homingInProgress_ = false;
        }
        lastXPositionCounts_ = currentXCounts;
    } else if (homingInProgress_) {
        if (xMinSwitch_.isActive()) {
            axisX_->setCurrentPulses(0);
        }
        homingInProgress_ = false;
        lastXPositionCounts_ = axisX_->getCurrentPulses();
    } else {
        lastXPositionCounts_ = axisX_->getCurrentPulses();
    }

    currentX_mm_ = axisX_->getCurrentMm();

    if (axisTheta_) {
        axisTheta_->update();
        currentTheta_ = (int32_t)axisTheta_->getCurrentDeg();
    }
}

void Gantry::stopAllMotion() {
    if (axisX_)     axisX_->stopMotion();
    if (axisZ_)     axisZ_->stopMotion();
    if (axisTheta_) axisTheta_->stopMotion();
    homingInProgress_      = false;
    calibrationInProgress_ = false;
    eipLimitPhase_         = EipLimitPhase::kIdle;
    eipLimitAxis_          = EipLimitAxisRole::kNone;
    bringUpPhase_          = BringUpPhase::kIdle;
    motionState_           = MotionState::IDLE;
    jointDirectMove_       = false;
    pathSegmentCount_      = 0;
    pathSegmentIndex_      = 0;
    pathDoGripperAfter_    = false;
}

GantryLinearAxis* Gantry::eipLimitActiveAxis() {
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return axisZ_.get();
    }
    return axisX_.get();
}

const GantryLinearAxis* Gantry::eipLimitActiveAxis() const {
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return axisZ_.get();
    }
    return axisX_.get();
}

bool Gantry::eipMinWarningActive() const {
    auto* eip = dynamic_cast<const GantryEipLinearAxis*>(eipLimitActiveAxis());
    if (eip == nullptr) return false;
    // X: A014 = joint min. Z: A015 = joint min (−Z).
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return eip->isA015WarningActive();
    }
    return eip->isA014WarningActive();
}

bool Gantry::eipMaxWarningActive() const {
    auto* eip = dynamic_cast<const GantryEipLinearAxis*>(eipLimitActiveAxis());
    if (eip == nullptr) return false;
    // X: A015 = joint max. Z: A014 = joint max (+Z).
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return eip->isA014WarningActive();
    }
    return eip->isA015WarningActive();
}

float Gantry::eipSeekSpeedMmS() const {
    return 10.0f;
}

float Gantry::eipCreepSpeedMmS() const {
    return 1.0f;
}

float Gantry::eipCalSeekSpeedMmS() const {
    if (speed_mm_per_s_ > 0) {
        return static_cast<float>(speed_mm_per_s_);
    }
    return eipSeekSpeedMmS();
}

bool Gantry::eipStartMoveDelta(float delta_mm, float speed_mm_s) {
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) return false;
    if (eipLimitAxis_ == EipLimitAxisRole::kX &&
        !requireXTraverseInterlock("EIP X Absolute")) {
        return false;
    }
    const float accel =
        (acceleration_mm_per_s2_ > 0)
            ? static_cast<float>(acceleration_mm_per_s2_)
            : 500.0f;
    const float decel =
        (deceleration_mm_per_s2_ > 0)
            ? static_cast<float>(deceleration_mm_per_s2_)
            : accel;
    const float here = axis->getCurrentMm();
    return axis->moveToMm(here + delta_mm, speed_mm_s, accel, decel);
}

void Gantry::startEipPrecisionHome(EipLimitAxisRole role) {
    if (role == EipLimitAxisRole::kX &&
        !requireXTraverseInterlock("EIP home X")) {
        homingInProgress_ = false;
        eipLimitPhase_ = EipLimitPhase::kIdle;
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    eipLimitAxis_ = role;
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    abortRequested_ = false;
    homingInProgress_ = true;
    eipLimitSawBusy_ = false;
    eipLimitPhaseStartMs_ = gantry_millis();
    const char* ax = EipLimit::axisLetter(role);
    const char* minCode = EipLimit::minWarningLabel(role);

    if (eipMinWarningActive()) {
        ESP_LOGI(TAG,
                 "[HOME] EIP already on -%s limit %s - creep toward +%s until clear",
                 ax, minCode, ax);
        eipLimitPhase_ = EipLimitPhase::kHomeCreepClear;
        return;
    }

    ESP_LOGI(TAG, "[HOME] EIP seek -%s limit %s at %.1f mm/s",
             ax, minCode, eipSeekSpeedMmS());
    eipLimitPhase_ = EipLimitPhase::kHomeSeekMin;
}

void Gantry::startEipPrecisionCalibrate(EipLimitAxisRole role) {
    if (role == EipLimitAxisRole::kX &&
        !requireXTraverseInterlock("EIP calibrate X")) {
        calibrationInProgress_ = false;
        eipLimitPhase_ = EipLimitPhase::kIdle;
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    eipLimitAxis_ = role;
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    abortRequested_ = false;
    calibrationInProgress_ = true;
    eipLimitSawBusy_ = false;
    eipLimitPhaseStartMs_ = gantry_millis();
    const char* ax = EipLimit::axisLetter(role);
    const char* maxCode = EipLimit::maxWarningLabel(role);

    if (eipMaxWarningActive()) {
        ESP_LOGI(TAG,
                 "[CAL] EIP already on +%s limit %s - creep toward -%s until clear",
                 ax, maxCode, ax);
        eipLimitPhase_ = EipLimitPhase::kCalCreepClear;
        return;
    }

    ESP_LOGI(TAG, "[CAL] EIP seek +%s limit %s at %.1f mm/s",
             ax, maxCode, eipCalSeekSpeedMmS());
    eipLimitPhase_ = EipLimitPhase::kCalSeekMax;
}

void Gantry::finishEipHomeOk() {
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        failEipLimitSequence("no active axis");
        return;
    }
    axis->stopMotion();
    eipLimitPhase_ = EipLimitPhase::kHomeSettle;
    eipLimitPhaseStartMs_ = gantry_millis();
    eipLimitSawBusy_ = false;
    const char* ax = EipLimit::axisLetter(eipLimitAxis_);
    ESP_LOGI(TAG,
             "[HOME] EIP -%s limit cleared - StopMotion hold, then set joint zero",
             ax);
}

void Gantry::finishEipCalOk() {
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        failEipLimitSequence("no active axis");
        return;
    }
    axis->stopMotion();
    const float max_mm = axis->getCurrentMm();
    const int32_t length = static_cast<int32_t>(llround(max_mm));
    const char* ax = EipLimit::axisLetter(eipLimitAxis_);

    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        zAxisLength_ = length < 1 ? 1 : length;
        config_.limits.z_max = static_cast<float>(zAxisLength_);
        if (config_.limits.z_max < config_.limits.z_min + 1.0f) {
            config_.limits.z_max = config_.limits.z_min + 1.0f;
        }
        currentZ_ = (int32_t)max_mm;
        ESP_LOGI(TAG,
                 "[CAL] EIP +Z limit %s cleared - joint max ~ %.3f mm (%ld); "
                 "hold-here then return to 0",
                 EipLimit::maxWarningLabel(EipLimitAxisRole::kZ), max_mm,
                 (long)zAxisLength_);
    } else {
        axisLength_ = length < 1 ? 1 : length;
        currentX_mm_ = max_mm;
        ESP_LOGI(TAG,
                 "[CAL] EIP +X limit %s cleared - joint max ~ %.3f mm (%ld); "
                 "hold-here then return to 0",
                 EipLimit::maxWarningLabel(EipLimitAxisRole::kX), max_mm,
                 (long)axisLength_);
    }
    eipLimitPhase_ = EipLimitPhase::kCalReturnZero;
    eipLimitPhaseStartMs_ = gantry_millis();
    calibrationInProgress_ = true;
    eipLimitSawBusy_ = false;
    (void)ax;
}

void Gantry::failEipLimitSequence(const char* why) {
    ESP_LOGE(TAG, "[EIP-LIMIT] failed: %s", why ? why : "?");
    if (GantryLinearAxis* axis = eipLimitActiveAxis()) {
        axis->stopMotion();
    }
    eipLimitPhase_ = EipLimitPhase::kIdle;
    eipLimitAxis_ = EipLimitAxisRole::kNone;
    eipLimitSawBusy_ = false;
    homingInProgress_ = false;
    calibrationInProgress_ = false;
}

void Gantry::advanceEipLimitSequence() {
    if (abortRequested_) {
        failEipLimitSequence("abort requested");
        return;
    }
    constexpr unsigned long kPhaseTimeoutMs = TRAVEL_MEASUREMENT_TIMEOUT_MS;
    if ((gantry_millis() - eipLimitPhaseStartMs_) > kPhaseTimeoutMs) {
        failEipLimitSequence("phase timeout");
        return;
    }

    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        failEipLimitSequence("no active axis");
        return;
    }
    const char* ax = EipLimit::axisLetter(eipLimitAxis_);

    switch (eipLimitPhase_) {
        case EipLimitPhase::kHomeSeekMin: {
            using EipLimit::SeekOutcome;
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMinWarningActive(), axis->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                ESP_LOGI(TAG,
                         "[HOME] -%s limit tripped - will creep toward +%s @ %.1f mm/s",
                         ax, ax, eipCreepSpeedMmS());
                eipLimitPhase_ = EipLimitPhase::kHomeCreepClear;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && axis->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!eipStartMoveDelta(EipLimit::homeSeekDeltaMm(),
                                       eipSeekSpeedMmS())) {
                    failEipLimitSequence("seek min limit start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipLimitSequence(
                    eipLimitAxis_ == EipLimitAxisRole::kZ
                        ? "seek -Z ended without trip (check A015/NL at joint -Z)"
                        : "seek -X ended without trip (check A014/PL at joint -X)");
            }
            break;
        }

        case EipLimitPhase::kHomeCreepClear: {
            using EipLimit::CreepOutcome;
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMinWarningActive(), axis->isBusy());
            if (o == CreepOutcome::kCleared) {
                finishEipHomeOk();
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!eipStartMoveDelta(EipLimit::homeCreepDeltaMm(),
                                          eipCreepSpeedMmS())) {
                failEipLimitSequence("home creep start failed");
            }
            break;
        }

        case EipLimitPhase::kHomeSettle: {
            if (!axis->isBusy()) {
                axis->setCurrentPulses(0);
                if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
                    currentZ_ = (int32_t)axis->getCurrentMm();
                    ESP_LOGI(TAG,
                             "[HOME] EIP joint zero set here (Z=%.3f mm reported)",
                             (float)currentZ_);
                } else {
                    currentX_mm_ = axis->getCurrentMm();
                    ESP_LOGI(TAG,
                             "[HOME] EIP joint zero set here (%.3f mm reported)",
                             currentX_mm_);
                }
                homingInProgress_ = false;
                if (calibrationInProgress_) {
                    const char* maxCode = EipLimit::maxWarningLabel(eipLimitAxis_);
                    ESP_LOGI(TAG, "[CAL] EIP seek +%s limit %s at %.1f mm/s",
                             ax, maxCode, eipCalSeekSpeedMmS());
                    eipLimitPhase_ = EipLimitPhase::kCalSeekMax;
                    eipLimitPhaseStartMs_ = gantry_millis();
                    eipLimitSawBusy_ = false;
                } else {
                    eipLimitPhase_ = EipLimitPhase::kIdle;
                    eipLimitAxis_ = EipLimitAxisRole::kNone;
                    ESP_LOGI(TAG, "[HOME] EIP complete");
                }
            }
            break;
        }

        case EipLimitPhase::kCalSeekMax: {
            using EipLimit::SeekOutcome;
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMaxWarningActive(), axis->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                ESP_LOGI(TAG,
                         "[CAL] +%s limit tripped - will creep toward -%s @ %.1f mm/s",
                         ax, ax, eipCreepSpeedMmS());
                eipLimitPhase_ = EipLimitPhase::kCalCreepClear;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && axis->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!eipStartMoveDelta(EipLimit::calSeekDeltaMm(),
                                       eipCalSeekSpeedMmS())) {
                    failEipLimitSequence("seek max limit start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipLimitSequence(
                    eipLimitAxis_ == EipLimitAxisRole::kZ
                        ? "seek +Z ended without trip (check A014/PL at joint +Z)"
                        : "seek +X ended without trip (check A015/NL at joint +X)");
            }
            break;
        }

        case EipLimitPhase::kCalCreepClear: {
            using EipLimit::CreepOutcome;
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMaxWarningActive(), axis->isBusy());
            if (o == CreepOutcome::kCleared) {
                finishEipCalOk();
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!eipStartMoveDelta(EipLimit::calCreepDeltaMm(),
                                          eipCreepSpeedMmS())) {
                failEipLimitSequence("cal creep start failed");
            }
            break;
        }

        case EipLimitPhase::kCalReturnZero: {
            if (!axis->isBusy()) {
                if (!eipLimitSawBusy_) {
                    if (std::fabs(axis->getCurrentMm()) > 0.5f) {
                        const float accel =
                            (acceleration_mm_per_s2_ > 0)
                                ? static_cast<float>(acceleration_mm_per_s2_)
                                : 500.0f;
                        const float decel =
                            (deceleration_mm_per_s2_ > 0)
                                ? static_cast<float>(deceleration_mm_per_s2_)
                                : accel;
                        if (!axis->moveToMm(0.0f, eipCalSeekSpeedMmS(), accel,
                                            decel)) {
                            failEipLimitSequence("return to 0 start failed");
                            break;
                        }
                        eipLimitSawBusy_ = true;
                        eipLimitPhaseStartMs_ = gantry_millis();
                    } else {
                        if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
                            currentZ_ = (int32_t)axis->getCurrentMm();
                            ESP_LOGI(TAG,
                                     "[CAL] EIP complete - at joint 0 (Z=%.3f mm)",
                                     (float)currentZ_);
                        } else {
                            currentX_mm_ = axis->getCurrentMm();
                            ESP_LOGI(TAG,
                                     "[CAL] EIP complete - at joint 0 (%.3f mm)",
                                     currentX_mm_);
                        }
                        eipLimitPhase_ = EipLimitPhase::kIdle;
                        eipLimitAxis_ = EipLimitAxisRole::kNone;
                        calibrationInProgress_ = false;
                    }
                } else {
                    eipLimitSawBusy_ = false;
                    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
                        currentZ_ = (int32_t)axis->getCurrentMm();
                        ESP_LOGI(TAG,
                                 "[CAL] EIP complete - at joint 0 (Z=%.3f mm)",
                                 (float)currentZ_);
                    } else {
                        currentX_mm_ = axis->getCurrentMm();
                        ESP_LOGI(TAG,
                                 "[CAL] EIP complete - at joint 0 (%.3f mm)",
                                 currentX_mm_);
                    }
                    eipLimitPhase_ = EipLimitPhase::kIdle;
                    eipLimitAxis_ = EipLimitAxisRole::kNone;
                    calibrationInProgress_ = false;
                }
            }
            break;
        }

        case EipLimitPhase::kIdle:
        default:
            break;
    }
}

// ============================================================================
// EIP BRING-UP (Z- → X home/cal → X park → Z+ stroke → return 0)
// ============================================================================

bool Gantry::eipBringUpInProgress() const {
    return bringUpPhase_ != BringUpPhase::kIdle;
}

bool Gantry::startEipBringUp() {
    GANTRY_CHECK_INITIALIZED_RET(false);
    GANTRY_CHECK_ENABLED_RET(false);
    if (!driveManagedLimitsEnabled()) {
        ESP_LOGE(TAG, "[BRINGUP] requires drive-managed EIP limits");
        return false;
    }
    if (!axisX_ || !axisZ_) {
        ESP_LOGE(TAG, "[BRINGUP] needs both X and Z axes");
        return false;
    }
    if (bringUpPhase_ != BringUpPhase::kIdle ||
        eipLimitPhase_ != EipLimitPhase::kIdle ||
        motionState_ != MotionState::IDLE) {
        ESP_LOGE(TAG, "[BRINGUP] busy — finish or stop first");
        return false;
    }
    if (axisX_->isAlarmActive() || axisZ_->isAlarmActive()) {
        ESP_LOGE(TAG, "[BRINGUP] alarm active");
        return false;
    }

    abortRequested_ = false;
    homingInProgress_ = true;
    calibrationInProgress_ = true;
    axisLength_ = 0;
    zAxisLength_ = 0;
    eipLimitSawBusy_ = false;
    eipLimitPhaseStartMs_ = gantry_millis();
    eipLimitAxis_ = EipLimitAxisRole::kZ;
    eipLimitPhase_ = EipLimitPhase::kIdle;

    ESP_LOGI(TAG,
             "[BRINGUP] start: Z- datum (A015) -> X home/cal -> X=%.1f "
             "-> Z+ stroke (A014) -> SAFE_Z band ceiling "
             "(SAFE_Z=%.1f mm from Z-/A015; Z invert_dir; "
             "X-:A014/PL, X+:A015/NL)",
             calXParkMm_, safeZMarginFromMaxMm_);

    if (eipMinWarningActive()) {
        ESP_LOGI(TAG, "[BRINGUP] already on -Z A015 — creep clear");
        bringUpPhase_ = BringUpPhase::kZMinusCreep;
    } else {
        ESP_LOGI(TAG, "[BRINGUP] seek -Z A015 at %.1f mm/s", eipSeekSpeedMmS());
        bringUpPhase_ = BringUpPhase::kZMinusSeek;
    }
    return true;
}

void Gantry::failEipBringUp(const char* why) {
    ESP_LOGE(TAG, "[BRINGUP] failed: %s", why ? why : "?");
    if (axisX_) axisX_->stopMotion();
    if (axisZ_) axisZ_->stopMotion();
    bringUpPhase_ = BringUpPhase::kIdle;
    eipLimitPhase_ = EipLimitPhase::kIdle;
    eipLimitAxis_ = EipLimitAxisRole::kNone;
    eipLimitSawBusy_ = false;
    homingInProgress_ = false;
    calibrationInProgress_ = false;
}

void Gantry::finishEipBringUpOk() {
    ESP_LOGI(TAG,
             "[BRINGUP] complete: X stroke=%ld mm, Z stroke=%ld mm, "
             "SAFE_Z ceiling=%.1f mm (z_min + %.1f)",
             (long)axisLength_, (long)zAxisLength_,
             traverseClearanceZMm(), safeZMarginFromMaxMm_);
    bringUpPhase_ = BringUpPhase::kIdle;
    eipLimitPhase_ = EipLimitPhase::kIdle;
    eipLimitAxis_ = EipLimitAxisRole::kNone;
    eipLimitSawBusy_ = false;
    homingInProgress_ = false;
    calibrationInProgress_ = false;
}

void Gantry::advanceEipBringUp() {
    if (abortRequested_) {
        failEipBringUp("abort requested");
        return;
    }
    constexpr unsigned long kPhaseTimeoutMs = TRAVEL_MEASUREMENT_TIMEOUT_MS;
    if ((gantry_millis() - eipLimitPhaseStartMs_) > kPhaseTimeoutMs) {
        failEipBringUp("phase timeout");
        return;
    }

    using EipLimit::CreepOutcome;
    using EipLimit::SeekOutcome;

    auto startDelta = [this](float delta, float speed) -> bool {
        return eipStartMoveDelta(delta, speed);
    };

    switch (bringUpPhase_) {
        case BringUpPhase::kZMinusSeek: {
            eipLimitAxis_ = EipLimitAxisRole::kZ;
            GantryLinearAxis* z = axisZ_.get();
            if (!z) { failEipBringUp("no Z"); return; }
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMinWarningActive(), z->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                ESP_LOGI(TAG, "[BRINGUP] -Z A015 tripped — creep clear");
                bringUpPhase_ = BringUpPhase::kZMinusCreep;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && z->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!startDelta(EipLimit::homeSeekDeltaMm(), eipSeekSpeedMmS())) {
                    failEipBringUp("Z- seek start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipBringUp("Z- seek ended without A015");
            }
            break;
        }

        case BringUpPhase::kZMinusCreep: {
            eipLimitAxis_ = EipLimitAxisRole::kZ;
            GantryLinearAxis* z = axisZ_.get();
            if (!z) { failEipBringUp("no Z"); return; }
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMinWarningActive(), z->isBusy());
            if (o == CreepOutcome::kCleared) {
                z->stopMotion();
                bringUpPhase_ = BringUpPhase::kZMinusSettle;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
                ESP_LOGI(TAG, "[BRINGUP] -Z cleared — set joint Z=0");
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!startDelta(EipLimit::homeCreepDeltaMm(), eipCreepSpeedMmS())) {
                failEipBringUp("Z- creep start failed");
            }
            break;
        }

        case BringUpPhase::kZMinusSettle: {
            eipLimitAxis_ = EipLimitAxisRole::kZ;
            GantryLinearAxis* z = axisZ_.get();
            if (!z) { failEipBringUp("no Z"); return; }
            if (!z->isBusy()) {
                z->setCurrentPulses(0);
                currentZ_ = (int32_t)z->getCurrentMm();
                // SAFE_Z is bottom band from Z−/A015 — already in band at Z≈0.
                ESP_LOGI(TAG,
                         "[BRINGUP] Z=0 at -Z/A015 clear (SAFE_Z ceiling=%.1f); "
                         "starting X home",
                         traverseClearanceZMm());
                eipLimitAxis_ = EipLimitAxisRole::kX;
                eipLimitSawBusy_ = false;
                eipLimitPhaseStartMs_ = gantry_millis();
                if (eipMinWarningActive()) {
                    bringUpPhase_ = BringUpPhase::kXHomeCreep;
                } else {
                    bringUpPhase_ = BringUpPhase::kXHomeSeek;
                    ESP_LOGI(TAG, "[BRINGUP] seek -X A014 at %.1f mm/s",
                             eipSeekSpeedMmS());
                }
            }
            break;
        }

        case BringUpPhase::kZEnterBand: {
            // Legacy top-band lift phase; unused with Z− SAFE_Z. Fall through to X.
            eipLimitAxis_ = EipLimitAxisRole::kX;
            eipLimitSawBusy_ = false;
            eipLimitPhaseStartMs_ = gantry_millis();
            if (eipMinWarningActive()) {
                bringUpPhase_ = BringUpPhase::kXHomeCreep;
            } else {
                bringUpPhase_ = BringUpPhase::kXHomeSeek;
            }
            break;
        }

        case BringUpPhase::kXHomeSeek: {
            eipLimitAxis_ = EipLimitAxisRole::kX;
            GantryLinearAxis* x = axisX_.get();
            if (!x) { failEipBringUp("no X"); return; }
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMinWarningActive(), x->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                bringUpPhase_ = BringUpPhase::kXHomeCreep;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && x->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!startDelta(EipLimit::homeSeekDeltaMm(), eipSeekSpeedMmS())) {
                    failEipBringUp("X- seek start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipBringUp("X- seek ended without A014");
            }
            break;
        }

        case BringUpPhase::kXHomeCreep: {
            eipLimitAxis_ = EipLimitAxisRole::kX;
            GantryLinearAxis* x = axisX_.get();
            if (!x) { failEipBringUp("no X"); return; }
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMinWarningActive(), x->isBusy());
            if (o == CreepOutcome::kCleared) {
                x->stopMotion();
                bringUpPhase_ = BringUpPhase::kXHomeSettle;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!startDelta(EipLimit::homeCreepDeltaMm(), eipCreepSpeedMmS())) {
                failEipBringUp("X- creep start failed");
            }
            break;
        }

        case BringUpPhase::kXHomeSettle: {
            eipLimitAxis_ = EipLimitAxisRole::kX;
            GantryLinearAxis* x = axisX_.get();
            if (!x) { failEipBringUp("no X"); return; }
            if (!x->isBusy()) {
                x->setCurrentPulses(0);
                currentX_mm_ = x->getCurrentMm();
                ESP_LOGI(TAG, "[BRINGUP] X=0 at -X clear; seek +X A015");
                eipLimitSawBusy_ = false;
                eipLimitPhaseStartMs_ = gantry_millis();
                if (eipMaxWarningActive()) {
                    bringUpPhase_ = BringUpPhase::kXCalCreep;
                } else {
                    bringUpPhase_ = BringUpPhase::kXCalSeek;
                }
            }
            break;
        }

        case BringUpPhase::kXCalSeek: {
            eipLimitAxis_ = EipLimitAxisRole::kX;
            GantryLinearAxis* x = axisX_.get();
            if (!x) { failEipBringUp("no X"); return; }
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMaxWarningActive(), x->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                bringUpPhase_ = BringUpPhase::kXCalCreep;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && x->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!startDelta(EipLimit::calSeekDeltaMm(), eipCalSeekSpeedMmS())) {
                    failEipBringUp("X+ seek start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipBringUp("X+ seek ended without A015");
            }
            break;
        }

        case BringUpPhase::kXCalCreep: {
            eipLimitAxis_ = EipLimitAxisRole::kX;
            GantryLinearAxis* x = axisX_.get();
            if (!x) { failEipBringUp("no X"); return; }
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMaxWarningActive(), x->isBusy());
            if (o == CreepOutcome::kCleared) {
                x->stopMotion();
                bringUpPhase_ = BringUpPhase::kXCalSettle;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!startDelta(EipLimit::calCreepDeltaMm(), eipCreepSpeedMmS())) {
                failEipBringUp("X+ creep start failed");
            }
            break;
        }

        case BringUpPhase::kXCalSettle: {
            eipLimitAxis_ = EipLimitAxisRole::kX;
            GantryLinearAxis* x = axisX_.get();
            if (!x) { failEipBringUp("no X"); return; }
            if (!x->isBusy()) {
                const float max_mm = x->getCurrentMm();
                axisLength_ = static_cast<int32_t>(llround(max_mm));
                if (axisLength_ < 1) axisLength_ = 1;
                config_.limits.x_max = static_cast<float>(axisLength_);
                currentX_mm_ = max_mm;
                ESP_LOGI(TAG,
                         "[BRINGUP] X stroke=%ld mm; park X=%.1f before Z+",
                         (long)axisLength_, calXParkMm_);
                if (!requireXTraverseInterlock("bringup X park")) {
                    failEipBringUp("X park blocked (Z out of SAFE_Z band)");
                    break;
                }
                const float accel =
                    (acceleration_mm_per_s2_ > 0)
                        ? static_cast<float>(acceleration_mm_per_s2_)
                        : 500.0f;
                const float decel =
                    (deceleration_mm_per_s2_ > 0)
                        ? static_cast<float>(deceleration_mm_per_s2_)
                        : accel;
                if (!x->moveToMm(calXParkMm_, eipCalSeekSpeedMmS(), accel, decel)) {
                    failEipBringUp("X park start failed");
                    break;
                }
                bringUpPhase_ = BringUpPhase::kXPark;
                eipLimitSawBusy_ = true;
                eipLimitPhaseStartMs_ = gantry_millis();
            }
            break;
        }

        case BringUpPhase::kXPark: {
            eipLimitAxis_ = EipLimitAxisRole::kX;
            GantryLinearAxis* x = axisX_.get();
            if (!x) { failEipBringUp("no X"); return; }
            if (!x->isBusy()) {
                currentX_mm_ = x->getCurrentMm();
                ESP_LOGI(TAG, "[BRINGUP] at X=%.3f; seek +Z A014 for stroke",
                         currentX_mm_);
                eipLimitAxis_ = EipLimitAxisRole::kZ;
                eipLimitSawBusy_ = false;
                eipLimitPhaseStartMs_ = gantry_millis();
                if (eipMaxWarningActive()) {
                    bringUpPhase_ = BringUpPhase::kZPlusCreep;
                } else {
                    bringUpPhase_ = BringUpPhase::kZPlusSeek;
                }
            }
            break;
        }

        case BringUpPhase::kZPlusSeek: {
            eipLimitAxis_ = EipLimitAxisRole::kZ;
            GantryLinearAxis* z = axisZ_.get();
            if (!z) { failEipBringUp("no Z"); return; }
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMaxWarningActive(), z->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                ESP_LOGI(TAG, "[BRINGUP] +Z A014 tripped — creep clear");
                bringUpPhase_ = BringUpPhase::kZPlusCreep;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && z->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!startDelta(EipLimit::calSeekDeltaMm(), eipCalSeekSpeedMmS())) {
                    failEipBringUp("Z+ seek start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipBringUp("Z+ seek ended without A014");
            }
            break;
        }

        case BringUpPhase::kZPlusCreep: {
            eipLimitAxis_ = EipLimitAxisRole::kZ;
            GantryLinearAxis* z = axisZ_.get();
            if (!z) { failEipBringUp("no Z"); return; }
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMaxWarningActive(), z->isBusy());
            if (o == CreepOutcome::kCleared) {
                z->stopMotion();
                bringUpPhase_ = BringUpPhase::kZPlusSettle;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!startDelta(EipLimit::calCreepDeltaMm(), eipCreepSpeedMmS())) {
                failEipBringUp("Z+ creep start failed");
            }
            break;
        }

        case BringUpPhase::kZPlusSettle: {
            eipLimitAxis_ = EipLimitAxisRole::kZ;
            GantryLinearAxis* z = axisZ_.get();
            if (!z) { failEipBringUp("no Z"); return; }
            if (!z->isBusy()) {
                const float max_mm = z->getCurrentMm();
                zAxisLength_ = static_cast<int32_t>(llround(max_mm));
                if (zAxisLength_ < 1) zAxisLength_ = 1;
                config_.limits.z_min = 0.0f;
                config_.limits.z_max = static_cast<float>(zAxisLength_);
                currentZ_ = (int32_t)max_mm;
                ESP_LOGI(TAG,
                         "[BRINGUP] Z stroke=%.3f mm (%ld); return to SAFE_Z "
                         "ceiling %.1f",
                         max_mm, (long)zAxisLength_, traverseClearanceZMm());
                const float accel =
                    (acceleration_mm_per_s2_ > 0)
                        ? static_cast<float>(acceleration_mm_per_s2_)
                        : 500.0f;
                const float decel =
                    (deceleration_mm_per_s2_ > 0)
                        ? static_cast<float>(deceleration_mm_per_s2_)
                        : accel;
                const float band = traverseClearanceZMm();
                if (std::fabs(max_mm - band) > 0.5f) {
                    if (!z->moveToMm(band, eipCalSeekSpeedMmS(), accel, decel)) {
                        failEipBringUp("Z return-to-band start failed");
                        break;
                    }
                    bringUpPhase_ = BringUpPhase::kZReturnBand;
                    eipLimitSawBusy_ = true;
                    eipLimitPhaseStartMs_ = gantry_millis();
                } else {
                    finishEipBringUpOk();
                }
            }
            break;
        }

        case BringUpPhase::kZReturnBand: {
            eipLimitAxis_ = EipLimitAxisRole::kZ;
            GantryLinearAxis* z = axisZ_.get();
            if (!z) { failEipBringUp("no Z"); return; }
            if (!z->isBusy()) {
                currentZ_ = (int32_t)z->getCurrentMm();
                finishEipBringUpOk();
            }
            break;
        }

        case BringUpPhase::kIdle:
        default:
            break;
    }
}

// ============================================================================
// PATH / SEQUENTIAL MOTION
// ============================================================================

bool Gantry::planPathTo(float x0, float z0, float x1, float z1) {
    const float clearance = traverseClearanceZMm();
    pathSegmentCount_ = Path::planSegments(x0, z0, x1, z1, clearance,
                                           pathSegments_);
    pathSegmentIndex_ = 0;
    pathSegStartX_mm_ = x0;
    pathSegStartZ_mm_ = z0;
    return pathSegmentCount_ > 0;
}

bool Gantry::pathSegmentAxesIdle() const {
    if (pathSegmentIndex_ >= pathSegmentCount_) {
        return true;
    }
    const Path::PathSegment& seg = pathSegments_[pathSegmentIndex_];
    if (seg.move_x && axisX_ && axisX_->isMotionActive()) {
        return false;
    }
    if (seg.move_z && axisZ_ && axisZ_->isBusy()) {
        return false;
    }
    return true;
}

bool Gantry::pathSegmentAtTarget() const {
    if (pathSegmentIndex_ >= pathSegmentCount_) {
        return true;
    }
    const Path::PathSegment& seg = pathSegments_[pathSegmentIndex_];
    const float x = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float z = axisZ_ ? axisZ_->getCurrentMm() : static_cast<float>(currentZ_);
    // Wider than drive arrival band (0.5 mm): catch timeout/abort that
    // clears busy far from the commanded endpoint (e.g. X never moved).
    constexpr float kPathArriveEpsMm = 1.0f;
    return Path::segmentEndpointsReached(x, z, seg, kPathArriveEpsMm);
}

void Gantry::failPath(const char* why) {
    ESP_LOGE(TAG, "[PATH] abort: %s", why ? why : "?");
    stopAllMotion();
}

bool Gantry::armCurrentPathSegment() {
    if (pathSegmentIndex_ >= pathSegmentCount_) {
        return false;
    }

    const Path::PathSegment& seg = pathSegments_[pathSegmentIndex_];
    const float dx = seg.x_mm - pathSegStartX_mm_;
    const float dz = seg.z_mm - pathSegStartZ_mm_;
    const Path::PathProfile profile{
        static_cast<float>(speed_mm_per_s_),
        static_cast<float>(acceleration_mm_per_s2_),
        static_cast<float>(deceleration_mm_per_s2_),
    };
    const Path::AxisComponents c = Path::decompose(dx, dz, profile);

    ESP_LOGI(TAG,
             "[PATH] seg %u/%u -> (%.3f, %.3f) move_x=%d move_z=%d "
             "vx=%.1f vz=%.1f ax=%.1f az=%.1f",
             (unsigned)(pathSegmentIndex_ + 1), (unsigned)pathSegmentCount_,
             seg.x_mm, seg.z_mm, (int)seg.move_x, (int)seg.move_z,
             c.x_speed, c.z_speed, c.x_accel, c.z_accel);

    // Arm both axes BEFORE publishing motionState_ (A603 race guard).
    if (seg.move_z) {
        if (!moveZAxisTo(seg.z_mm, c.z_speed, c.z_accel, c.z_decel)) {
            ESP_LOGE(TAG, "[PATH] Z arm failed");
            stopAllMotion();
            return false;
        }
    }
    if (seg.move_x) {
        if (!startXAxisMotion(seg.x_mm, c.x_speed, c.x_accel, c.x_decel)) {
            ESP_LOGE(TAG, "[PATH] X arm failed");
            stopAllMotion();
            return false;
        }
    }

    if (seg.move_x && seg.move_z) {
        motionState_ = MotionState::XZ_MOVING;
    } else if (seg.move_x) {
        motionState_ = MotionState::X_MOVING;
    } else if (seg.move_z) {
        motionState_ = (seg.z_mm < pathSegStartZ_mm_)
                           ? MotionState::Z_DESCENDING
                           : MotionState::Z_RETRACTING;
    } else {
        return false;
    }
    return true;
}

void Gantry::startThetaOrIdle() {
#if CONFIG_GANTRY_THETA_SEQUENTIAL
    if (axisTheta_) {
        motionState_ = MotionState::THETA_MOVING;
        if (!axisTheta_->moveToDeg(targetTheta_deg_,
                                   (float)speed_deg_per_s_,
                                   (float)acceleration_mm_per_s2_,
                                   (float)deceleration_mm_per_s2_)) {
            const float currentTheta = axisTheta_->getCurrentDeg();
            if (std::fabs(currentTheta - targetTheta_deg_) > 0.5f) {
                ESP_LOGE(TAG,
                         "[THETA] moveToDeg failed (current=%.2f target=%.2f) - aborting sequence",
                         currentTheta, targetTheta_deg_);
                stopAllMotion();
                return;
            }
        }
        return;
    }
#endif
    motionState_ = MotionState::IDLE;
    pathSegmentCount_ = 0;
    pathSegmentIndex_ = 0;
}

void Gantry::finishPathOrAdvance() {
    if (pathSegmentIndex_ < pathSegmentCount_) {
        const Path::PathSegment& done = pathSegments_[pathSegmentIndex_];
        pathSegStartX_mm_ = done.x_mm;
        pathSegStartZ_mm_ = done.z_mm;
        ++pathSegmentIndex_;
    }

    if (pathSegmentIndex_ < pathSegmentCount_) {
        if (!armCurrentPathSegment()) {
            return;
        }
        return;
    }

    // Path complete.
    pathSegmentCount_ = 0;
    pathSegmentIndex_ = 0;

    if (pathDoGripperAfter_) {
        pathDoGripperAfter_ = false;
        motionState_ = MotionState::GRIPPER_ACTUATING;
        grip(gripperTargetState_);
        gripperActuateStart_ms_ = gantry_millis();
        return;
    }

    startThetaOrIdle();
}

void Gantry::startSequentialMotion() {
    if (!enabled_ || motionState_ != MotionState::IDLE) {
        return;
    }

    const float currentX = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float currentZ = axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_;

    // Descending (Z decreasing, toward belt) = picking (grip close).
    // Ascending (Z increasing, away from belt) = placing (grip open).
    gripperTargetState_ = (targetZ_mm_ < currentZ);
    gripperActuateDurationMs_ = gripperTargetState_
        ? GRIPPER_ACTUATE_TIME_MS
        : GRIPPER_ACTUATE_TIME_MS;

    pathDoGripperAfter_ = false;

    if (jointDirectMove_) {
        ESP_LOGI(TAG,
                 "[JOINT_DIRECT] path (%.3f,%.3f) -> (%.3f,%.3f) SAFE_Z "
                 "ceiling=%.1f (z_min=%.1f margin=%.1f)",
                 currentX, currentZ, targetX_mm_, targetZ_mm_,
                 traverseClearanceZMm(), config_.limits.z_min,
                 safeZMarginFromMaxMm_);
        if (!planPathTo(currentX, currentZ, targetX_mm_, targetZ_mm_)) {
            ESP_LOGI(TAG, "[JOINT_DIRECT] already at target");
            startThetaOrIdle();
        } else if (!armCurrentPathSegment()) {
            return;
        }
    } else {
        // PnP / pose: path to pick/place target; gripper after descend if needed.
        pathDoGripperAfter_ = gripperTargetState_;
        ESP_LOGI(TAG,
                 "[PnP] path (%.3f,%.3f) -> (%.3f,%.3f) SAFE_Z ceiling=%.1f "
                 "(z_min=%.1f margin=%.1f) gripper_after=%d",
                 currentX, currentZ, targetX_mm_, targetZ_mm_,
                 traverseClearanceZMm(), config_.limits.z_min,
                 safeZMarginFromMaxMm_, (int)pathDoGripperAfter_);
        if (!planPathTo(currentX, currentZ, targetX_mm_, targetZ_mm_)) {
            if (pathDoGripperAfter_) {
                pathDoGripperAfter_ = false;
                motionState_ = MotionState::GRIPPER_ACTUATING;
                grip(gripperTargetState_);
                gripperActuateStart_ms_ = gantry_millis();
            } else {
                startThetaOrIdle();
            }
        } else if (!armCurrentPathSegment()) {
            return;
        }
    }

    if (axisTheta_) {
#if !CONFIG_GANTRY_THETA_SEQUENTIAL
        if (!axisTheta_->moveToDeg(targetTheta_deg_,
                                   (float)speed_deg_per_s_,
                                   (float)acceleration_mm_per_s2_,
                                   (float)deceleration_mm_per_s2_)) {
            const float currentTheta = axisTheta_->getCurrentDeg();
            if (std::fabs(currentTheta - targetTheta_deg_) > 0.5f) {
                ESP_LOGE(TAG,
                         "[THETA] moveToDeg failed (current=%.2f target=%.2f) - aborting sequence",
                         currentTheta, targetTheta_deg_);
                stopAllMotion();
                return;
            }
            ESP_LOGI(TAG,
                     "[THETA] moveToDeg no-op treated as success (current=%.2f target=%.2f)",
                     currentTheta, targetTheta_deg_);
        }
#endif
        currentTheta_ = (int32_t)axisTheta_->getCurrentDeg();
    } else {
        currentTheta_ = (int32_t)targetTheta_deg_;
    }
}

void Gantry::processSequentialMotion() {
    if (!enabled_) {
        stopAllMotion();
        return;
    }

    switch (motionState_) {
        case MotionState::Z_DESCENDING:
        case MotionState::Z_RETRACTING:
        case MotionState::X_MOVING:
        case MotionState::XZ_MOVING:
            if (pathSegmentAxesIdle()) {
                if (!pathSegmentAtTarget()) {
                    const Path::PathSegment& seg =
                        pathSegments_[pathSegmentIndex_];
                    const float x =
                        axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
                    const float z = axisZ_ ? axisZ_->getCurrentMm()
                                           : static_cast<float>(currentZ_);
                    ESP_LOGE(TAG,
                             "[PATH] seg %u ended off-target "
                             "(at %.3f,%.3f want %.3f,%.3f move_x=%d move_z=%d) "
                             "— refusing further segments",
                             (unsigned)(pathSegmentIndex_ + 1), x, z,
                             seg.x_mm, seg.z_mm, (int)seg.move_x,
                             (int)seg.move_z);
                    failPath("segment ended without reaching target");
                    return;
                }
                finishPathOrAdvance();
            }
            break;

        case MotionState::GRIPPER_ACTUATING:
            if (gantry_millis() - gripperActuateStart_ms_ >= gripperActuateDurationMs_) {
                const float currentX = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
                const float currentZ = axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_;
                // Retract to clearance at current X (planner yields Z-alone lift).
                const float clearance = traverseClearanceZMm();
                ESP_LOGI(TAG, "[PnP] post-grip to SAFE_Z ceiling Z=%.1f",
                         clearance);
                pathDoGripperAfter_ = false;
                if (!planPathTo(currentX, currentZ, currentX, clearance)) {
                    startThetaOrIdle();
                } else if (!armCurrentPathSegment()) {
                    return;
                }
            }
            break;

        case MotionState::THETA_MOVING:
            if (!axisTheta_ || !axisTheta_->isMotionActive()) {
                motionState_ = MotionState::IDLE;
            }
            break;

        case MotionState::IDLE:
        default:
            break;
    }
}

bool Gantry::startXAxisMotion(float target_mm, float speed_mm_s, float accel_mm_s2,
                              float decel_mm_s2) {
    if (!axisX_) {
        return false;
    }
    if (!requireXTraverseInterlock("path X Absolute")) {
        return false;
    }

    const float xPpm = getPulsesPerMm();

    const int32_t driverReportedPulses = (int32_t)axisX_->getCurrentPulses();
    const int32_t trackedPulses        = (int32_t)(currentX_mm_ * xPpm);
    const int32_t encoderPulses        = axisX_->getEncoderPulses();
    const int32_t targetPulses         = (int32_t)(target_mm * xPpm);

    if (axisX_->isEncoderFeedbackEnabled()) {
        const int32_t encDrvMismatch = encoderPulses - driverReportedPulses;
        if (encDrvMismatch > 10 || encDrvMismatch < -10) {
            ESP_LOGW(TAG,
                     "[X_MOVE] encoder/driver mismatch: encoder=%ld driver=%ld mismatch=%ld (planning uses driver)",
                     (long)encoderPulses, (long)driverReportedPulses, (long)encDrvMismatch);
        }
    }

    const int32_t trackedMismatch = trackedPulses - driverReportedPulses;
    if (trackedMismatch > 10 || trackedMismatch < -10) {
        ESP_LOGW(TAG,
                 "[X_MOVE] tracked/driver mismatch: tracked=%ld driver=%ld mismatch=%ld",
                 (long)trackedPulses, (long)driverReportedPulses, (long)trackedMismatch);
    }

    axisX_->setCurrentPulses((uint32_t)driverReportedPulses);
    const int32_t deltaX = targetPulses - driverReportedPulses;

    ESP_LOGI(TAG,
             "[X_MOVE] current=%ld tracked=%ld driver=%ld encoder=%ld target=%ld delta=%ld "
             "speed=%.1f accel=%.1f decel=%.1f",
             (long)driverReportedPulses, (long)trackedPulses, (long)driverReportedPulses,
             (long)encoderPulses, (long)targetPulses, (long)deltaX,
             speed_mm_s, accel_mm_s2, decel_mm_s2);

    if (deltaX == 0) {
        return true;  // already there; caller still owns motionState_
    }

    const uint32_t speed_pps = (uint32_t)(speed_mm_s * xPpm);
    uint32_t accel_pps = (accel_mm_s2 > 0.0f)
        ? (uint32_t)(accel_mm_s2 * xPpm) : 0;
    uint32_t decel_pps = (decel_mm_s2 > 0.0f)
        ? (uint32_t)(decel_mm_s2 * xPpm) : 0;
    if (accel_pps == 0) accel_pps = speed_pps / 2;
    if (decel_pps == 0) decel_pps = speed_pps / 2;

    const uint32_t target_counts = (uint32_t)((int32_t)driverReportedPulses + deltaX);
    if (!axisX_->moveToPulses(target_counts, speed_pps, accel_pps, decel_pps)) {
        ESP_LOGE(TAG,
                 "[X_MOVE] moveToPulses rejected (delta=%ld, speed_pps=%lu, accel_pps=%lu, decel_pps=%lu)",
                 (long)deltaX, (unsigned long)speed_pps,
                 (unsigned long)accel_pps, (unsigned long)decel_pps);
        return false;
    }
    return true;
}

} // namespace Gantry
