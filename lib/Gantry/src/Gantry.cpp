/**
 * @file Gantry.cpp
 * @brief Implementation of Gantry class.
 * @version 2.0.0
 */

#include "Gantry.h"
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
// PulseMotor constructor, factory methods, and preparePinsForBoot — REMOVED
// All drive control is EtherNet/IP over W5500. The DI constructor is the
// only remaining path. See Gantry.h for the updated constructor signature.
// ============================================================================

// Dependency-injection constructor — axisX_, axisZ_, axisTheta_ supplied.
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
    config_(),
    kinematicParams_(),
    stepsPerRev_(DEFAULT_STEPS_PER_REV),
    motionState_(MotionState::IDLE),
    targetX_mm_(0.0f),
    targetZ_mm_(0.0f),
    targetTheta_deg_(0.0f),
    safeZHeight_mm_(DEFAULT_SAFE_Z_HEIGHT_MM),
    speed_mm_per_s_(DEFAULT_SPEED_MM_PER_S),
    speed_deg_per_s_(DEFAULT_SPEED_DEG_PER_S),
    acceleration_mm_per_s2_(0),
    deceleration_mm_per_s2_(0),
    gripperTargetState_(false),
    gripperActuateStart_ms_(0),
    gripperActuateDurationMs_(GRIPPER_ACTUATE_TIME_MS),
    lastXPositionCounts_(0),
    xPulsesPerMmOverride_(0.0f),
    jointDirectMove_(false) {
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
    // Always re-issue axis enable so ServoOn gets a fresh 0→1 edge. Skipping
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
    if (safeHeight_mm > 0.0f) {
        safeZHeight_mm_ = safeHeight_mm;
    }
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
    // Z limit switches are defined in the pinout (PIN_Z_LIMIT_MIN/MAX) but the
    // Gantry class does not yet wire them up. Surface this as a warning so
    // operators know the command was received but did not act on hardware.
    ESP_LOGW(TAG, "[HOME] Z axis home not yet wired (Z limit switches not integrated)");
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
    ESP_LOGW(TAG, "[CAL] Z axis calibrate not yet wired (Z limit switches not integrated)");
    return 0;
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
    // alarm) — those branches have already returned. Reaching this point
    // with stableState=true is the success signal.
    //
    // DO NOT call xMaxSwitch_.update(true) here. Force-overwriting the
    // debounced stableState with the raw level at this instant races
    // against the carriage's residual deceleration overshooting the
    // inductive sensor's detection zone. The debounce that already
    // committed during the inner loop is the correct truth — the carriage
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
    // docs/LOW_LEVEL_GANTRY_CONTROL.md — wiring TODO). We measured the
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
    xMinSwitch_.update();
    xMaxSwitch_.update();

    if (!axisX_) {
        return;
    }

    uint32_t currentXCounts = axisX_->getCurrentPulses();
    if (axisX_->isMotionActive()) {
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
    } else if (homingInProgress_) {
        if (xMinSwitch_.isActive()) {
            axisX_->setCurrentPulses(0);
        }
        homingInProgress_ = false;
    }
    lastXPositionCounts_ = currentXCounts;

    currentX_mm_ = pulsesToMm((int32_t)currentXCounts);

    // Tick the X axis so it can run its MOVE-log state machine. The driver
    // itself runs on its own esp_timer, so update() is a logging-only hook
    // here (mirrors what's done for Z and Theta below).
    axisX_->update();

    if (axisZ_) {
        axisZ_->update();
        currentZ_ = (int32_t)axisZ_->getCurrentMm();
    }
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
    motionState_           = MotionState::IDLE;
    jointDirectMove_       = false;
}

// ============================================================================
// SEQUENTIAL MOTION
// ============================================================================

void Gantry::startSequentialMotion() {
    if (!enabled_ || motionState_ != MotionState::IDLE) {
        return;
    }

    const float currentZ = axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_;

    // Descending (Z decreasing, toward belt) = picking (grip close).
    // Ascending (Z increasing, away from belt) = placing (grip open).
    gripperTargetState_ = (targetZ_mm_ < currentZ);
    gripperActuateDurationMs_ = gripperTargetState_
        ? GRIPPER_ACTUATE_TIME_MS   // close; see docs (Constants)
        : GRIPPER_ACTUATE_TIME_MS;  // open;  see docs (Constants)

    // Start the axis move BEFORE publishing motionState_. Otherwise the gantry
    // update task can preempt between "Z_RETRACTING" and moveZAxisTo() and see
    // !isBusy(), advancing to X_MOVING while Z is still being armed — dual
    // StartMotion edges and A603 on both drives.
    if (jointDirectMove_) {
        // Soft bring-up / console: Z to joint target, then X (skip SAFE_Z=150).
        constexpr float kZEpsMm = 0.05f;
        if (std::fabs(targetZ_mm_ - currentZ) > kZEpsMm) {
            ESP_LOGI(TAG,
                     "[JOINT_DIRECT] Z %.3f -> %.3f mm, then X=%.3f",
                     currentZ, targetZ_mm_, targetX_mm_);
            if (!moveZAxisTo(targetZ_mm_, (float)speed_mm_per_s_,
                             (float)acceleration_mm_per_s2_,
                             (float)deceleration_mm_per_s2_)) {
                stopAllMotion();
                return;
            }
            motionState_ = (targetZ_mm_ < currentZ)
                               ? MotionState::Z_DESCENDING
                               : MotionState::Z_RETRACTING;
            if (!axisZ_) {
                motionState_ = MotionState::X_MOVING;
                startXAxisMotion();
            }
        } else {
            ESP_LOGI(TAG, "[JOINT_DIRECT] X=%.3f (Z already at target)",
                     targetX_mm_);
            motionState_ = MotionState::X_MOVING;
            startXAxisMotion();
        }
    } else if (currentZ < safeZHeight_mm_) {
        ESP_LOGI(TAG, "[PnP] Z retract %.3f -> SAFE_Z=%.3f mm", currentZ,
                 safeZHeight_mm_);
        if (!moveZAxisTo(safeZHeight_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_,
                         (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        motionState_ = MotionState::Z_RETRACTING;
        if (!axisZ_) {
            motionState_ = MotionState::X_MOVING;
            startXAxisMotion();
        }
    } else if (targetZ_mm_ < currentZ) {
        ESP_LOGI(TAG, "[PnP] Z descend %.3f -> %.3f mm", currentZ, targetZ_mm_);
        if (!moveZAxisTo(targetZ_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_,
                         (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        motionState_ = MotionState::Z_DESCENDING;
        if (!axisZ_) {
            motionState_ = MotionState::GRIPPER_ACTUATING;
            grip(gripperTargetState_);
            gripperActuateStart_ms_ = gantry_millis();
        }
    } else {
        ESP_LOGI(TAG, "[PnP] X direct (Z=%.3f >= SAFE_Z)", currentZ);
        motionState_ = MotionState::X_MOVING;
        startXAxisMotion();
    }

    if (axisTheta_) {
        if (!axisTheta_->moveToDeg(targetTheta_deg_,
                                   (float)speed_deg_per_s_,
                                   0.0f, 0.0f)) {
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
            if (!axisZ_ || !axisZ_->isBusy()) {
                if (jointDirectMove_) {
                    motionState_ = MotionState::X_MOVING;
                    startXAxisMotion();
                } else {
                    motionState_ = MotionState::GRIPPER_ACTUATING;
                    grip(gripperTargetState_);
                    gripperActuateStart_ms_ = gantry_millis();
                }
            }
            break;

        case MotionState::GRIPPER_ACTUATING:
            if (gantry_millis() - gripperActuateStart_ms_ >= gripperActuateDurationMs_) {
                if (!moveZAxisTo(safeZHeight_mm_, (float)speed_mm_per_s_,
                                 (float)acceleration_mm_per_s2_,
                                 (float)deceleration_mm_per_s2_)) {
                    stopAllMotion();
                    return;
                }
                motionState_ = MotionState::Z_RETRACTING;
                if (!axisZ_) {
                    motionState_ = MotionState::X_MOVING;
                    startXAxisMotion();
                }
            }
            break;

        case MotionState::Z_RETRACTING:
            if (!axisZ_ || !axisZ_->isBusy()) {
                motionState_ = MotionState::X_MOVING;
                startXAxisMotion();
            }
            break;

        case MotionState::X_MOVING:
            if (!axisX_ || !axisX_->isMotionActive()) {
                motionState_ = MotionState::IDLE;
            }
            break;

        case MotionState::THETA_MOVING:
            if (!axisTheta_) {
                motionState_ = MotionState::IDLE;
            }
            break;

        case MotionState::IDLE:
        default:
            break;
    }
}

void Gantry::startXAxisMotion() {
    if (!axisX_) {
        motionState_ = MotionState::IDLE;
        return;
    }

    const float xPpm = getPulsesPerMm();

    const int32_t driverReportedPulses = (int32_t)axisX_->getCurrentPulses();
    const int32_t trackedPulses        = (int32_t)(currentX_mm_ * xPpm);
    const int32_t encoderPulses        = axisX_->getEncoderPulses();
    const int32_t targetPulses         = (int32_t)(targetX_mm_ * xPpm);

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
             "[X_MOVE] current=%ld tracked=%ld driver=%ld encoder=%ld target=%ld delta=%ld speed=%lu accel=%lu decel=%lu",
             (long)driverReportedPulses, (long)trackedPulses, (long)driverReportedPulses,
             (long)encoderPulses, (long)targetPulses, (long)deltaX,
             (unsigned long)speed_mm_per_s_, (unsigned long)acceleration_mm_per_s2_,
             (unsigned long)deceleration_mm_per_s2_);

    if (deltaX == 0) {
        motionState_ = MotionState::IDLE;
        return;
    }

    const uint32_t speed_pps = (uint32_t)(speed_mm_per_s_ * xPpm);
    uint32_t accel_pps = (acceleration_mm_per_s2_ > 0)
        ? (uint32_t)(acceleration_mm_per_s2_ * xPpm) : 0;
    uint32_t decel_pps = (deceleration_mm_per_s2_ > 0)
        ? (uint32_t)(deceleration_mm_per_s2_ * xPpm) : 0;
    if (accel_pps == 0) accel_pps = speed_pps / 2;
    if (decel_pps == 0) decel_pps = speed_pps / 2;

    const uint32_t target_counts = (uint32_t)((int32_t)driverReportedPulses + deltaX);
    if (!axisX_->moveToPulses(target_counts, speed_pps, accel_pps, decel_pps)) {
        ESP_LOGE(TAG,
                 "[X_MOVE] moveToPulses rejected (delta=%ld, speed_pps=%lu, accel_pps=%lu, decel_pps=%lu)",
                 (long)deltaX, (unsigned long)speed_pps,
                 (unsigned long)accel_pps, (unsigned long)decel_pps);
        stopAllMotion();
    }
}

} // namespace Gantry
