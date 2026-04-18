/**
 * @file Gantry.cpp
 * @brief Three-axis pulse-train gantry controller implementation.
 * @version 2.0.0
 */

#include "Gantry.h"
#include <cmath>
#include "esp_log.h"

using namespace Gantry::Constants;

namespace Gantry {
static const char *TAG = "Gantry";

#ifndef GANTRY_DIAG_SKIP_AXIS_X_INIT
#define GANTRY_DIAG_SKIP_AXIS_X_INIT 0
#endif

#ifndef GANTRY_DIAG_SKIP_AXIS_Y_INIT
#define GANTRY_DIAG_SKIP_AXIS_Y_INIT 0
#endif

#ifndef GANTRY_DIAG_SKIP_THETA_INIT
#define GANTRY_DIAG_SKIP_THETA_INIT 0
#endif

// ============================================================================
// CONSTRUCTORS
// ============================================================================

Gantry::Gantry(const PulseMotor::DriverConfig &xConfig, int gripperPin)
    : Gantry(xConfig, PulseMotor::DriverConfig(), PulseMotor::DriverConfig(), gripperPin) {
}

Gantry::Gantry(const PulseMotor::DriverConfig &xConfig,
               const PulseMotor::DriverConfig &yConfig,
               int gripperPin)
    : Gantry(xConfig, yConfig, PulseMotor::DriverConfig(), gripperPin) {}

Gantry::Gantry(const PulseMotor::DriverConfig &xConfig,
               const PulseMotor::DriverConfig &yConfig,
               const PulseMotor::DriverConfig &thetaConfig,
               int gripperPin)
    : axisX_(xConfig), axisY_(yConfig), axisTheta_(thetaConfig),
      yConfigured_(yConfig.pulse_pin >= 0),
      yUseEncoder_(yConfig.enable_encoder_feedback),
      thetaConfigured_(thetaConfig.pulse_pin >= 0),
      thetaUseEncoder_(thetaConfig.enable_encoder_feedback),
      xDrivetrain_(), yDrivetrain_(), thetaDrivetrain_(),
      gripperPin_(gripperPin), xMinPin_(-1), xMaxPin_(-1),
      initialized_(false), enabled_(false), abortRequested_(false),
      homingInProgress_(false), calibrationInProgress_(false),
      gripperActive_(false),
      currentX_mm_(0.0f), currentY_(0), currentTheta_(0),
      targetY_(0), targetTheta_(0), axisLength_(0),
      config_(), kinematicParams_(),
      motionState_(MotionState::IDLE),
      targetX_mm_(0.0f), targetY_mm_(0.0f), targetTheta_deg_(0.0f),
      safeYHeight_mm_(DEFAULT_SAFE_Y_HEIGHT_MM),
      speed_mm_per_s_(DEFAULT_SPEED_MM_PER_S),
      speed_deg_per_s_(DEFAULT_SPEED_DEG_PER_S),
      acceleration_mm_per_s2_(0), deceleration_mm_per_s2_(0),
      gripperTargetState_(false), gripperActuateStart_ms_(0),
      lastXPositionCounts_(0) {
  // Populate default drivetrain scaling from driver-provided encoder PPRs so
  // simple single-config uses (no explicit setXDrivetrain call) still produce
  // plausible pulse/mm output in tests.
  xDrivetrain_.encoder_ppr = (float)xConfig.encoder_ppr;
  yDrivetrain_.encoder_ppr = (float)yConfig.encoder_ppr;
  thetaDrivetrain_.type = PulseMotor::DrivetrainType::ROTARY_DIRECT;
  thetaDrivetrain_.encoder_ppr = (float)thetaConfig.encoder_ppr;
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

    // Limit switches are handled at the Gantry layer; keep the driver internal
    // limit logic disabled for X.
    PulseMotor::DriverConfig xConfig = axisX_.getConfig();
    xConfig.limit_min_pin = -1;
    xConfig.limit_max_pin = -1;
    axisX_.setConfig(xConfig);

    xMinSwitch_.begin();
    xMaxSwitch_.begin();
    ESP_LOGI(TAG, "[BEGIN] Limit switch objects initialized");

    if (GANTRY_DIAG_SKIP_AXIS_X_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping X-axis initialize (diagnostic toggle)");
    } else {
        ESP_LOGI(TAG, "[BEGIN] Initializing X-axis driver");
        if (!axisX_.initialize()) {
            ESP_LOGE(TAG, "[BEGIN] X-axis initialize failed");
            return false;
        }
        ESP_LOGI(TAG, "[BEGIN] X-axis initialize OK");
    }

    if (GANTRY_DIAG_SKIP_AXIS_Y_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping Y-axis initialize (diagnostic toggle)");
    } else if (yConfigured_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing Y-axis driver");
        if (!axisY_.initialize()) {
            ESP_LOGE(TAG, "[BEGIN] Y-axis initialize failed");
            return false;
        }
        ESP_LOGI(TAG, "[BEGIN] Y-axis initialize OK");
    }

    if (GANTRY_DIAG_SKIP_THETA_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping theta initialize (diagnostic toggle)");
    } else if (thetaConfigured_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing theta driver");
        if (!axisTheta_.initialize()) {
            ESP_LOGE(TAG, "[BEGIN] Theta initialize failed");
            return false;
        }
        ESP_LOGI(TAG, "[BEGIN] Theta initialize OK");
    }

    initialized_ = true;
    ESP_LOGI(TAG, "[BEGIN] Gantry::begin() complete");
    return true;
}

// ============================================================================
// ENABLE / DISABLE
// ============================================================================

void Gantry::enable() {
    GANTRY_CHECK_INITIALIZED();
    bool xEnabled = axisX_.isEnabled() || axisX_.enable();
    if (yConfigured_) {
        if (xEnabled) {
            axisY_.enable();
        } else {
            axisY_.disable();
        }
    }
    if (thetaConfigured_) {
        if (xEnabled) {
            axisTheta_.enable();
        } else {
            axisTheta_.disable();
        }
    }
    enabled_ = xEnabled;
}

void Gantry::disable() {
    GANTRY_CHECK_INITIALIZED();
    if (isBusy()) {
        stopAllMotion();
    }
    axisX_.disable();
    if (yConfigured_) {
        axisY_.disable();
    }
    if (thetaConfigured_) {
        axisTheta_.disable();
    }
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

void Gantry::setXDrivetrain(const PulseMotor::DrivetrainConfig &cfg) {
    xDrivetrain_ = cfg;
}

void Gantry::setYDrivetrain(const PulseMotor::DrivetrainConfig &cfg) {
    yDrivetrain_ = cfg;
}

void Gantry::setThetaDrivetrain(const PulseMotor::DrivetrainConfig &cfg) {
    thetaDrivetrain_ = cfg;
}

void Gantry::setYAxisLimits(float minMm, float maxMm) {
    config_.limits.y_min = minMm;
    config_.limits.y_max = maxMm;
}

void Gantry::setThetaLimits(float minDeg, float maxDeg) {
    config_.limits.theta_min = minDeg;
    config_.limits.theta_max = maxDeg;
}

void Gantry::setJointLimits(float xMin, float xMax,
                            float yMin, float yMax,
                            float thetaMin, float thetaMax) {
    config_.limits.x_min = xMin;
    config_.limits.x_max = xMax;
    config_.limits.y_min = yMin;
    config_.limits.y_max = yMax;
    config_.limits.theta_min = thetaMin;
    config_.limits.theta_max = thetaMax;
}

void Gantry::setEndEffectorPin(int pin, bool activeHigh) {
    gripperPin_ = pin;
    endEffector_.configurePin(pin, activeHigh);
    if (initialized_) {
        endEffector_.begin();
    }
}

void Gantry::setSafeYHeight(float safeHeight_mm) {
    if (safeHeight_mm > 0.0f) {
        safeYHeight_mm_ = safeHeight_mm;
    }
}

// ============================================================================
// MOTION CONTROL
// ============================================================================

void Gantry::moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed_mm_per_s) {
    if (!initialized_ || !enabled_) {
        return;
    }

    if (speed_mm_per_s == 0) {
        speed_mm_per_s = 50;
    }

    targetY_ = y;
    targetTheta_ = theta;

    targetX_mm_ = (float)x;
    targetY_mm_ = (float)y;
    targetTheta_deg_ = (float)theta;
    speed_mm_per_s_ = speed_mm_per_s;
    acceleration_mm_per_s2_ = 0;
    deceleration_mm_per_s2_ = 0;

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

    if (axisX_.isAlarmActive()) {
        return GantryError::TIMEOUT;
    }

    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }

    targetX_mm_ = joint.x;
    targetY_mm_ = joint.y;
    targetTheta_deg_ = joint.theta;
    speed_mm_per_s_ = speed_mm_per_s;
    speed_deg_per_s_ = speed_deg_per_s;
    acceleration_mm_per_s2_ = acceleration_mm_per_s2;
    deceleration_mm_per_s2_ = deceleration_mm_per_s2;

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

    return moveTo(joint, speed_mm_per_s, speed_deg_per_s,
                  acceleration_mm_per_s2, deceleration_mm_per_s2);
}

bool Gantry::isBusy() const {
    if (!initialized_) {
        return false;
    }
    return motionState_ != MotionState::IDLE ||
           axisX_.isMotionActive() || axisX_.isHoming() ||
           isYAxisBusy() ||
           (thetaConfigured_ && axisTheta_.isMotionActive());
}

bool Gantry::isEnabled() const {
    return initialized_ && enabled_;
}

void Gantry::requestAbort() {
    abortRequested_ = true;
    if (initialized_) {
        stopAllMotion();
    }
}

bool Gantry::isAbortRequested() const {
    return abortRequested_;
}

void Gantry::update() {
    GANTRY_CHECK_INITIALIZED();

    if (motionState_ != MotionState::IDLE) {
        processSequentialMotion();
    }

    updateAxisPositions();
}

// ============================================================================
// HOMING AND CALIBRATION
// ============================================================================

void Gantry::home() {
    GANTRY_CHECK_INITIALIZED();
    GANTRY_CHECK_ENABLED();
    abortRequested_ = false;
    homingInProgress_ = false;

    if (axisX_.isAlarmActive()) {
        return;
    }

    if (!xMinSwitch_.isConfigured() || !xMaxSwitch_.isConfigured()) {
        return;
    }

    xMinSwitch_.update(true);
    xMaxSwitch_.update(true);

    if (xMinSwitch_.isActive()) {
        axisX_.stopMotion(0);
        axisX_.setPosition(0);
        return;
    }

    constexpr uint32_t kHomingStartPosition = 100000000;
    uint32_t speed = getHomingSpeed();
    axisX_.setPosition(kHomingStartPosition);
    if (!axisX_.moveToPosition(0, speed, speed, speed)) {
        stopAllMotion();
        return;
    }
    homingInProgress_ = true;
}

int Gantry::calibrate() {
    GANTRY_CHECK_INITIALIZED_RET(0);
    GANTRY_CHECK_ENABLED_RET(0);
    abortRequested_ = false;

    if (axisX_.isAlarmActive()) {
        return 0;
    }
    calibrationInProgress_ = true;

    if (!xMinSwitch_.isConfigured() || !xMaxSwitch_.isConfigured()) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    home();

    unsigned long start_ms = millis();
    while (homingInProgress_ && (millis() - start_ms) < CALIBRATION_TIMEOUT_MS) {
        if (abortRequested_) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
        xMinSwitch_.update();
        xMaxSwitch_.update();
        delay(10);
    }

    if (homingInProgress_ || !xMinSwitch_.isActive()) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    uint32_t speed = getHomingSpeed();
    constexpr uint32_t kCalibrationTarget = 1000000000UL;
    constexpr uint32_t kMinReleaseTimeoutMs = 3000;
    if (!axisX_.moveToPosition(kCalibrationTarget, speed, speed, speed)) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    start_ms = millis();
    bool minReleased = false;
    while (axisX_.isMotionActive() && (millis() - start_ms) < TRAVEL_MEASUREMENT_TIMEOUT_MS) {
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
            } else if ((millis() - start_ms) > kMinReleaseTimeoutMs) {
                ESP_LOGW(TAG,
                         "Calibration abort: MIN limit did not release within %lu ms",
                         (unsigned long)kMinReleaseTimeoutMs);
                stopAllMotion();
                calibrationInProgress_ = false;
                return 0;
            }
        }

        delay(10);
        if (axisX_.isAlarmActive()) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
    }

    xMaxSwitch_.update(true);
    if (!xMaxSwitch_.isActive()) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    axisLength_ = (int32_t)xPulsesToMm((int32_t)axisX_.getPosition());
    calibrationInProgress_ = false;
    return axisLength_;
}

// ============================================================================
// GRIPPER CONTROL
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
    if (!initialized_) {
        return 0;
    }
    if (axisX_.getConfig().enable_encoder_feedback) {
        return axisX_.getEncoderPosition();
    }
    return (int32_t)axisX_.getPosition();
}

int Gantry::getXEncoderRaw() const {
    if (!initialized_) {
        return 0;
    }
    return axisX_.getEncoderPosition();
}

int32_t Gantry::getXCommandedPulses() const {
    if (!initialized_) {
        return 0;
    }
    if (axisX_.isHoming() || homingInProgress_ || calibrationInProgress_) {
        return 0;
    }
    return (int32_t)axisX_.getPosition();
}

float Gantry::getXCommandedMm() const {
    return xPulsesToMm(getXCommandedPulses());
}

float Gantry::getXEncoderMm() const {
    return xPulsesToMm(getXEncoderRaw());
}

int Gantry::getCurrentY() const {
    return (int)getCurrentYPosition();
}

int Gantry::getCurrentTheta() const {
    if (thetaConfigured_) {
        const int32_t thetaCounts = thetaUseEncoder_
            ? axisTheta_.getEncoderPosition()
            : (int32_t)axisTheta_.getPosition();
        return (int)thetaPulsesToDeg(thetaCounts);
    }
    return currentTheta_;
}

// ============================================================================
// ALARM MONITORING
// ============================================================================

bool Gantry::isAlarmActive() const {
    if (!initialized_) {
        return false;
    }
    const bool xAlarm = axisX_.getStatus().alarm_active;
    const bool yAlarm = yConfigured_ ? axisY_.getStatus().alarm_active : false;
    const bool thetaAlarm = thetaConfigured_ ? axisTheta_.getStatus().alarm_active : false;
    return xAlarm || yAlarm || thetaAlarm;
}

bool Gantry::clearAlarm() {
    if (!initialized_) {
        return false;
    }
    bool ok = axisX_.clearAlarm();
    if (yConfigured_) {
        ok = axisY_.clearAlarm() || ok;
    }
    if (thetaConfigured_) {
        ok = axisTheta_.clearAlarm() || ok;
    }
    return ok;
}

void Gantry::setHomingSpeed(uint32_t speed_pps) {
    if (initialized_) {
        PulseMotor::DriverConfig config = axisX_.getConfig();
        config.homing_speed_pps = speed_pps;
        axisX_.setConfig(config);
    }
}

// ============================================================================
// ENHANCED KINEMATICS API
// ============================================================================

EndEffectorPose Gantry::forwardKinematics(const JointConfig& joint) const {
    return Kinematics::forward(joint, kinematicParams_);
}

JointConfig Gantry::inverseKinematics(const EndEffectorPose& pose) const {
    return Kinematics::inverse(pose, kinematicParams_);
}

JointConfig Gantry::getCurrentJointConfig() const {
    JointConfig joint;
    joint.x = currentX_mm_;
    joint.y = getCurrentYPosition();
    if (thetaConfigured_) {
        const int32_t thetaCounts = thetaUseEncoder_
            ? axisTheta_.getEncoderPosition()
            : (int32_t)axisTheta_.getPosition();
        joint.theta = thetaPulsesToDeg(thetaCounts);
    } else {
        joint.theta = (float)currentTheta_;
    }
    return joint;
}

JointConfig Gantry::getTargetJointConfig() const {
    JointConfig joint;
    joint.x = xPulsesToMm(getXEncoder());
    joint.y = targetY_mm_;
    joint.theta = (float)targetTheta_;
    return joint;
}

EndEffectorPose Gantry::getCurrentEndEffectorPose() const {
    return forwardKinematics(getCurrentJointConfig());
}

EndEffectorPose Gantry::getTargetEndEffectorPose() const {
    return forwardKinematics(getTargetJointConfig());
}

// ============================================================================
// UNIT CONVERSION HELPERS
// ============================================================================

double Gantry::xPulsesPerMm() const {
    const double v = PulseMotor::pulsesPerMm(xDrivetrain_);
    return v > 0.0 ? v : 1.0;
}

double Gantry::yPulsesPerMm() const {
    const double v = PulseMotor::pulsesPerMm(yDrivetrain_);
    return v > 0.0 ? v : 1.0;
}

double Gantry::thetaPulsesPerDeg() const {
    const double v = PulseMotor::pulsesPerDeg(thetaDrivetrain_);
    return v > 0.0 ? v : 1.0;
}

float Gantry::xPulsesToMm(int32_t pulses) const {
    return (float)((double)pulses / xPulsesPerMm());
}

int32_t Gantry::xMmToPulses(float mm) const {
    return (int32_t)((double)mm * xPulsesPerMm());
}

float Gantry::yPulsesToMm(int32_t pulses) const {
    return (float)((double)pulses / yPulsesPerMm());
}

int32_t Gantry::yMmToPulses(float mm) const {
    return (int32_t)((double)mm * yPulsesPerMm());
}

float Gantry::thetaPulsesToDeg(int32_t pulses) const {
    return (float)((double)pulses / thetaPulsesPerDeg());
}

int32_t Gantry::thetaDegToPulses(float deg) const {
    return (int32_t)((double)deg * thetaPulsesPerDeg());
}

// ============================================================================
// HELPERS
// ============================================================================

float Gantry::getCurrentYPosition() const {
    if (yConfigured_) {
        return getCurrentYFromServoMm();
    }
    return (float)currentY_;
}

uint32_t Gantry::getHomingSpeed() const {
    uint32_t speed = axisX_.getConfig().homing_speed_pps;
    return (speed > 0) ? speed : DEFAULT_HOMING_SPEED_PPS;
}

bool Gantry::moveYAxisTo(float targetY, float speed, float accel, float decel) {
    if (!yConfigured_) {
        currentY_ = (int32_t)targetY;
        return true;
    }
    const double yPpm = yPulsesPerMm();
    const int32_t currentYCounts = yUseEncoder_
        ? axisY_.getEncoderPosition()
        : (int32_t)axisY_.getPosition();
    const int32_t targetYCounts = (int32_t)((double)targetY * yPpm);
    const int32_t deltaY = targetYCounts - currentYCounts;
    if (deltaY == 0) {
        return true;
    }
    uint32_t speed_pps = (uint32_t)((double)speed * yPpm);
    uint32_t accel_pps = (accel > 0.0f) ? (uint32_t)((double)accel * yPpm) : (speed_pps / 2);
    uint32_t decel_pps = (decel > 0.0f) ? (uint32_t)((double)decel * yPpm) : (speed_pps / 2);
    if (speed_pps == 0) speed_pps = 1;
    if (accel_pps == 0) accel_pps = 1;
    if (decel_pps == 0) decel_pps = 1;
    return axisY_.moveRelative(deltaY, speed_pps, accel_pps, decel_pps);
}

bool Gantry::isYAxisConfigured() const {
    return yConfigured_;
}

bool Gantry::isYAxisBusy() const {
    return yConfigured_ && axisY_.isMotionActive();
}

float Gantry::getCurrentYFromServoMm() const {
    const int32_t yCounts = yUseEncoder_
        ? axisY_.getEncoderPosition()
        : (int32_t)axisY_.getPosition();
    return yPulsesToMm(yCounts);
}

void Gantry::updateAxisPositions() {
    xMinSwitch_.update();
    xMaxSwitch_.update();

    uint32_t currentXCounts = axisX_.getPosition();
    if (axisX_.isMotionActive()) {
        bool movingTowardMax = currentXCounts > lastXPositionCounts_;
        bool movingTowardMin = currentXCounts < lastXPositionCounts_;

        if (movingTowardMax && xMaxSwitch_.isConfigured() && xMaxSwitch_.isActive()) {
            axisX_.stopMotion(0);
            homingInProgress_ = false;
            calibrationInProgress_ = false;
        } else if (movingTowardMin && xMinSwitch_.isConfigured() && xMinSwitch_.isActive()) {
            axisX_.stopMotion(0);
            axisX_.setPosition(0);
            homingInProgress_ = false;
        }
    } else if (homingInProgress_) {
        if (xMinSwitch_.isActive()) {
            axisX_.setPosition(0);
        }
        homingInProgress_ = false;
    }
    lastXPositionCounts_ = currentXCounts;

    currentX_mm_ = xPulsesToMm(getXEncoder());

    if (yConfigured_) {
        currentY_ = (int32_t)getCurrentYFromServoMm();
    }
    if (thetaConfigured_) {
        const int32_t thetaCounts = thetaUseEncoder_
            ? axisTheta_.getEncoderPosition()
            : (int32_t)axisTheta_.getPosition();
        currentTheta_ = (int32_t)thetaPulsesToDeg(thetaCounts);
    }
}

void Gantry::stopAllMotion() {
    axisX_.stopMotion(0);
    if (yConfigured_) {
        axisY_.stopMotion(0);
    }
    if (thetaConfigured_) {
        axisTheta_.stopMotion(0);
    }
    homingInProgress_ = false;
    calibrationInProgress_ = false;
    motionState_ = MotionState::IDLE;
}

// ============================================================================
// SEQUENTIAL MOTION IMPLEMENTATION
// ============================================================================

void Gantry::startSequentialMotion() {
    if (!enabled_ || motionState_ != MotionState::IDLE) {
        return;
    }

    float currentY = getCurrentYPosition();

    gripperTargetState_ = (targetY_mm_ < currentY);

    if (currentY < safeYHeight_mm_) {
        motionState_ = MotionState::Y_RETRACTING;
        if (!moveYAxisTo(safeYHeight_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_, (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        if (!isYAxisConfigured()) {
            motionState_ = MotionState::X_MOVING;
            startXAxisMotion();
        }
    } else if (targetY_mm_ < currentY) {
        motionState_ = MotionState::Y_DESCENDING;
        if (!moveYAxisTo(targetY_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_, (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        if (!isYAxisConfigured()) {
            motionState_ = MotionState::GRIPPER_ACTUATING;
            grip(gripperTargetState_);
            gripperActuateStart_ms_ = millis();
        }
    } else {
        motionState_ = MotionState::X_MOVING;
        startXAxisMotion();
    }

    // Theta moves independently
    if (thetaConfigured_) {
        const int32_t currentThetaCounts = thetaUseEncoder_
            ? axisTheta_.getEncoderPosition()
            : (int32_t)axisTheta_.getPosition();
        const int32_t targetThetaCounts = thetaDegToPulses(targetTheta_deg_);
        const int32_t deltaTheta = targetThetaCounts - currentThetaCounts;
        const double thetaPpd = thetaPulsesPerDeg();
        const uint32_t thetaSpeedPps =
            (uint32_t)((double)speed_deg_per_s_ * thetaPpd);
        const uint32_t thetaAccelPps = (thetaSpeedPps > 1) ? thetaSpeedPps / 2 : 1;
        const uint32_t thetaDecelPps = (thetaSpeedPps > 1) ? thetaSpeedPps / 2 : 1;
        if (deltaTheta != 0 &&
            !axisTheta_.moveRelative(deltaTheta, thetaSpeedPps > 0 ? thetaSpeedPps : 1,
                                     thetaAccelPps, thetaDecelPps)) {
            ESP_LOGE(TAG, "[THETA] moveRelative failed - aborting sequence");
            stopAllMotion();
            return;
        }
        currentTheta_ = (int32_t)thetaPulsesToDeg(thetaUseEncoder_
            ? axisTheta_.getEncoderPosition()
            : (int32_t)axisTheta_.getPosition());
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
        case MotionState::Y_DESCENDING:
            if (!isYAxisConfigured() || !isYAxisBusy()) {
                motionState_ = MotionState::GRIPPER_ACTUATING;
                grip(gripperTargetState_);
                gripperActuateStart_ms_ = millis();
            }
            break;

        case MotionState::GRIPPER_ACTUATING: {
            const uint32_t gripperDelayMs =
                gripperTargetState_ ? GANTRY_GRIPPER_CLOSE_TIME_MS
                                    : GANTRY_GRIPPER_OPEN_TIME_MS;
            if (millis() - gripperActuateStart_ms_ >= gripperDelayMs) {
                motionState_ = MotionState::Y_RETRACTING;
                if (!moveYAxisTo(safeYHeight_mm_, (float)speed_mm_per_s_,
                                 (float)acceleration_mm_per_s2_, (float)deceleration_mm_per_s2_)) {
                    stopAllMotion();
                    return;
                }
                if (!isYAxisConfigured()) {
                    motionState_ = MotionState::X_MOVING;
                    startXAxisMotion();
                }
            }
            break;
        }

        case MotionState::Y_RETRACTING:
            if (!isYAxisConfigured() || !isYAxisBusy()) {
                motionState_ = MotionState::X_MOVING;
                startXAxisMotion();
            }
            break;

        case MotionState::X_MOVING:
            if (!axisX_.isMotionActive()) {
                motionState_ = MotionState::IDLE;
            }
            break;

        case MotionState::THETA_MOVING:
            if (!thetaConfigured_ || !axisTheta_.isMotionActive()) {
                motionState_ = MotionState::IDLE;
            }
            break;

        case MotionState::IDLE:
        default:
            break;
    }
}

void Gantry::startXAxisMotion() {
    const int32_t driverReportedPulses = (int32_t)axisX_.getPosition();
    int32_t currentX_pulses = driverReportedPulses;
    const int32_t trackedPulses = xMmToPulses(currentX_mm_);
    const int32_t encoderPulses = axisX_.getEncoderPosition();
    if (axisX_.getConfig().enable_encoder_feedback) {
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
    axisX_.setPosition((uint32_t)currentX_pulses);
    int32_t targetX_pulses = xMmToPulses(targetX_mm_);
    int32_t deltaX = targetX_pulses - currentX_pulses;

    ESP_LOGI(TAG,
             "[X_MOVE] current=%ld tracked=%ld driver=%ld encoder=%ld target=%ld delta=%ld speed=%lu accel=%lu decel=%lu",
             (long)currentX_pulses, (long)trackedPulses, (long)driverReportedPulses,
             (long)encoderPulses,
             (long)targetX_pulses, (long)deltaX,
             (unsigned long)speed_mm_per_s_, (unsigned long)acceleration_mm_per_s2_,
             (unsigned long)deceleration_mm_per_s2_);

    if (deltaX == 0) {
        motionState_ = MotionState::IDLE;
        return;
    }

    const double xPpm = xPulsesPerMm();
    uint32_t speed_pps = (uint32_t)((double)speed_mm_per_s_ * xPpm);

    uint32_t accel_pps = 0;
    uint32_t decel_pps = 0;
    if (acceleration_mm_per_s2_ > 0) {
        accel_pps = (uint32_t)((double)acceleration_mm_per_s2_ * xPpm);
    }
    if (deceleration_mm_per_s2_ > 0) {
        decel_pps = (uint32_t)((double)deceleration_mm_per_s2_ * xPpm);
    }

    if (accel_pps == 0) accel_pps = speed_pps / 2;
    if (decel_pps == 0) decel_pps = speed_pps / 2;

    if (!axisX_.moveRelative(deltaX, speed_pps, accel_pps, decel_pps)) {
        ESP_LOGE(TAG,
                 "[X_MOVE] moveRelative rejected (delta=%ld, speed_pps=%lu, accel_pps=%lu, decel_pps=%lu)",
                 (long)deltaX, (unsigned long)speed_pps, (unsigned long)accel_pps,
                 (unsigned long)decel_pps);
        stopAllMotion();
    }
}

} // namespace Gantry
