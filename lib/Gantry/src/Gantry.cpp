/**
 * @file Gantry.cpp
 * @brief Implementation of Gantry class
 * @version 1.0.0
 */

#include "Gantry.h"
#include <cmath>
#include "esp_log.h"

using namespace Gantry::Constants;

namespace Gantry {
static const char *TAG = "Gantry";

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Gantry::Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin)
    : axisX_(xConfig), gripperPin_(gripperPin), xMinPin_(-1), xMaxPin_(-1),
      initialized_(false), enabled_(false), abortRequested_(false),
      homingInProgress_(false), calibrationInProgress_(false), gripperActive_(false),
      currentX_mm_(0.0f), currentY_(0), currentTheta_(0), targetY_(0), targetTheta_(0),
      axisLength_(0), config_(), kinematicParams_(),
      stepsPerRev_(DEFAULT_STEPS_PER_REV),
      motionState_(MotionState::IDLE),
      targetX_mm_(0.0f), targetY_mm_(0.0f), targetTheta_deg_(0.0f),
      safeYHeight_mm_(DEFAULT_SAFE_Y_HEIGHT_MM),
      speed_mm_per_s_(DEFAULT_SPEED_MM_PER_S), speed_deg_per_s_(DEFAULT_SPEED_DEG_PER_S),
      acceleration_mm_per_s2_(0), deceleration_mm_per_s2_(0),
      gripperTargetState_(false), gripperActuateStart_ms_(0), lastXPositionCounts_(0) {
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool Gantry::begin() {
    if (initialized_) {
        return true;
    }
    
    // Initialize end-effector (gripper) pin
    if (gripperPin_ >= 0) {
        endEffector_.configurePin(gripperPin_, true);
        endEffector_.begin();
        gripperActive_ = false;
    }
    
    // Limits are handled by reusable GantryLimitSwitch objects in this library.
    // Keep driver internal limit handling disabled.
    BergerdaServo::DriverConfig& xConfig =
        const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
    xConfig.limit_min_pin = -1;
    xConfig.limit_max_pin = -1;

    xMinSwitch_.begin();
    xMaxSwitch_.begin();
    
    // Initialize X-axis servo driver (will configure limit switches if pins are set)
    if (!axisX_.initialize()) {
        return false;
    }
    
    // Initialize Y-axis stepper if configured
    if (axisY_.isConfigured()) {
        axisY_.begin();
    }

    // Initialize theta servo if configured
    if (axisTheta_.isConfigured()) {
        axisTheta_.begin();
    }

    initialized_ = true;
    return true;
}

// ============================================================================
// ENABLE/DISABLE
// ============================================================================

void Gantry::enable() {
    GANTRY_CHECK_INITIALIZED();
    // Consider already-enabled driver as success.
    bool xEnabled = axisX_.isEnabled() || axisX_.enable();
    if (axisY_.isConfigured()) {
        if (xEnabled) {
            axisY_.enable();
        } else {
            axisY_.disable();
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
    if (axisY_.isConfigured()) {
        axisY_.disable();
    }
    enabled_ = false;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void Gantry::setLimitPins(int xMinPin, int xMaxPin) {
    xMinPin_ = xMinPin;
    xMaxPin_ = xMaxPin;
    xMinSwitch_.configure(xMinPin_, true, true, 3);
    xMaxSwitch_.configure(xMaxPin_, true, true, 3);
}

void Gantry::setYAxisPins(int stepPin, int dirPin, int enablePin,
                          bool invertDir, bool enableActiveLow) {
    axisY_.configurePins(stepPin, dirPin, enablePin, invertDir, enableActiveLow);
}

void Gantry::setYAxisStepsPerMm(float stepsPerMm) {
    axisY_.setStepsPerMm(stepsPerMm);
}

void Gantry::setYAxisLimits(float minMm, float maxMm) {
    axisY_.setLimits(minMm, maxMm);
}

void Gantry::setYAxisMotionLimits(float maxSpeedMmPerS,
                                  float accelMmPerS2,
                                  float decelMmPerS2) {
    axisY_.setMotionLimits(maxSpeedMmPerS, accelMmPerS2, decelMmPerS2);
}

void Gantry::setThetaServo(int pwmPin, int pwmChannel) {
    axisTheta_.configurePin(pwmPin, pwmChannel);
}

void Gantry::setThetaLimits(float minDeg, float maxDeg) {
    axisTheta_.setAngleRange(minDeg, maxDeg);
}

void Gantry::setThetaPulseRange(uint16_t minPulseUs, uint16_t maxPulseUs) {
    axisTheta_.setPulseRange(minPulseUs, maxPulseUs);
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

void Gantry::moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed) {
    if (!initialized_ || !enabled_) {
        return;
    }
    
    // Validate inputs
    if (speed == 0) {
        speed = 5000; // Default speed
    }
    
    // Store target positions for Y and Theta
    targetY_ = y;
    targetTheta_ = theta;
    if (axisY_.isConfigured()) {
        if (!axisY_.moveToMm((float)y)) {
            stopAllMotion();
            return;
        }
    } else {
        currentY_ = y;  // Simulated - instant update
    }
    if (axisTheta_.isConfigured()) {
        if (!axisTheta_.moveToDeg((float)theta)) {
            stopAllMotion();
            return;
        }
        currentTheta_ = (int32_t)axisTheta_.getCurrentDeg();
    } else {
        currentTheta_ = theta;  // Simulated - instant update
    }
    
    // Store targets for sequential motion
    targetX_mm_ = (float)x;
    targetY_mm_ = (float)y;
    targetTheta_deg_ = (float)theta;
    speed_mm_per_s_ = (uint32_t)(speed / getPulsesPerMm());  // Convert pps to mm/s
    acceleration_mm_per_s2_ = 0;
    deceleration_mm_per_s2_ = 0;
    
    // Start sequential motion sequence
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
    
    // Store targets for sequential motion
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
    
    return moveTo(joint, speed_mm_per_s, speed_deg_per_s, acceleration_mm_per_s2, deceleration_mm_per_s2);
}

bool Gantry::isBusy() const {
    if (!initialized_) {
        return false;
    }
    return motionState_ != MotionState::IDLE ||
           axisX_.isMotionActive() || axisX_.isHoming() ||
           (axisY_.isConfigured() && axisY_.isBusy());
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

    // Do not start homing while driver alarm input is active.
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
        // If homing command fails to start, force-stop any ongoing X motion.
        stopAllMotion();
        return;
    }
    homingInProgress_ = true;
}

int Gantry::calibrate() {
    GANTRY_CHECK_INITIALIZED_RET(0);
    GANTRY_CHECK_ENABLED_RET(0);
    abortRequested_ = false;

    // Do not start calibration while driver alarm input is active.
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
    
    // Wait for homing to complete
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
    
    // Measure travel distance from MIN to MAX using Gantry limit switches.
    uint32_t speed = getHomingSpeed();
    constexpr uint32_t kCalibrationTarget = 1000000000UL;
    // Some drives/mechanics can take >1s to physically leave MIN after command start.
    // Keep this guard, but relax it to avoid false first-attempt calibration failures.
    constexpr uint32_t kMinReleaseTimeoutMs = 3000;
    if (!axisX_.moveToPosition(kCalibrationTarget, speed, speed, speed)) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }
    
    // Wait for max switch hit.
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

        // Starting from MIN, the MIN switch must release quickly while moving to MAX.
        // If it stays active, motion is not progressing in the expected direction.
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
    
    axisLength_ = pulsesToMm((int32_t)axisX_.getPosition());
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

    // Use encoder only when explicitly enabled; otherwise report driver commanded
    // position so runtime telemetry remains meaningful in open-loop test setups.
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
    // Homing/calibration can seed very large synthetic counters to enforce
    // direction and travel. Those internal references are not physical
    // machine coordinates and must not be exposed as live commanded position.
    if (axisX_.isHoming() || homingInProgress_ || calibrationInProgress_) {
        return 0;
    }
    return (int32_t)axisX_.getPosition();
}

float Gantry::getXCommandedMm() const {
    // Commanded position track converted with current pulses/mm configuration.
    return pulsesToMm(getXCommandedPulses());
}

float Gantry::getXEncoderMm() const {
    // Raw encoder feedback converted with the same pulses/mm model.
    return pulsesToMm(getXEncoderRaw());
}

int Gantry::getCurrentY() const {
    return (int)getCurrentYPosition();
}

int Gantry::getCurrentTheta() const {
    if (axisTheta_.isConfigured()) {
        return (int)axisTheta_.getCurrentDeg();
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
    // Use driver-maintained alarm status instead of raw GPIO reads.
    return axisX_.getStatus().alarm_active;
}

bool Gantry::clearAlarm() {
    if (!initialized_) {
        return false;
    }
    return axisX_.clearAlarm();
}

void Gantry::setHomingSpeed(uint32_t speed_pps) {
    if (initialized_) {
        BergerdaServo::DriverConfig& config = 
            const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
        config.homing_speed_pps = speed_pps;
    }
}

// ============================================================================
// ENHANCED KINEMATICS API (Phase 1.2)
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
    joint.theta = axisTheta_.isConfigured() ? axisTheta_.getCurrentDeg() : (float)currentTheta_;
    return joint;
}

JointConfig Gantry::getTargetJointConfig() const {
    JointConfig joint;
    joint.x = pulsesToMm(getXEncoder());  // Current X (target tracking not implemented)
    joint.y = axisY_.isConfigured() ? axisY_.getTargetMm() : (float)targetY_;
    joint.theta = (float)targetTheta_;
    return joint;
}

EndEffectorPose Gantry::getCurrentEndEffectorPose() const {
    JointConfig joint = getCurrentJointConfig();
    return forwardKinematics(joint);
}

EndEffectorPose Gantry::getTargetEndEffectorPose() const {
    JointConfig joint = getTargetJointConfig();
    return forwardKinematics(joint);
}

// ============================================================================
// HELPER METHODS
// ============================================================================

void Gantry::setStepsPerRevolution(float steps_per_rev) {
    if (steps_per_rev > 0) {
        stepsPerRev_ = steps_per_rev;
    }
}

float Gantry::getPulsesPerMm() const {
    const float pitch = kinematicParams_.x_axis_ball_screw_pitch_mm;
    if (pitch <= 0) return DEFAULT_PULSES_PER_MM;
    return stepsPerRev_ / pitch;
}

float Gantry::pulsesToMm(int32_t pulses) const {
    // Convert motor pulses to mm
    // Formula: mm = pulses / pulses_per_mm
    // For ball screw: mm = pulses * (pitch_mm / steps_per_rev)
    // Example: 150 pulses * (40mm / 6000) = 1mm
    float pulsesPerMm = getPulsesPerMm();
    if (pulsesPerMm <= 0) return 0.0f;
    return (float)pulses / pulsesPerMm;
}

int32_t Gantry::mmToPulses(float mm) const {
    return (int32_t)(mm * getPulsesPerMm());
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

float Gantry::getCurrentYPosition() const {
    return axisY_.isConfigured() ? axisY_.getCurrentMm() : (float)currentY_;
}

uint32_t Gantry::getHomingSpeed() const {
    uint32_t speed = axisX_.getConfig().homing_speed_pps;
    return (speed > 0) ? speed : DEFAULT_HOMING_SPEED_PPS;
}

bool Gantry::moveYAxisTo(float targetY, float speed, float accel, float decel) {
    if (axisY_.isConfigured()) {
        return axisY_.moveToMm(targetY, speed, accel, decel);
    } else {
        currentY_ = (int32_t)targetY;
        return true;
    }
}

void Gantry::updateAxisPositions() {
    xMinSwitch_.update();
    xMaxSwitch_.update();

    uint32_t currentXCounts = axisX_.getPosition();
    if (axisX_.isMotionActive()) {
        // Only treat as directional travel when encoder counts actually changed.
        // Equality is an indeterminate startup/settling state and must not trip limits.
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
        // Clear homing flag once motion stops.
        if (xMinSwitch_.isActive()) {
            axisX_.setPosition(0);
        }
        homingInProgress_ = false;
    }
    lastXPositionCounts_ = currentXCounts;

    // Refresh X joint position every update tick from encoder feedback.
    currentX_mm_ = pulsesToMm(getXEncoder());

    if (axisY_.isConfigured()) {
        axisY_.update();
        currentY_ = (int32_t)axisY_.getCurrentMm();
    }
    if (axisTheta_.isConfigured()) {
        currentTheta_ = (int32_t)axisTheta_.getCurrentDeg();
    }
}

void Gantry::stopAllMotion() {
    axisX_.stopMotion(0);
    if (axisY_.isConfigured()) {
        axisY_.stop();
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
    
    // Determine gripper action: descending = picking (close), ascending = placing (open)
    gripperTargetState_ = (targetY_mm_ < currentY);
    
    // Determine motion sequence based on current and target Y positions
    if (currentY < safeYHeight_mm_) {
        // Retract Y to safe height first
        motionState_ = MotionState::Y_RETRACTING;
        if (!moveYAxisTo(safeYHeight_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_, (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        if (!axisY_.isConfigured()) {
            motionState_ = MotionState::X_MOVING;
            startXAxisMotion();
        }
    } else if (targetY_mm_ < currentY) {
        // Y needs to descend to target
        motionState_ = MotionState::Y_DESCENDING;
        if (!moveYAxisTo(targetY_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_, (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        if (!axisY_.isConfigured()) {
            motionState_ = MotionState::GRIPPER_ACTUATING;
            grip(gripperTargetState_);
            gripperActuateStart_ms_ = millis();
        }
    } else {
        // Y is at safe height, start X movement
        motionState_ = MotionState::X_MOVING;
        startXAxisMotion();
    }
    
    // Theta moves independently
    if (axisTheta_.isConfigured()) {
        if (!axisTheta_.moveToDeg(targetTheta_deg_)) {
            stopAllMotion();
            return;
        }
        currentTheta_ = (int32_t)axisTheta_.getCurrentDeg();
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
            // Wait for Y-axis to reach target
            if (!axisY_.isConfigured() || !axisY_.isBusy()) {
                // Y-axis reached target, actuate gripper
                motionState_ = MotionState::GRIPPER_ACTUATING;
                grip(gripperTargetState_);
                gripperActuateStart_ms_ = millis();
            }
            break;
            
        case MotionState::GRIPPER_ACTUATING:
            if (millis() - gripperActuateStart_ms_ >= Constants::GRIPPER_ACTUATE_TIME_MS) {
                motionState_ = MotionState::Y_RETRACTING;
                if (!moveYAxisTo(safeYHeight_mm_, (float)speed_mm_per_s_,
                                 (float)acceleration_mm_per_s2_, (float)deceleration_mm_per_s2_)) {
                    stopAllMotion();
                    return;
                }
                if (!axisY_.isConfigured()) {
                    motionState_ = MotionState::X_MOVING;
                    startXAxisMotion();
                }
            }
            break;
            
        case MotionState::Y_RETRACTING:
            if (!axisY_.isConfigured() || !axisY_.isBusy()) {
                float currentY = getCurrentYPosition();
                if (targetY_mm_ < currentY) {
                    motionState_ = MotionState::Y_DESCENDING;
                    if (!moveYAxisTo(targetY_mm_, (float)speed_mm_per_s_,
                                     (float)acceleration_mm_per_s2_, (float)deceleration_mm_per_s2_)) {
                        stopAllMotion();
                        return;
                    }
                    if (!axisY_.isConfigured()) {
                        motionState_ = MotionState::GRIPPER_ACTUATING;
                        grip(gripperTargetState_);
                        gripperActuateStart_ms_ = millis();
                    }
                } else {
                    motionState_ = MotionState::X_MOVING;
                    startXAxisMotion();
                }
            }
            break;
            
        case MotionState::X_MOVING:
            // Wait for X-axis to reach target
            if (!axisX_.isMotionActive()) {
                // All motion complete
                motionState_ = MotionState::IDLE;
            }
            break;
            
        case MotionState::THETA_MOVING:
            // Theta moves independently, check if complete
            if (!axisTheta_.isConfigured()) {
                motionState_ = MotionState::IDLE;
            }
            break;
            
        case MotionState::IDLE:
        default:
            break;
    }
}

void Gantry::startXAxisMotion() {
    // Calculate X-axis movement in pulses
    int32_t currentX_pulses = axisX_.getPosition();
    int32_t targetX_pulses = mmToPulses(targetX_mm_);
    int32_t deltaX = targetX_pulses - currentX_pulses;
    
    if (deltaX == 0) {
        // Already at target X
        motionState_ = MotionState::IDLE;
        return;
    }
    
    // Convert speed: mm/s to pulses/s for X-axis
    float pulsesPerMm = getPulsesPerMm();
    uint32_t speed_pps = (uint32_t)(speed_mm_per_s_ * pulsesPerMm);
    
    // Convert acceleration/deceleration if provided
    uint32_t accel_pps = 0;
    uint32_t decel_pps = 0;
    if (acceleration_mm_per_s2_ > 0) {
        accel_pps = (uint32_t)(acceleration_mm_per_s2_ * pulsesPerMm);
    }
    if (deceleration_mm_per_s2_ > 0) {
        decel_pps = (uint32_t)(deceleration_mm_per_s2_ * pulsesPerMm);
    }
    
    // Use default if not specified
    if (accel_pps == 0) accel_pps = speed_pps / 2;
    if (decel_pps == 0) decel_pps = speed_pps / 2;
    
    // Move X-axis; if command fails, force-stop to avoid any stale motion profile.
    if (!axisX_.moveRelative(deltaX, speed_pps, accel_pps, decel_pps)) {
        stopAllMotion();
    }
}

} // namespace Gantry
