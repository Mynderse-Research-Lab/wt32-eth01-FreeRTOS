/**
 * @file GantryMotion.cpp
 * @brief Gantry class: motion commands, safe-Z retract, preload and the gripper.
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

// ============================================================================
// MOTION CONTROL
// ============================================================================

void Gantry::moveTo(int32_t x, int32_t z, int32_t theta, uint32_t speed) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !enabled_ || !workspaceCalibrated_) {
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
    acceleration_deg_per_s2_ = 0;
    deceleration_deg_per_s2_ = 0;
    jointDirectMove_ = true;
    thetaStartFrac_ = -1.0f;
    thetaFullSpeed_ = false;

    startSequentialMotion();
}

GantryError Gantry::moveTo(const JointConfig& joint,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2,
                           uint32_t acceleration_deg_per_s2,
                           uint32_t deceleration_deg_per_s2,
                           float theta_start_frac,
                           bool theta_full_speed) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    if (!workspaceCalibrated_) {
        return GantryError::NOT_CALIBRATED;
    }
    GANTRY_CHECK_BUSY_RET(GantryError::ALREADY_MOVING);

    if (axisX_ && axisX_->isAlarmActive()) {
        return GantryError::TIMEOUT;
    }

    JointConfig goal = joint;
    if (theta_start_frac >= 0.0f) {
        const float cur = axisTheta_ ? axisTheta_->getCurrentDeg()
                                     : static_cast<float>(currentTheta_);
        goal.theta = Path::commandThetaDeg(cur, joint.theta);
    }
    if (!Kinematics::validate(goal, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }

    targetX_mm_       = goal.x;
    targetZ_mm_       = goal.z;
    targetTheta_deg_  = goal.theta;
    speed_mm_per_s_   = speed_mm_per_s;
    speed_deg_per_s_  = speed_deg_per_s;
    acceleration_mm_per_s2_ = acceleration_mm_per_s2;
    deceleration_mm_per_s2_ = deceleration_mm_per_s2;
    acceleration_deg_per_s2_ = acceleration_deg_per_s2;
    deceleration_deg_per_s2_ = deceleration_deg_per_s2;
    jointDirectMove_ = true;
    thetaStartFrac_ = theta_start_frac;
    thetaFullSpeed_ = theta_full_speed;

    startSequentialMotion();
    return GantryError::OK;
}

GantryError Gantry::preloadJointMove(const JointConfig& joint,
                                     uint32_t speed_mm_per_s,
                                     uint32_t acceleration_mm_per_s2,
                                     uint32_t deceleration_mm_per_s2) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    if (!workspaceCalibrated_) {
        return GantryError::NOT_CALIBRATED;
    }
    GANTRY_CHECK_BUSY_RET(GantryError::ALREADY_MOVING);

    if (axisX_ && axisX_->isAlarmActive()) {
        return GantryError::TIMEOUT;
    }
    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }

    const float x = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float z = axisZ_ ? axisZ_->getCurrentMm() : static_cast<float>(currentZ_);
    const float th = axisTheta_ ? axisTheta_->getCurrentDeg() : static_cast<float>(currentTheta_);
    if (std::fabs(joint.x - x) > 0.5f || std::fabs(joint.theta - th) > 0.5f) {
        return GantryError::INVALID_PARAMETER;
    }
    if (std::fabs(joint.z - z) <= Path::kAxisEpsMm) {
        zStartHeld_ = false;
        return GantryError::OK;
    }

    targetX_mm_ = joint.x;
    targetZ_mm_ = joint.z;
    targetTheta_deg_ = joint.theta;
    speed_mm_per_s_ = speed_mm_per_s;
    speed_deg_per_s_ = 0;
    acceleration_mm_per_s2_ = acceleration_mm_per_s2;
    deceleration_mm_per_s2_ = deceleration_mm_per_s2;
    acceleration_deg_per_s2_ = 0;
    deceleration_deg_per_s2_ = 0;
    jointDirectMove_ = true;
    thetaStartFrac_ = -1.0f;
    thetaFullSpeed_ = false;
    holdZStart_ = true;
    zStartHeld_ = false;

    startSequentialMotion();
    if (holdZStart_) {
        holdZStart_ = false;
        return GantryError::INVALID_PARAMETER;
    }
    if (!zStartHeld_ && !(axisZ_ && axisZ_->isBusy())) {
        return GantryError::TIMEOUT;
    }
    return GantryError::OK;
}

bool Gantry::releasePreloadedMotion() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!axisZ_) {
        return false;
    }
    const bool ok = axisZ_->releaseStartMotion();
    if (ok) {
        zStartHeld_ = false;
    }
    return ok;
}

bool Gantry::isPreloadHeld() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisZ_ && axisZ_->isStartHeld();
}

GantryError Gantry::retractToSafeZ(uint32_t speed_mm_per_s,
                                   uint32_t acceleration_mm_per_s2,
                                   uint32_t deceleration_mm_per_s2) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    if (!axisZ_) {
        return GantryError::NOT_INITIALIZED;
    }
    if (axisZ_->isAlarmActive() ||
        axisZ_->getDrivePositionRef() == DrivePositionRef::kLost) {
        return GantryError::TIMEOUT;
    }
    if (motionState_ != MotionState::IDLE || axisZ_->isBusy() ||
        (axisX_ && axisX_->isMotionActive()) ||
        (axisTheta_ && axisTheta_->isMotionActive())) {
        stopAllMotion();
    }

    const float x = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float z = axisZ_->getCurrentMm();
    const float ceiling = traverseClearanceZMm();
    if (Path::zInTraverseBand(z, ceiling)) {
        return GantryError::OK;
    }

    targetX_mm_ = x;
    targetZ_mm_ = ceiling;
    targetTheta_deg_ = axisTheta_ ? axisTheta_->getCurrentDeg() : static_cast<float>(currentTheta_);
    speed_mm_per_s_ = static_cast<uint32_t>(Path::zCappedPathSpeed(
        static_cast<float>(speed_mm_per_s), 0.0f, ceiling - z,
        runtime_.z_speed_cap_mm_s));
    const float z_accel = Path::zCappedPathAccel(
        static_cast<float>(acceleration_mm_per_s2), 0.0f, ceiling - z,
        runtime_.z_accel_cap_mm_s2);
    const float z_decel = Path::zCappedPathAccel(
        static_cast<float>(deceleration_mm_per_s2), 0.0f, ceiling - z,
        runtime_.z_decel_cap_mm_s2);
    speed_deg_per_s_ = 0;
    acceleration_mm_per_s2_ = static_cast<uint32_t>(z_accel);
    deceleration_mm_per_s2_ = static_cast<uint32_t>(z_decel);
    acceleration_deg_per_s2_ = 0;
    deceleration_deg_per_s2_ = 0;
    jointDirectMove_ = true;
    thetaStartFrac_ = -1.0f;
    thetaFullSpeed_ = false;
    thetaPending_ = false;
    pathSegments_[0] = Path::PathSegment{x, ceiling, false, true};
    pathSegmentCount_ = 1;
    pathSegmentIndex_ = 0;
    pathSegStartX_mm_ = x;
    pathSegStartZ_mm_ = z;
    pathSegXArmed_ = false;
    pathSegZArmed_ = false;
    retractOnly_ = true;

    if (!moveZAxisTo(ceiling, static_cast<float>(speed_mm_per_s_), z_accel, z_decel)) {
        stopAllMotion();
        return GantryError::TIMEOUT;
    }
    pathSegZArmed_ = true;
    motionState_ = MotionState::Z_RETRACTING;
    ESP_LOGD(TAG, "[RETRACT] Z %.2f -> %.2f mm (Z only)", z, ceiling);
    return GantryError::OK;
}

GantryError Gantry::queueRetractToSafeZ(uint32_t speed_mm_per_s,
                                        uint32_t acceleration_mm_per_s2,
                                        uint32_t deceleration_mm_per_s2) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    if (!axisZ_) {
        return GantryError::NOT_INITIALIZED;
    }
    if (axisZ_->isAlarmActive() ||
        axisZ_->getDrivePositionRef() == DrivePositionRef::kLost) {
        return GantryError::TIMEOUT;
    }
    // Only behind the released plunge (Z-only descent, edge already sent).
    if (motionState_ != MotionState::Z_DESCENDING || !axisZ_->isBusy() ||
        axisZ_->isStartHeld()) {
        return GantryError::INVALID_PARAMETER;
    }

    const float x = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float z_from = targetZ_mm_;  // the plunge depth the retract starts from
    const float ceiling = traverseClearanceZMm();
    const float speed = Path::zCappedPathSpeed(
        static_cast<float>(speed_mm_per_s), 0.0f, ceiling - z_from,
        runtime_.z_speed_cap_mm_s);
    const float z_accel = Path::zCappedPathAccel(
        static_cast<float>(acceleration_mm_per_s2), 0.0f, ceiling - z_from,
        runtime_.z_accel_cap_mm_s2);
    const float z_decel = Path::zCappedPathAccel(
        static_cast<float>(deceleration_mm_per_s2), 0.0f, ceiling - z_from,
        runtime_.z_decel_cap_mm_s2);
    if (!axisZ_->queueHeldMm(ceiling, speed, z_accel, z_decel)) {
        return GantryError::TIMEOUT;
    }

    // The path now ends at the ceiling: busy until the queued retract arrives.
    targetX_mm_ = x;
    targetZ_mm_ = ceiling;
    speed_mm_per_s_ = static_cast<uint32_t>(speed);
    acceleration_mm_per_s2_ = static_cast<uint32_t>(z_accel);
    deceleration_mm_per_s2_ = static_cast<uint32_t>(z_decel);
    thetaPending_ = false;
    pathSegments_[0] = Path::PathSegment{x, ceiling, false, true};
    pathSegmentCount_ = 1;
    pathSegmentIndex_ = 0;
    pathSegStartX_mm_ = x;
    pathSegStartZ_mm_ = z_from;
    pathSegXArmed_ = false;
    pathSegZArmed_ = true;
    retractOnly_ = true;
    motionState_ = MotionState::Z_RETRACTING;
    ESP_LOGD(TAG, "[RETRACT] queued Z %.2f -> %.2f mm behind the plunge (held)", z_from,
             ceiling);
    return GantryError::OK;
}

bool Gantry::chainMoveAfterRetract(const JointConfig& joint,
                                   uint32_t speed_mm_per_s,
                                   uint32_t speed_deg_per_s,
                                   uint32_t acceleration_mm_per_s2,
                                   uint32_t deceleration_mm_per_s2,
                                   uint32_t acceleration_deg_per_s2,
                                   uint32_t deceleration_deg_per_s2,
                                   float theta_start_frac,
                                   bool theta_full_speed) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !enabled_ || !workspaceCalibrated_ || !axisZ_) {
        return false;
    }
    if (motionState_ != MotionState::Z_RETRACTING || !retractOnly_) {
        return false;
    }
    // Only chain once the retract StartMotion edge has fired and Z is moving up.
    if (!axisZ_->isBusy() || axisZ_->isStartHeld()) {
        return false;
    }
    if (axisZ_->isAlarmActive() ||
        (axisX_ && axisX_->isAlarmActive()) ||
        (axisTheta_ && axisTheta_->isAlarmActive())) {
        return false;
    }

    JointConfig goal = joint;
    if (theta_start_frac >= 0.0f) {
        const float cur = axisTheta_ ? axisTheta_->getCurrentDeg()
                                     : static_cast<float>(currentTheta_);
        goal.theta = Path::commandThetaDeg(cur, joint.theta);
    }
    if (!Kinematics::validate(goal, config_.limits)) {
        return false;
    }

    const float curX = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float curZ = axisZ_->getCurrentMm();
    const float ceiling = traverseClearanceZMm();
    const bool needX = std::fabs(goal.x - curX) > Path::kAxisEpsMm;
    const float seg0X = needX ? goal.x : curX;

    targetX_mm_ = goal.x;
    targetZ_mm_ = goal.z;
    targetTheta_deg_ = goal.theta;
    speed_mm_per_s_ = speed_mm_per_s;
    speed_deg_per_s_ = speed_deg_per_s;
    acceleration_mm_per_s2_ = acceleration_mm_per_s2;
    deceleration_mm_per_s2_ = deceleration_mm_per_s2;
    acceleration_deg_per_s2_ = acceleration_deg_per_s2;
    deceleration_deg_per_s2_ = deceleration_deg_per_s2;
    jointDirectMove_ = true;
    thetaStartFrac_ = theta_start_frac;
    thetaFullSpeed_ = theta_full_speed;
    pathDoGripperAfter_ = false;
    retractOnly_ = false;

    pathSegments_[0] = Path::PathSegment{seg0X, ceiling, needX, true};
    pathSegmentCount_ = 1;
    if (std::fabs(goal.z - ceiling) > Path::kAxisEpsMm) {
        pathSegments_[1] = Path::PathSegment{seg0X, goal.z, false, true};
        pathSegmentCount_ = 2;
    }
    pathSegmentIndex_ = 0;
    pathSegStartX_mm_ = curX;
    // Keep the higher Z start so retractLeadMm sees the full retract stroke.
    pathSegStartZ_mm_ = curZ > pathSegStartZ_mm_ ? curZ : pathSegStartZ_mm_;
    pathSegXArmed_ = false;
    pathSegZArmed_ = true;

    if (axisTheta_) {
#if !CONFIG_GANTRY_THETA_SEQUENTIAL
        scheduleThetaForPath();
#endif
        currentTheta_ = static_cast<int32_t>(axisTheta_->getCurrentDeg());
    } else {
        currentTheta_ = static_cast<int32_t>(targetTheta_deg_);
    }

    // If Z already entered the SAFE_Z band, arm X and theta immediately.
    tryArmDeferredPathX();
    tryStartPendingTheta();
    return true;
}

bool Gantry::isRetractHandedOff() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return pathSegXArmed_ || (pathSegmentIndex_ > 0) || !isBusy();
}

GantryError Gantry::moveTo(const EndEffectorPose& pose,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2,
                           uint32_t acceleration_deg_per_s2,
                           uint32_t deceleration_deg_per_s2) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    if (!workspaceCalibrated_) {
        return GantryError::NOT_CALIBRATED;
    }
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
    acceleration_deg_per_s2_ = acceleration_deg_per_s2;
    deceleration_deg_per_s2_ = deceleration_deg_per_s2;
    jointDirectMove_ = false;
    thetaStartFrac_ = -1.0f;
    thetaFullSpeed_ = false;

    startSequentialMotion();
    return GantryError::OK;
}

bool Gantry::isBusy() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) {
        return false;
    }
    const bool xBusy = axisX_ && (axisX_->isMotionActive());
    const bool zBusy = axisZ_ && axisZ_->isBusy();
    const bool tBusy = axisTheta_ && axisTheta_->isMotionActive();
    return motionState_ != MotionState::IDLE || xBusy || zBusy || tBusy ||
           homingInProgress_ || calibrationInProgress_;
}

bool Gantry::isEnabled() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return initialized_ && enabled_;
}

void Gantry::requestAbort() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    abortRequested_ = true;
    if (initialized_) {
        stopAllMotion();
    }
}

bool Gantry::isAbortRequested() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return abortRequested_;
}

void Gantry::update() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED();

    // Advance EIP axis state machines first so isBusy() reflects the latest
    // StartMotion preload/pulse phase before sequential sequencing decides.
    updateAxisPositions();
    pollDrivePositionLoss();

    if (motionState_ != MotionState::IDLE) {
        processSequentialMotion();
    }
}

// ============================================================================
// GRIPPER
// ============================================================================

bool Gantry::grip(bool active) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!endEffector_.isConfigured()) {
        return false;
    }
    if (!endEffector_.setActive(active)) {
        ESP_LOGE(TAG, "gripper %s write failed", active ? "close" : "open");
        return false;
    }
    gripperActive_ = active;
    return true;
}

} // namespace Gantry
