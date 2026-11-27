/**
 * @file GantryAxisState.cpp
 * @brief Gantry class: accessors, kinematics and axis state helpers.
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
// ACCESSORS
// ============================================================================

int Gantry::getXEncoder() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !axisX_) return 0;
    if (axisX_->isEncoderFeedbackEnabled()) {
        return axisX_->getEncoderPulses();
    }
    return (int32_t)axisX_->getCurrentPulses();
}

int Gantry::getXEncoderRaw() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !axisX_) return 0;
    return axisX_->getEncoderPulses();
}

int32_t Gantry::getXCommandedPulses() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !axisX_) return 0;
    if (homingInProgress_ || calibrationInProgress_) {
        return 0;
    }
    return (int32_t)axisX_->getCurrentPulses();
}

float Gantry::getXCommandedMm() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)((double)getXCommandedPulses() / (double)ppm);
}

float Gantry::getXEncoderMm() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)((double)getXEncoderRaw() / (double)ppm);
}

int Gantry::getCurrentZ() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return (int)(axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_);
}

float Gantry::getZCommandedMm() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisZ_ ? axisZ_->getTargetMm() : targetZ_mm_;
}

float Gantry::getZEncoderMm() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_;
}

bool Gantry::sampleZFeedbackMm(float& mm) const {
    // axisZ_ is set by the constructor and never reseated.
    if (!axisZ_ || !axisZ_->isLinkOnline()) {
        return false;
    }
    mm = axisZ_->getCurrentMm();
    return true;
}

bool Gantry::sampleThetaTrace(RotaryTraceSample& out) const {
    // axisTheta_ is set by the constructor and never reseated.
    return axisTheta_ && axisTheta_->sampleTrace(out);
}

bool Gantry::getThetaMoveStamp(RotaryMoveStamp& out) const {
    return axisTheta_ && axisTheta_->getMoveStamp(out);
}

int32_t Gantry::getZEncoderPulses() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !axisZ_) return 0;
    return axisZ_->getEncoderPulses();
}

float Gantry::getZPulsesPerMm() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (axisZ_ && axisZ_->pulsesPerMm() > 0.0) {
        return static_cast<float>(axisZ_->pulsesPerMm());
    }
    return 0.0f;
}

int Gantry::getCurrentTheta() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return static_cast<int>(getCurrentThetaDeg());
}

float Gantry::getCurrentThetaDeg() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisTheta_ ? axisTheta_->getCurrentDeg() : static_cast<float>(currentTheta_);
}

float Gantry::getThetaDriveAbsDeg() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisTheta_ ? axisTheta_->getDriveAbsDeg() : static_cast<float>(currentTheta_);
}

bool Gantry::isThetaDriveOriginAligned() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisTheta_ && axisTheta_->isDriveOriginAligned();
}

float Gantry::getThetaPulsesPerDeg() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (axisTheta_ && axisTheta_->pulsesPerDeg() > 0.0) {
        return static_cast<float>(axisTheta_->pulsesPerDeg());
    }
    return 0.0f;
}

bool Gantry::setThetaPuuPerDeg(double puu_per_deg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!axisTheta_) {
        return false;
    }
    return axisTheta_->setPuuPerDeg(puu_per_deg);
}

bool Gantry::hasThetaAxis() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisTheta_ != nullptr;
}

bool Gantry::hasThetaLiveFeedback() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return axisTheta_ && axisTheta_->hasLiveFeedback();
}

bool Gantry::getThetaCipStatus(char* buf, size_t n) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!axisTheta_) {
        if (buf && n) buf[0] = '\0';
        return false;
    }
    return axisTheta_->formatCipStatus(buf, n);
}

bool Gantry::getThetaDriveAlarmSummary(char* buf, size_t n) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!axisTheta_) {
        if (buf && n) buf[0] = '\0';
        return false;
    }
    return axisTheta_->getDriveAlarmSummary(buf, n);
}

// ============================================================================
// KINEMATICS
// ============================================================================

EndEffectorPose Gantry::forwardKinematics(const JointConfig& joint) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return Kinematics::forward(joint, kinematicParams_);
}

JointConfig Gantry::inverseKinematics(const EndEffectorPose& pose) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return Kinematics::inverse(pose, kinematicParams_);
}

JointConfig Gantry::getCurrentJointConfig() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    JointConfig joint;
    joint.x     = currentX_mm_;
    joint.z     = axisZ_     ? axisZ_->getCurrentMm()     : (float)currentZ_;
    joint.theta = axisTheta_ ? axisTheta_->getCurrentDeg() : (float)currentTheta_;
    return joint;
}

JointConfig Gantry::getTargetJointConfig() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    JointConfig joint;
    joint.x     = targetX_mm_;
    joint.z     = axisZ_ ? axisZ_->getTargetMm() : targetZ_mm_;
    joint.theta = axisTheta_ ? axisTheta_->getTargetDeg() : (float)targetTheta_;
    return joint;
}

EndEffectorPose Gantry::getCurrentEndEffectorPose() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return forwardKinematics(getCurrentJointConfig());
}

EndEffectorPose Gantry::getTargetEndEffectorPose() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return forwardKinematics(getTargetJointConfig());
}

// ============================================================================
// HELPERS
// ============================================================================

void Gantry::setStepsPerRevolution(float steps_per_rev) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (steps_per_rev > 0) {
        stepsPerRev_ = steps_per_rev;
    }
}

float Gantry::getPulsesPerMm() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
        const float curMm = axisZ_->getCurrentMm();
        if (holdZStart_) {
            holdZStart_ = false;
            const bool ok = axisZ_->moveToMmHeld(targetZ, speed, accel, decel);
            zStartHeld_ = ok && axisZ_->isStartHeld();
            ESP_LOGD(TAG,
                     "[Z_MOVE] current=%.3f mm target=%.3f mm pulses=%lu "
                     "speed=%.1f accel=%.1f decel=%.1f (held)",
                     curMm, targetZ,
                     (unsigned long)axisZ_->getCurrentPulses(), speed, accel, decel);
            return ok;
        }
        const bool ok = axisZ_->moveToMm(targetZ, speed, accel, decel);
        ESP_LOGD(TAG,
                 "[Z_MOVE] current=%.3f mm target=%.3f mm pulses=%lu "
                 "speed=%.1f accel=%.1f decel=%.1f",
                 curMm, targetZ,
                 (unsigned long)axisZ_->getCurrentPulses(), speed, accel, decel);
        return ok;
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
            // Only a GPIO home switch marks joint zero here. A drive-managed
            // switch is also set by any fault-stop, so with drive-managed
            // limits the EIP home sequence alone sets the datum (G-A9).
            if (xMinSwitch_.source() != GantryLimitSwitch::Source::kDriveManaged) {
                axisX_->setCurrentPulses(0);
            }
            homingInProgress_ = false;
        }
        lastXPositionCounts_ = currentXCounts;
    } else if (homingInProgress_) {
        if (xMinSwitch_.isActive() &&
            xMinSwitch_.source() != GantryLimitSwitch::Source::kDriveManaged) {
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

bool Gantry::isWorkspaceCalibrated() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return workspaceCalibrated_;
}

void Gantry::setWorkspaceCalibrated(bool on) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (workspaceCalibrated_ == on) {
        return;
    }
    workspaceCalibrated_ = on;
    ESP_LOGI(TAG, "Workspace calibrated: %s", on ? "Yes" : "No");
}

bool Gantry::driveRefPollPaused() const {
    return homingInProgress_ || calibrationInProgress_ ||
           eipLimitPhase_ != EipLimitPhase::kIdle ||
           bringUpPhase_ != BringUpPhase::kIdle;
}

void Gantry::tryLatchWorkspaceFromStrokes() {
    if (axisLength_ > 0 && zAxisLength_ > 0) {
        setWorkspaceCalibrated(true);
    }
}

void Gantry::pollDrivePositionLoss() {
    if (!workspaceCalibrated_ || driveRefPollPaused()) {
        return;
    }
    auto linearLost = [](GantryLinearAxis* axis, const char* name) {
        if (axis && axis->getDrivePositionRef() == DrivePositionRef::kLost) {
            ESP_LOGW(TAG, "%s drive position ref lost (24 V / unhomed)", name);
            return true;
        }
        return false;
    };
    if (linearLost(axisX_.get(), "X") || linearLost(axisZ_.get(), "Z")) {
        setWorkspaceCalibrated(false);
        return;
    }
    if (axisTheta_ &&
        axisTheta_->getDrivePositionRef() == DrivePositionRef::kLost) {
        ESP_LOGW(TAG, "Theta drive position ref lost (24 V / not in-reference)");
        setWorkspaceCalibrated(false);
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
    eipLimitSawBusy_       = false;
    bringUpPhase_          = BringUpPhase::kIdle;
    motionState_           = MotionState::IDLE;
    jointDirectMove_       = false;
    pathSegmentCount_      = 0;
    pathSegmentIndex_      = 0;
    pathSegXArmed_         = false;
    pathSegZArmed_         = false;
    pathDoGripperAfter_    = false;
    thetaPending_          = false;
    thetaPendingSpeedDegS_ = 0.0f;
    thetaWindowSegIndex_   = 0;
    thetaHoldsDescent_     = false;
    thetaFullSpeed_        = false;
    holdZStart_            = false;
    zStartHeld_            = false;
    retractOnly_           = false;
}
// ============================================================================
// EIP BRING-UP STATE
// ============================================================================

bool Gantry::eipBringUpInProgress() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return bringUpPhase_ != BringUpPhase::kIdle;
}

} // namespace Gantry
