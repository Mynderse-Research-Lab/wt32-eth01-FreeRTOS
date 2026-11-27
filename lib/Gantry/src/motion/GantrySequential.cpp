/**
 * @file GantrySequential.cpp
 * @brief Gantry class: sequential motion and X axis motion start.
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

void Gantry::startSequentialMotion() {
    if (!enabled_ || motionState_ != MotionState::IDLE) {
        return;
    }

    const float currentX = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float currentZ = axisZ_ ? axisZ_->getCurrentMm() : (float)currentZ_;

    // Descending (+Z down, toward belt) = picking (grip close).
    // Retracting (toward A015) = placing (grip open).
    gripperTargetState_ = (targetZ_mm_ > currentZ);
    gripperActuateDurationMs_ = gripperTargetState_
        ? runtime_.gripper_close_ms
        : runtime_.gripper_open_ms;

    pathDoGripperAfter_ = false;

    if (jointDirectMove_) {
        ESP_LOGD(TAG,
                 "[JOINT_DIRECT] path (%.3f,%.3f) -> (%.3f,%.3f) SAFE_Z "
                 "ceiling=%.1f (z_min=%.1f margin=%.1f)",
                 currentX, currentZ, targetX_mm_, targetZ_mm_,
                 traverseClearanceZMm(), config_.limits.z_min,
                 safeZMarginFromMaxMm_);
        if (!planPathTo(currentX, currentZ, targetX_mm_, targetZ_mm_)) {
            ESP_LOGD(TAG,
                     "[JOINT_DIRECT] linear already at target (%.3f,%.3f); "
                     "theta %.2f -> %.2f",
                     currentX, currentZ,
                     axisTheta_ ? axisTheta_->getCurrentDeg() : 0.0f,
                     targetTheta_deg_);
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
        if (pathSegmentCount_ > 0) {
            scheduleThetaForPath();
            if (thetaPending_ && thetaWindowStartFrac_ <= 0.0f && zInTraverseBand()) {
                tryStartPendingTheta();
            }
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

    if (motionState_ != MotionState::IDLE) {
        const bool other_alarm =
            !retractOnly_ && ((axisX_ && axisX_->isAlarmActive()) ||
                              (axisTheta_ && axisTheta_->isAlarmActive()));
        if (other_alarm || (axisZ_ && axisZ_->isAlarmActive())) {
            ESP_LOGE(TAG, "[PATH] cross-axis alarm detected, aborting sequence");
            failPath("axis alarm");
            return;
        }
    }

    switch (motionState_) {
        case MotionState::Z_DESCENDING:
        case MotionState::Z_RETRACTING:
        case MotionState::X_MOVING:
        case MotionState::XZ_MOVING:
            tryArmDeferredPathX();
            tryStartPendingTheta();
            if (motionState_ == MotionState::IDLE) {
                return;
            }
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
                              float decel_mm_s2, bool allow_retract_lead) {
    if (!axisX_) {
        return false;
    }
    if (!allow_retract_lead && !requireXTraverseInterlock("path X Absolute")) {
        return false;
    }

    const float xPpm = getPulsesPerMm();

    const int32_t driverReportedPulses = (int32_t)axisX_->getCurrentPulses();
    const int32_t trackedPulses        = (int32_t)(currentX_mm_ * xPpm);
    const int32_t encoderPulses        = axisX_->getEncoderPulses();
    const int32_t targetPulses         = (int32_t)(target_mm * xPpm);

    axisX_->setCurrentPulses((uint32_t)driverReportedPulses);
    const int32_t deltaX = targetPulses - driverReportedPulses;

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

    if (axisX_->isEncoderFeedbackEnabled()) {
        const int32_t encDrvMismatch = encoderPulses - driverReportedPulses;
        if (encDrvMismatch > 10 || encDrvMismatch < -10) {
            ESP_LOGD(TAG,
                     "[X_MOVE] encoder/driver mismatch: encoder=%ld driver=%ld mismatch=%ld (planning uses driver)",
                     (long)encoderPulses, (long)driverReportedPulses, (long)encDrvMismatch);
        }
    }

    const int32_t trackedMismatch = trackedPulses - driverReportedPulses;
    if (trackedMismatch > 10 || trackedMismatch < -10) {
        ESP_LOGD(TAG,
                 "[X_MOVE] tracked/driver mismatch: tracked=%ld driver=%ld mismatch=%ld",
                 (long)trackedPulses, (long)driverReportedPulses, (long)trackedMismatch);
    }

    ESP_LOGD(TAG,
             "[X_MOVE] current=%ld tracked=%ld driver=%ld encoder=%ld target=%ld delta=%ld "
             "speed=%.1f accel=%.1f decel=%.1f",
             (long)driverReportedPulses, (long)trackedPulses, (long)driverReportedPulses,
             (long)encoderPulses, (long)targetPulses, (long)deltaX,
             speed_mm_s, accel_mm_s2, decel_mm_s2);
    return true;
}

} // namespace Gantry
