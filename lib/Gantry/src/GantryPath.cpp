/**
 * @file GantryPath.cpp
 * @brief Gantry class: path planning and segment arming.
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
// PATH / SEQUENTIAL MOTION
// ============================================================================

bool Gantry::planPathTo(float x0, float z0, float x1, float z1) {
    const float clearance = traverseClearanceZMm();
    pathSegmentCount_ = Path::planSegments(x0, z0, x1, z1, clearance,
                                           pathSegments_);
    pathSegmentIndex_ = 0;
    pathSegStartX_mm_ = x0;
    pathSegStartZ_mm_ = z0;
    pathSegXArmed_ = false;
    pathSegZArmed_ = false;
    return pathSegmentCount_ > 0;
}

bool Gantry::pathSegmentAxesIdle() const {
    if (pathSegmentIndex_ >= pathSegmentCount_) {
        return true;
    }
    const Path::PathSegment& seg = pathSegments_[pathSegmentIndex_];
    if (seg.move_x && !pathSegXArmed_) {
        return false;
    }
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
    return Path::segmentEndpointsReached(x, z, seg, Path::kArriveEpsMm);
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
    const bool x_unlocked = !seg.move_x || zInTraverseBand();
    const bool together = seg.move_x && seg.move_z && x_unlocked;
    const float cap_dx = together ? dx : 0.0f;
    const float cap_dz = seg.move_z ? dz : 0.0f;
    const Path::PathProfile profile{
        Path::zCappedPathSpeed(static_cast<float>(speed_mm_per_s_),
                               cap_dx, cap_dz, runtime_.z_speed_cap_mm_s),
        Path::zCappedPathAccel(static_cast<float>(acceleration_mm_per_s2_),
                               cap_dx, cap_dz, runtime_.z_accel_cap_mm_s2),
        Path::zCappedPathAccel(static_cast<float>(deceleration_mm_per_s2_),
                               cap_dx, cap_dz, runtime_.z_decel_cap_mm_s2),
    };
    const Path::AxisComponents c = together
        ? Path::decompose(dx, dz, profile)
        : Path::decompose((seg.move_x && x_unlocked) ? dx : 0.0f,
                          seg.move_z ? dz : 0.0f, profile);

    pathSegXArmed_ = false;
    pathSegZArmed_ = false;

    if (seg.move_z) {
        if (!moveZAxisTo(seg.z_mm, c.z_speed, c.z_accel, c.z_decel)) {
            ESP_LOGE(TAG, "[PATH] Z arm failed");
            stopAllMotion();
            return false;
        }
        pathSegZArmed_ = true;
    }
    if (seg.move_x && x_unlocked) {
        if (!startXAxisMotion(seg.x_mm, c.x_speed, c.x_accel, c.x_decel)) {
            ESP_LOGE(TAG, "[PATH] X arm failed");
            stopAllMotion();
            return false;
        }
        pathSegXArmed_ = true;
    }

    ESP_LOGD(TAG,
             "[PATH] seg %u/%u -> (%.3f, %.3f) move_x=%d move_z=%d "
             "vx=%.1f vz=%.1f ax=%.1f az=%.1f%s",
             (unsigned)(pathSegmentIndex_ + 1), (unsigned)pathSegmentCount_,
             seg.x_mm, seg.z_mm, (int)seg.move_x, (int)seg.move_z,
             c.x_speed, c.z_speed, c.x_accel, c.z_accel,
             (seg.move_x && !x_unlocked) ? " (X deferred until SAFE_Z)" : "");

    if (pathSegXArmed_ && pathSegZArmed_) {
        motionState_ = MotionState::XZ_MOVING;
    } else if (pathSegXArmed_) {
        motionState_ = MotionState::X_MOVING;
    } else if (pathSegZArmed_) {
        motionState_ = (seg.z_mm > pathSegStartZ_mm_)
                           ? MotionState::Z_DESCENDING
                           : MotionState::Z_RETRACTING;
    } else {
        return false;
    }
    return true;
}

bool Gantry::retractLeadReady(float lead_s) const {
    if (thetaStartFrac_ < 0.0f || pathSegmentIndex_ >= pathSegmentCount_) {
        return false;
    }
    const Path::PathSegment& seg = pathSegments_[pathSegmentIndex_];
    const float ceiling = traverseClearanceZMm();
    if (!seg.move_z || !pathSegZArmed_ || seg.z_mm > ceiling + Path::kAxisEpsMm) {
        return false;
    }
    if (!axisZ_ || !axisZ_->isBusy() || axisZ_->isStartHeld() || axisZ_->isAlarmActive()) {
        return false;
    }
    const float dz = pathSegStartZ_mm_ - ceiling;
    if (dz <= Path::kAxisEpsMm) {
        return false;
    }
    const float z_speed = Path::zCappedPathSpeed(
        static_cast<float>(speed_mm_per_s_), 0.0f, dz, runtime_.z_speed_cap_mm_s);
    const float z_decel = Path::zCappedPathAccel(
        static_cast<float>(deceleration_mm_per_s2_), 0.0f, dz, runtime_.z_decel_cap_mm_s2);
    const float lead_mm = Path::retractLeadMm(pathSegStartZ_mm_, ceiling, z_speed, z_decel, lead_s);
    if (lead_mm <= 0.0f) {
        return false;
    }
    return axisZ_->getCurrentMm() <= (ceiling + lead_mm);
}

void Gantry::tryArmDeferredPathX() {
    if (pathSegmentIndex_ >= pathSegmentCount_) {
        return;
    }
    const Path::PathSegment& seg = pathSegments_[pathSegmentIndex_];
    if (!seg.move_x || pathSegXArmed_) {
        return;
    }
    const bool in_band = zInTraverseBand();
    const bool early_lead = !in_band && retractLeadReady(Path::kRetractXLeadS);
    if (!in_band && !early_lead) {
        return;
    }

    const float dx = seg.x_mm - pathSegStartX_mm_;
    const Path::PathProfile profile{
        static_cast<float>(speed_mm_per_s_),
        static_cast<float>(acceleration_mm_per_s2_),
        static_cast<float>(deceleration_mm_per_s2_),
    };
    const Path::AxisComponents c = Path::decompose(dx, 0.0f, profile);
    if (!startXAxisMotion(seg.x_mm, c.x_speed, c.x_accel, c.x_decel, early_lead)) {
        ESP_LOGE(TAG, "[PATH] deferred X arm failed");
        failPath("deferred X arm failed");
        return;
    }
    pathSegXArmed_ = true;
    if (pathSegZArmed_ && axisZ_ && axisZ_->isBusy()) {
        motionState_ = MotionState::XZ_MOVING;
    } else {
        motionState_ = MotionState::X_MOVING;
    }
    ESP_LOGD(TAG, "[PATH] X unlocked (%s, Z=%.2f) -> %.3f mm vx=%.1f",
             early_lead ? "retract lead" : "SAFE_Z band",
             axisZ_ ? axisZ_->getCurrentMm() : static_cast<float>(currentZ_),
             seg.x_mm, c.x_speed);
}

void Gantry::finishPathOrAdvance() {
    tryStartPendingTheta();

    const size_t next_i = pathSegmentIndex_ + 1;
    if (next_i < pathSegmentCount_) {
        const Path::PathSegment& next = pathSegments_[next_i];
        const bool descends_below =
            next.z_mm > (traverseClearanceZMm() + Path::kAxisEpsMm);
        const bool theta_active =
            axisTheta_ && axisTheta_->isMotionActive();
        if (descends_below && (theta_active || thetaPending_)) {
            if (thetaPending_ && zInTraverseBand()) {
                thetaPending_ = false;
                if (!commandThetaMove(thetaPendingSpeedDegS_)) {
                    return;
                }
            }
            if (axisTheta_ && axisTheta_->isMotionActive()) {
                if (!thetaHoldsDescent_) {
                    thetaHoldsDescent_ = true;
                    ESP_LOGD(TAG,
                             "[THETA] holding descent until theta completes "
                             "(speed clamped)");
                }
                return;
            }
        }
    }
    thetaHoldsDescent_ = false;

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
    pathSegXArmed_ = false;
    pathSegZArmed_ = false;

    if (retractOnly_) {
        retractOnly_ = false;
        thetaPending_ = false;
        motionState_ = MotionState::IDLE;
        return;
    }

    if (pathDoGripperAfter_) {
        pathDoGripperAfter_ = false;
        motionState_ = MotionState::GRIPPER_ACTUATING;
        grip(gripperTargetState_);
        gripperActuateStart_ms_ = gantry_millis();
        return;
    }

    startThetaOrIdle();
}

} // namespace Gantry
