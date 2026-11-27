/**
 * @file GantryPathTheta.cpp
 * @brief Gantry class: theta move scheduling along a path.
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
// THETA SCHEDULING ALONG A PATH
// ============================================================================

float Gantry::thetaAccelDegPerS2() const {
    if (acceleration_deg_per_s2_ > 0) {
        return static_cast<float>(acceleration_deg_per_s2_);
    }
    return runtime_.theta_accel_deg_s2;
}

float Gantry::thetaDecelDegPerS2() const {
    if (deceleration_deg_per_s2_ > 0) {
        return static_cast<float>(deceleration_deg_per_s2_);
    }
    return runtime_.theta_decel_deg_s2;
}

bool Gantry::commandThetaMove(float speed_deg_s) {
    if (!axisTheta_) {
        return false;
    }
    if (!axisTheta_->moveToDeg(targetTheta_deg_,
                               speed_deg_s,
                               thetaAccelDegPerS2(),
                               thetaDecelDegPerS2())) {
        const float currentTheta = axisTheta_->getCurrentDeg();
        const float already =
            (thetaStartFrac_ >= 0.0f) ? Path::kThetaCommandMinDeg : 0.5f;
        if (std::fabs(currentTheta - targetTheta_deg_) >= already) {
            ESP_LOGE(TAG,
                     "[THETA] moveToDeg failed (current=%.2f target=%.2f) - aborting sequence",
                     currentTheta, targetTheta_deg_);
            stopAllMotion();
            return false;
        }
        ESP_LOGI(TAG,
                 "[THETA] moveToDeg no-op treated as success (current=%.2f target=%.2f)",
                 currentTheta, targetTheta_deg_);
    }
    currentTheta_ = (int32_t)axisTheta_->getCurrentDeg();
    return true;
}

float Gantry::pathSegProgressFrac() const {
    if (pathSegmentIndex_ >= pathSegmentCount_) {
        return 1.0f;
    }
    const Path::PathSegment& seg = pathSegments_[pathSegmentIndex_];
    const float dx = seg.x_mm - pathSegStartX_mm_;
    const float dz = seg.z_mm - pathSegStartZ_mm_;
    const float L = std::hypot(dx, dz);
    if (L <= Path::kAxisEpsMm) {
        return 1.0f;
    }
    const float x = axisX_ ? axisX_->getCurrentMm() : currentX_mm_;
    const float z = axisZ_ ? axisZ_->getCurrentMm() : static_cast<float>(currentZ_);
    float frac = std::hypot(x - pathSegStartX_mm_, z - pathSegStartZ_mm_) / L;
    if (frac < 0.0f) {
        frac = 0.0f;
    }
    if (frac > 1.0f) {
        frac = 1.0f;
    }
    return frac;
}

void Gantry::scheduleThetaForPath() {
    thetaPending_ = false;
    thetaPendingSpeedDegS_ = 0.0f;
    thetaWindowSegIndex_ = 0;
    thetaHoldsDescent_ = false;

    if (!axisTheta_ || pathSegmentCount_ == 0) {
        return;
    }

    const float dth = targetTheta_deg_ - axisTheta_->getCurrentDeg();
    const bool pickMove = thetaStartFrac_ >= 0.0f;
    if (pickMove) {
        if (std::fabs(dth) < Path::kThetaCommandMinDeg) {
            return;
        }
    } else if (std::fabs(dth) <= 0.5f) {
        return;
    }

    const float ceiling = traverseClearanceZMm();
    float cx = pathSegStartX_mm_;
    float cz = pathSegStartZ_mm_;
    size_t pick = pathSegmentCount_;
    float pick_len = 0.0f;
    size_t x_pick = pathSegmentCount_;
    float x_len = 0.0f;

    for (size_t i = 0; i < pathSegmentCount_; ++i) {
        const Path::PathSegment& seg = pathSegments_[i];
        const float len = std::hypot(seg.x_mm - cx, seg.z_mm - cz);
        if (Path::zInTraverseBand(seg.z_mm, ceiling)) {
            pick = i;
            pick_len = len;
            if (seg.move_x) {
                x_pick = i;
                x_len = len;
            }
        }
        cx = seg.x_mm;
        cz = seg.z_mm;
    }
    if (x_pick < pathSegmentCount_) {
        pick = x_pick;
        pick_len = x_len;
    }

    if (pick >= pathSegmentCount_) {
        if (requireThetaTraverseInterlock("path theta") && pickMove) {
            const float spd = speed_deg_per_s_ > 0
                                  ? static_cast<float>(speed_deg_per_s_)
                                  : runtime_.theta_max_speed_deg_s;
            (void)commandThetaMove(spd);
        }
        return;
    }

    float start_frac = Path::kThetaStartFrac;
    float cap = runtime_.theta_max_speed_deg_s;
    if (thetaStartFrac_ >= 0.0f) {
        start_frac = thetaStartFrac_;
        if (speed_deg_per_s_ > 0 && static_cast<float>(speed_deg_per_s_) < cap) {
            cap = static_cast<float>(speed_deg_per_s_);
        }
    }
    if (thetaFullSpeed_) {
        thetaPending_ = true;
        thetaPendingSpeedDegS_ = cap > 0.0f ? cap : static_cast<float>(speed_deg_per_s_);
        thetaWindowSegIndex_ = pick;
        thetaWindowStartFrac_ = start_frac;
        ESP_LOGD(TAG,
                 "[THETA] pending full-speed seg %u/%u start=%.0f%% speed=%.1f deg/s dtheta=%.2f",
                 (unsigned)(pick + 1), (unsigned)pathSegmentCount_,
                 start_frac * 100.0f, thetaPendingSpeedDegS_, dth);
        return;
    }
    const Path::ThetaWindow w = Path::planThetaWindow(
        pick_len,
        static_cast<float>(speed_mm_per_s_),
        dth,
        cap,
        start_frac);
    if (!w.runnable) {
        if (pickMove && requireThetaTraverseInterlock("path theta")) {
            const float spd = speed_deg_per_s_ > 0
                                  ? static_cast<float>(speed_deg_per_s_)
                                  : runtime_.theta_max_speed_deg_s;
            (void)commandThetaMove(spd);
        }
        return;
    }

    thetaPending_ = true;
    thetaPendingSpeedDegS_ = w.speed_deg_per_s;
    thetaWindowSegIndex_ = pick;
    thetaWindowStartFrac_ = start_frac;
    ESP_LOGD(TAG,
             "[THETA] pending window seg %u/%u start=%.0f%% speed=%.1f deg/s "
             "clamped=%d dtheta=%.2f",
             (unsigned)(pick + 1), (unsigned)pathSegmentCount_,
             start_frac * 100.0f, w.speed_deg_per_s,
             (int)w.speed_clamped, dth);
}

void Gantry::tryStartPendingTheta() {
    if (!thetaPending_ || !axisTheta_) {
        return;
    }
    if (pathSegmentIndex_ != thetaWindowSegIndex_) {
        return;
    }
    const bool in_band = zInTraverseBand();
    const bool early_lead = !in_band && retractLeadReady(Path::kRetractThetaLeadS);
    if (!in_band && !early_lead) {
        return;
    }
    if (pathSegProgressFrac() < thetaWindowStartFrac_) {
        return;
    }
    if (!early_lead && !requireThetaTraverseInterlock("path theta")) {
        thetaPending_ = false;
        return;
    }
    const float speed = thetaPendingSpeedDegS_;
    thetaPending_ = false;
    (void)commandThetaMove(speed);
    ESP_LOGD(TAG, "[THETA] window start at %.0f%% of in-band seg %u (%.1f deg/s)",
             pathSegProgressFrac() * 100.0f,
             (unsigned)(thetaWindowSegIndex_ + 1),
             speed);
}

void Gantry::startThetaOrIdle() {
    if (axisTheta_) {
        const float currentTheta = axisTheta_->getCurrentDeg();
        const float delta = std::fabs(currentTheta - targetTheta_deg_);
        const bool need = (thetaStartFrac_ >= 0.0f)
                              ? (delta >= Path::kThetaCommandMinDeg)
                              : (delta > 0.5f);
        if (need && !axisTheta_->isMotionActive()) {
            if (!requireThetaTraverseInterlock("theta move")) {
                thetaPending_ = false;
            } else {
                const float spd =
                    (thetaPending_ && thetaPendingSpeedDegS_ > 0.0f)
                        ? thetaPendingSpeedDegS_
                        : static_cast<float>(speed_deg_per_s_);
                thetaPending_ = false;
                if (!commandThetaMove(spd)) {
                    return;
                }
                pathSegmentCount_ = 0;
                pathSegmentIndex_ = 0;
                pathSegXArmed_ = false;
                pathSegZArmed_ = false;
                motionState_ = MotionState::THETA_MOVING;
                return;
            }
        }
    }
    thetaPending_ = false;
    motionState_ = MotionState::IDLE;
    pathSegmentCount_ = 0;
    pathSegmentIndex_ = 0;
    pathSegXArmed_ = false;
    pathSegZArmed_ = false;
}

} // namespace Gantry
