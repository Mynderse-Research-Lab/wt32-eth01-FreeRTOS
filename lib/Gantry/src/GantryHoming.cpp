/**
 * @file GantryHoming.cpp
 * @brief Gantry class: blocking homing and calibration entry points.
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
// HOMING AND CALIBRATION
// ============================================================================

void Gantry::home() {
    // Legacy entry point: only X is currently real, so behavior is unchanged.
    // Z and Theta stubs are NOT invoked here so existing callers that depend
    // on `home()` blocking only on the X sweep keep working.
    homeX();
}

void Gantry::homeX() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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

bool Gantry::homeTheta() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED_RET(false);
    GANTRY_CHECK_ENABLED_RET(false);
    abortRequested_ = false;

    if (!axisTheta_) {
        ESP_LOGW(TAG, "[HOME] Theta axis not constructed (CONFIG_EIP_AXIS_THETA)");
        return false;
    }
    if (axisTheta_->isAlarmActive()) {
        ESP_LOGE(TAG, "[HOME] Theta alarm active — clear before soft-home");
        return false;
    }
    if (!axisTheta_->hasLiveFeedback()) {
        ESP_LOGE(TAG, "[HOME] Theta Class 1 feedback not live");
        return false;
    }
    if (!axisTheta_->captureSoftHome()) {
        ESP_LOGE(TAG, "[HOME] Theta origin capture failed");
        return false;
    }
    setThetaLimits(axisTheta_->getMinDeg(), axisTheta_->getMaxDeg());
    currentTheta_ = static_cast<int32_t>(axisTheta_->getCurrentDeg());
    ESP_LOGI(TAG,
             "[HOME] Theta origin: drive_abs=%.3f deg aligned=%d joint_lim=%.2f..%.2f "
             "(continuous slip-ring mode)",
             static_cast<double>(axisTheta_->getDriveAbsDeg()),
             axisTheta_->isDriveOriginAligned() ? 1 : 0,
             static_cast<double>(axisTheta_->getMinDeg()),
             static_cast<double>(axisTheta_->getMaxDeg()));

    ESP_LOGI(TAG, "[HOME] Rotating Theta back to physical zero...");
    axisTheta_->moveToDeg(0.0f, runtime_.home_theta_speed_deg_s, 0.0f, 0.0f);

    return true;
}

bool Gantry::thetaSettledAt(float target_deg) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!axisTheta_ || axisTheta_->isBusy() || axisTheta_->isAlarmActive()) {
        return false;
    }
    // Accept up to 0.5 deg so stiction-settled stops (<= 0.30 deg) pass bring-up orient.
    constexpr float kBringUpThetaTolDeg = 0.5f;
    return std::fabs(axisTheta_->getCurrentDeg() - target_deg) <= kBringUpThetaTolDeg;
}

void Gantry::softHomeJointDatum() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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

// advanceEipLimitSequence() limits each of the six precision home/cal phases
// to TRAVEL_MEASUREMENT_TIMEOUT_MS; the callers' wait only backstops the whole
// run. One phase budget for the total timed out a 500 mm X calibration at the
// 50 mm/s seek speed (~24 s, 2026-10-27).
constexpr unsigned long kEipCalTotalTimeoutMs = 6 * TRAVEL_MEASUREMENT_TIMEOUT_MS;

int Gantry::calibrateZ() {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    }
    unsigned long start_ms = gantry_millis();
    while (true) {
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (!calibrationInProgress_ ||
                (gantry_millis() - start_ms) >= kEipCalTotalTimeoutMs) {
                break;
            }
            if (abortRequested_) {
                stopAllMotion();
                calibrationInProgress_ = false;
                return 0;
            }
        }
        gantry_delay(10);
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (calibrationInProgress_) {
        ESP_LOGE(TAG, "[CAL] EIP Z calibrate timed out");
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }
    return zAxisLength_;
}

int Gantry::calibrateTheta() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GANTRY_CHECK_INITIALIZED_RET(0);
    GANTRY_CHECK_ENABLED_RET(0);

    if (!axisTheta_) {
        ESP_LOGW(TAG, "[CAL] Theta axis not constructed (CONFIG_EIP_AXIS_THETA)");
        return 0;
    }
    if (axisTheta_->isAlarmActive()) {
        ESP_LOGE(TAG, "[CAL] Theta alarm active — clear before calibrate");
        return 0;
    }
    if (!axisTheta_->hasLiveFeedback()) {
        ESP_LOGE(TAG, "[CAL] Theta Class 1 / encoder not live");
        return 0;
    }

    axisTheta_->setAngleRange(config_.limits.theta_min, config_.limits.theta_max);
    const float span = config_.limits.theta_max - config_.limits.theta_min;
    ESP_LOGI(TAG,
             "[CAL] Theta: joint envelope %.1f .. %.1f deg (span %.1f); "
             "drive_abs=%.3f aligned=%d",
             static_cast<double>(config_.limits.theta_min),
             static_cast<double>(config_.limits.theta_max),
             static_cast<double>(span),
             static_cast<double>(axisTheta_->getDriveAbsDeg()),
             axisTheta_->isDriveOriginAligned() ? 1 : 0);
    return static_cast<int>(span > 0.0f ? span : 0.0f);
}

int Gantry::calibrateX() {
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
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
            // Proceed to polling below outside the lock
        } else {
            calibrationInProgress_ = true;

            if (!xMinSwitch_.isConfigured() || !xMaxSwitch_.isConfigured()) {
                stopAllMotion();
                calibrationInProgress_ = false;
                return 0;
            }

            homeX();
        }
    }

    if (driveManagedLimitsEnabled()) {
        unsigned long start_ms = gantry_millis();
        while (true) {
            {
                std::lock_guard<std::recursive_mutex> lock(mutex_);
                if (!calibrationInProgress_ ||
                    (gantry_millis() - start_ms) >= kEipCalTotalTimeoutMs) {
                    break;
                }
                if (abortRequested_) {
                    stopAllMotion();
                    calibrationInProgress_ = false;
                    return 0;
                }
            }
            gantry_delay(10);
        }
        std::lock_guard<std::recursive_mutex> lock(mutex_);
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
    const double hardSpanMm = (double)runtime_.x_hard_span_mm;
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

} // namespace Gantry
