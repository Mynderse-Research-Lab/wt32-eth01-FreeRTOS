/**
 * @file GantryEipBringUp.cpp
 * @brief Gantry class: EtherNet/IP bring-up sequence.
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
// EIP BRING-UP (Z- → X home/cal → X park → Z+ stroke → return 0)
// ============================================================================

bool Gantry::startEipBringUp() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
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
        ESP_LOGI(TAG, "[BRINGUP] seek -Z A015 at %.1f mm/s a=%.0f",
                 eipSeekSpeedMmS(), eipHomeCalAccelMmS2());
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
    tryLatchWorkspaceFromStrokes();
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
                z->setCurrentPulses(0);
                z->stopMotion();
                bringUpPhase_ = BringUpPhase::kZMinusSettle;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
                ESP_LOGI(TAG,
                         "[BRINGUP] -Z switch disabled — Z=0 latched at this sample");
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
                currentZ_ = (int32_t)z->getCurrentMm();
                // SAFE_Z traverse/retract band from Z−/A015 — already in band at Z≈0.
                ESP_LOGI(TAG,
                         "[BRINGUP] StopMotion hold Z=%.3f mm (0 = switch-clear; "
                         "SAFE_Z ceiling=%.1f); starting X home",
                         (float)currentZ_, traverseClearanceZMm());
                eipLimitAxis_ = EipLimitAxisRole::kX;
                eipLimitSawBusy_ = false;
                eipLimitPhaseStartMs_ = gantry_millis();
                if (eipMinWarningActive()) {
                    bringUpPhase_ = BringUpPhase::kXHomeCreep;
                } else {
                    bringUpPhase_ = BringUpPhase::kXHomeSeek;
                    ESP_LOGI(TAG, "[BRINGUP] seek -X A014 at %.1f mm/s a=%.0f",
                             eipSeekSpeedMmS(), eipHomeCalAccelMmS2());
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
                x->setCurrentPulses(0);
                x->stopMotion();
                bringUpPhase_ = BringUpPhase::kXHomeSettle;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
                ESP_LOGI(TAG,
                         "[BRINGUP] -X switch disabled — X=0 latched at this sample");
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
                currentX_mm_ = x->getCurrentMm();
                ESP_LOGI(TAG,
                         "[BRINGUP] StopMotion hold X=%.3f mm (0 = switch-clear); "
                         "seek +X A015",
                         currentX_mm_);
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
                const float accel = eipHomeCalAccelMmS2();
                if (!x->moveToMm(calXParkMm_, eipCalSeekSpeedMmS(), accel, accel)) {
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
                ESP_LOGI(TAG, "[BRINGUP] at X=%.3f (parked)", currentX_mm_);
                if (axisTheta_ && axisTheta_->hasLiveFeedback()) {
                    axisTheta_->captureSoftHome();
                    // Continuous slip-ring axis: take the equivalent of 90 deg
                    // nearest the current pose (<= 180 deg of travel).
                    const float orient_deg = ThetaOrigin::nearestEquivalentDeg(
                        axisTheta_->getCurrentDeg(), Constants::BRINGUP_THETA_ORIENT_DEG);
                    ESP_LOGI(TAG, "[BRINGUP] Orienting Theta to %.1f deg (== %.1f mod 360) "
                             "before Z+ calibration...",
                             static_cast<double>(orient_deg),
                             static_cast<double>(Constants::BRINGUP_THETA_ORIENT_DEG));
                    axisTheta_->moveToDeg(orient_deg, runtime_.home_theta_speed_deg_s, 0.0f, 0.0f);
                    thetaOrientDeg_ = orient_deg;
                    bringUpPhase_ = BringUpPhase::kThetaOrient;
                    eipLimitSawBusy_ = true;
                    eipLimitPhaseStartMs_ = gantry_millis();
                } else {
                    ESP_LOGI(TAG, "[BRINGUP] Seek +Z A014 for stroke (no Theta axis)");
                    eipLimitAxis_ = EipLimitAxisRole::kZ;
                    eipLimitSawBusy_ = false;
                    eipLimitPhaseStartMs_ = gantry_millis();
                    if (eipMaxWarningActive()) {
                        bringUpPhase_ = BringUpPhase::kZPlusCreep;
                    } else {
                        bringUpPhase_ = BringUpPhase::kZPlusSeek;
                    }
                }
            }
            break;
        }

        case BringUpPhase::kThetaOrient: {
            if (!axisTheta_) { failEipBringUp("no Theta"); return; }
            if (!axisTheta_->isBusy()) {
                currentTheta_ = static_cast<int32_t>(axisTheta_->getCurrentDeg());
                // A faulted or timed-out orient also goes idle. Do not run the
                // Z+ stroke with theta at the wrong angle.
                if (!thetaSettledAt(thetaOrientDeg_)) {
                    ESP_LOGE(TAG, "[BRINGUP] Theta orient missed: at %.3f deg, target %.3f deg, alarm=%d",
                             (double)axisTheta_->getCurrentDeg(), (double)thetaOrientDeg_,
                             axisTheta_->isAlarmActive() ? 1 : 0);
                    failEipBringUp("theta orient did not arrive (clear the theta alarm, then rerun)");
                    return;
                }
                ESP_LOGI(TAG, "[BRINGUP] Theta oriented at %.1f deg; seek +Z A014 for stroke",
                         (double)axisTheta_->getCurrentDeg());
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
                const float accel = eipHomeCalAccelMmS2();
                const float band = traverseClearanceZMm();
                if (std::fabs(max_mm - band) > 0.5f) {
                    if (!z->moveToMm(band, eipCalSeekSpeedMmS(), accel, accel)) {
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

} // namespace Gantry
