/**
 * @file GantryEipHomeCal.cpp
 * @brief Gantry class: EtherNet/IP precision home and calibrate sequence.
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
// EIP LIMIT HOME / CALIBRATE SEQUENCE
// ============================================================================

GantryLinearAxis* Gantry::eipLimitActiveAxis() {
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return axisZ_.get();
    }
    return axisX_.get();
}

const GantryLinearAxis* Gantry::eipLimitActiveAxis() const {
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return axisZ_.get();
    }
    return axisX_.get();
}

bool Gantry::eipMinWarningActive() const {
    const GantryLinearAxis* axis = eipLimitActiveAxis();
    if (axis == nullptr) return false;
    // X: A014 = joint min. Z: A015 = joint min (−Z).
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return axis->isA015WarningActive();
    }
    return axis->isA014WarningActive();
}

bool Gantry::eipMaxWarningActive() const {
    const GantryLinearAxis* axis = eipLimitActiveAxis();
    if (axis == nullptr) return false;
    // X: A015 = joint max. Z: A014 = joint max (+Z).
    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        return axis->isA014WarningActive();
    }
    return axis->isA015WarningActive();
}

float Gantry::eipSeekSpeedMmS() const {
    return runtime_.home_speed_mm_s;
}

float Gantry::eipCreepSpeedMmS() const {
    return runtime_.creep_speed_mm_s;
}

float Gantry::eipCalSeekSpeedMmS() const {
    return runtime_.home_speed_mm_s;
}

float Gantry::eipHomeCalAccelMmS2() const {
    return runtime_.home_accel_mm_s2;
}

bool Gantry::eipStartMoveDelta(float delta_mm, float speed_mm_s) {
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) return false;
    if (eipLimitAxis_ == EipLimitAxisRole::kX &&
        !requireXTraverseInterlock("EIP X Absolute")) {
        return false;
    }
    const float accel = eipHomeCalAccelMmS2();
    const float here = axis->getCurrentMm();
    return axis->moveToMm(here + delta_mm, speed_mm_s, accel, accel);
}

void Gantry::startEipPrecisionHome(EipLimitAxisRole role) {
    if (role == EipLimitAxisRole::kX &&
        !requireXTraverseInterlock("EIP home X")) {
        homingInProgress_ = false;
        eipLimitPhase_ = EipLimitPhase::kIdle;
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    eipLimitAxis_ = role;
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    abortRequested_ = false;
    homingInProgress_ = true;
    eipLimitSawBusy_ = false;
    eipLimitPhaseStartMs_ = gantry_millis();
    const char* ax = EipLimit::axisLetter(role);
    const char* minCode = EipLimit::minWarningLabel(role);

    if (eipMinWarningActive()) {
        ESP_LOGI(TAG,
                 "[HOME] EIP already on -%s limit %s - creep toward +%s until clear",
                 ax, minCode, ax);
        eipLimitPhase_ = EipLimitPhase::kHomeCreepClear;
        return;
    }

    ESP_LOGI(TAG, "[HOME] EIP seek -%s limit %s at %.1f mm/s a=%.0f",
             ax, minCode, eipSeekSpeedMmS(), eipHomeCalAccelMmS2());
    eipLimitPhase_ = EipLimitPhase::kHomeSeekMin;
}

void Gantry::startEipPrecisionCalibrate(EipLimitAxisRole role) {
    if (role == EipLimitAxisRole::kX &&
        !requireXTraverseInterlock("EIP calibrate X")) {
        calibrationInProgress_ = false;
        eipLimitPhase_ = EipLimitPhase::kIdle;
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    eipLimitAxis_ = role;
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        eipLimitAxis_ = EipLimitAxisRole::kNone;
        return;
    }
    abortRequested_ = false;
    calibrationInProgress_ = true;
    eipLimitSawBusy_ = false;
    eipLimitPhaseStartMs_ = gantry_millis();
    const char* ax = EipLimit::axisLetter(role);
    const char* maxCode = EipLimit::maxWarningLabel(role);

    if (eipMaxWarningActive()) {
        ESP_LOGI(TAG,
                 "[CAL] EIP already on +%s limit %s - creep toward -%s until clear",
                 ax, maxCode, ax);
        eipLimitPhase_ = EipLimitPhase::kCalCreepClear;
        return;
    }

    ESP_LOGI(TAG, "[CAL] EIP seek +%s limit %s at %.1f mm/s a=%.0f",
             ax, maxCode, eipCalSeekSpeedMmS(), eipHomeCalAccelMmS2());
    eipLimitPhase_ = EipLimitPhase::kCalSeekMax;
}

void Gantry::finishEipHomeOk() {
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        failEipLimitSequence("no active axis");
        return;
    }
    axis->setCurrentPulses(0);
    axis->stopMotion();
    eipLimitPhase_ = EipLimitPhase::kHomeSettle;
    eipLimitPhaseStartMs_ = gantry_millis();
    eipLimitSawBusy_ = false;
    const char* ax = EipLimit::axisLetter(eipLimitAxis_);
    ESP_LOGI(TAG,
             "[HOME] EIP -%s switch disabled — joint 0 latched at this sample",
             ax);
}

void Gantry::finishEipCalOk() {
    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        failEipLimitSequence("no active axis");
        return;
    }
    axis->stopMotion();
    const float max_mm = axis->getCurrentMm();
    const int32_t length = static_cast<int32_t>(llround(max_mm));
    const char* ax = EipLimit::axisLetter(eipLimitAxis_);

    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
        zAxisLength_ = length < 1 ? 1 : length;
        config_.limits.z_max = static_cast<float>(zAxisLength_);
        if (config_.limits.z_max < config_.limits.z_min + 1.0f) {
            config_.limits.z_max = config_.limits.z_min + 1.0f;
        }
        currentZ_ = (int32_t)max_mm;
        ESP_LOGI(TAG,
                 "[CAL] EIP +Z limit %s cleared - joint max ~ %.3f mm (%ld); "
                 "hold-here then return to 0",
                 EipLimit::maxWarningLabel(EipLimitAxisRole::kZ), max_mm,
                 (long)zAxisLength_);
    } else {
        axisLength_ = length < 1 ? 1 : length;
        currentX_mm_ = max_mm;
        ESP_LOGI(TAG,
                 "[CAL] EIP +X limit %s cleared - joint max ~ %.3f mm (%ld); "
                 "hold-here then return to 0",
                 EipLimit::maxWarningLabel(EipLimitAxisRole::kX), max_mm,
                 (long)axisLength_);
    }
    eipLimitPhase_ = EipLimitPhase::kCalReturnZero;
    eipLimitPhaseStartMs_ = gantry_millis();
    calibrationInProgress_ = true;
    eipLimitSawBusy_ = false;
    (void)ax;
}

void Gantry::failEipLimitSequence(const char* why) {
    ESP_LOGE(TAG, "[EIP-LIMIT] failed: %s", why ? why : "?");
    if (GantryLinearAxis* axis = eipLimitActiveAxis()) {
        axis->stopMotion();
    }
    eipLimitPhase_ = EipLimitPhase::kIdle;
    eipLimitAxis_ = EipLimitAxisRole::kNone;
    eipLimitSawBusy_ = false;
    homingInProgress_ = false;
    calibrationInProgress_ = false;
}

void Gantry::advanceEipLimitSequence() {
    if (abortRequested_) {
        failEipLimitSequence("abort requested");
        return;
    }
    constexpr unsigned long kPhaseTimeoutMs = TRAVEL_MEASUREMENT_TIMEOUT_MS;
    if ((gantry_millis() - eipLimitPhaseStartMs_) > kPhaseTimeoutMs) {
        failEipLimitSequence("phase timeout");
        return;
    }

    GantryLinearAxis* axis = eipLimitActiveAxis();
    if (!axis) {
        failEipLimitSequence("no active axis");
        return;
    }
    const char* ax = EipLimit::axisLetter(eipLimitAxis_);

    switch (eipLimitPhase_) {
        case EipLimitPhase::kHomeSeekMin: {
            using EipLimit::SeekOutcome;
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMinWarningActive(), axis->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                ESP_LOGI(TAG,
                         "[HOME] -%s limit tripped - will creep toward +%s @ %.1f mm/s",
                         ax, ax, eipCreepSpeedMmS());
                eipLimitPhase_ = EipLimitPhase::kHomeCreepClear;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && axis->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!eipStartMoveDelta(EipLimit::homeSeekDeltaMm(),
                                       eipSeekSpeedMmS())) {
                    failEipLimitSequence("seek min limit start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipLimitSequence(
                    eipLimitAxis_ == EipLimitAxisRole::kZ
                        ? "seek -Z ended without trip (check A015/NL at joint -Z)"
                        : "seek -X ended without trip (check A014/PL at joint -X)");
            }
            break;
        }

        case EipLimitPhase::kHomeCreepClear: {
            using EipLimit::CreepOutcome;
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMinWarningActive(), axis->isBusy());
            if (o == CreepOutcome::kCleared) {
                finishEipHomeOk();
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!eipStartMoveDelta(EipLimit::homeCreepDeltaMm(),
                                          eipCreepSpeedMmS())) {
                failEipLimitSequence("home creep start failed");
            }
            break;
        }

        case EipLimitPhase::kHomeSettle: {
            if (!axis->isBusy()) {
                if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
                    currentZ_ = (int32_t)axis->getCurrentMm();
                    ESP_LOGI(TAG,
                             "[HOME] EIP StopMotion hold Z=%.3f mm (0 = switch-clear)",
                             (float)currentZ_);
                } else {
                    currentX_mm_ = axis->getCurrentMm();
                    ESP_LOGI(TAG,
                             "[HOME] EIP StopMotion hold X=%.3f mm (0 = switch-clear)",
                             currentX_mm_);
                }
                homingInProgress_ = false;
                if (calibrationInProgress_) {
                    const char* maxCode = EipLimit::maxWarningLabel(eipLimitAxis_);
                    ESP_LOGI(TAG, "[CAL] EIP seek +%s limit %s at %.1f mm/s a=%.0f",
                             ax, maxCode, eipCalSeekSpeedMmS(),
                             eipHomeCalAccelMmS2());
                    eipLimitPhase_ = EipLimitPhase::kCalSeekMax;
                    eipLimitPhaseStartMs_ = gantry_millis();
                    eipLimitSawBusy_ = false;
                } else {
                    eipLimitPhase_ = EipLimitPhase::kIdle;
                    eipLimitAxis_ = EipLimitAxisRole::kNone;
                    ESP_LOGI(TAG, "[HOME] EIP complete");
                }
            }
            break;
        }

        case EipLimitPhase::kCalSeekMax: {
            using EipLimit::SeekOutcome;
            const SeekOutcome o = EipLimit::evaluateSeek(
                eipMaxWarningActive(), axis->isBusy(), eipLimitSawBusy_);
            if (o == SeekOutcome::kTripped) {
                ESP_LOGI(TAG,
                         "[CAL] +%s limit tripped - will creep toward -%s @ %.1f mm/s",
                         ax, ax, eipCreepSpeedMmS());
                eipLimitPhase_ = EipLimitPhase::kCalCreepClear;
                eipLimitPhaseStartMs_ = gantry_millis();
                eipLimitSawBusy_ = false;
            } else if (o == SeekOutcome::kWait && axis->isBusy()) {
                eipLimitSawBusy_ = true;
            } else if (o == SeekOutcome::kStartMove) {
                if (!eipStartMoveDelta(EipLimit::calSeekDeltaMm(),
                                       eipCalSeekSpeedMmS())) {
                    failEipLimitSequence("seek max limit start failed");
                }
            } else if (o == SeekOutcome::kFailNoTrip) {
                failEipLimitSequence(
                    eipLimitAxis_ == EipLimitAxisRole::kZ
                        ? "seek +Z ended without trip (check A014/PL at joint +Z)"
                        : "seek +X ended without trip (check A015/NL at joint +X)");
            }
            break;
        }

        case EipLimitPhase::kCalCreepClear: {
            using EipLimit::CreepOutcome;
            const CreepOutcome o =
                EipLimit::evaluateCreep(eipMaxWarningActive(), axis->isBusy());
            if (o == CreepOutcome::kCleared) {
                finishEipCalOk();
            } else if (o == CreepOutcome::kWait) {
                eipLimitSawBusy_ = true;
            } else if (!eipStartMoveDelta(EipLimit::calCreepDeltaMm(),
                                          eipCreepSpeedMmS())) {
                failEipLimitSequence("cal creep start failed");
            }
            break;
        }

        case EipLimitPhase::kCalReturnZero: {
            if (!axis->isBusy()) {
                if (!eipLimitSawBusy_) {
                    if (std::fabs(axis->getCurrentMm()) > 0.5f) {
                        const float accel = eipHomeCalAccelMmS2();
                        if (!axis->moveToMm(0.0f, eipCalSeekSpeedMmS(), accel,
                                            accel)) {
                            failEipLimitSequence("return to 0 start failed");
                            break;
                        }
                        eipLimitSawBusy_ = true;
                        eipLimitPhaseStartMs_ = gantry_millis();
                    } else {
                        if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
                            currentZ_ = (int32_t)axis->getCurrentMm();
                            ESP_LOGI(TAG,
                                     "[CAL] EIP complete - at joint 0 (Z=%.3f mm)",
                                     (float)currentZ_);
                        } else {
                            currentX_mm_ = axis->getCurrentMm();
                            ESP_LOGI(TAG,
                                     "[CAL] EIP complete - at joint 0 (%.3f mm)",
                                     currentX_mm_);
                        }
                        eipLimitPhase_ = EipLimitPhase::kIdle;
                        eipLimitAxis_ = EipLimitAxisRole::kNone;
                        calibrationInProgress_ = false;
                        tryLatchWorkspaceFromStrokes();
                    }
                } else {
                    eipLimitSawBusy_ = false;
                    if (eipLimitAxis_ == EipLimitAxisRole::kZ) {
                        currentZ_ = (int32_t)axis->getCurrentMm();
                        ESP_LOGI(TAG,
                                 "[CAL] EIP complete - at joint 0 (Z=%.3f mm)",
                                 (float)currentZ_);
                    } else {
                        currentX_mm_ = axis->getCurrentMm();
                        ESP_LOGI(TAG,
                                 "[CAL] EIP complete - at joint 0 (%.3f mm)",
                                 currentX_mm_);
                    }
                    eipLimitPhase_ = EipLimitPhase::kIdle;
                    eipLimitAxis_ = EipLimitAxisRole::kNone;
                    calibrationInProgress_ = false;
                    tryLatchWorkspaceFromStrokes();
                }
            }
            break;
        }

        case EipLimitPhase::kIdle:
        default:
            break;
    }
}

} // namespace Gantry
