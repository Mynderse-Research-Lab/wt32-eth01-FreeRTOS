/**
 * @file FakeAxes.h
 * @brief Scriptable GantryLinearAxis / GantryRotaryAxis for host orchestration tests.
 */
#pragma once

#include "GantryLinearAxis.h"
#include "GantryRotaryAxis.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace Gantry {
namespace Test {

struct LinearMoveRecord {
    float target_mm = 0.0f;
    float speed_mm_s = 0.0f;
    float accel_mm_s2 = 0.0f;
    float decel_mm_s2 = 0.0f;
};

struct RotaryMoveRecord {
    float target_deg = 0.0f;
    float speed_deg_s = 0.0f;
    float accel_deg_s2 = 0.0f;
    float decel_deg_s2 = 0.0f;
};

class FakeLinearAxis : public GantryLinearAxis {
public:
    double ppm = 1000.0;
    bool begun = false;
    bool enabled = false;
    bool alarm = false;
    bool a014 = false;
    bool a015 = false;
    bool encoder_feedback = false;
    uint32_t homing_pps = 8000;
    const char* log_tag = nullptr;
    uint32_t log_rate_hz = 0;
    int attach_count = 0;
    bool joint_min_a015 = false;

    float mm = 0.0f;
    float target_mm = 0.0f;
    bool busy = false;
    uint32_t pulses_override_valid = 0;
    uint32_t pulses = 0;

    std::vector<LinearMoveRecord> moves;
    int stop_count = 0;

    bool begin() override {
        begun = true;
        return true;
    }
    bool enable() override {
        enabled = true;
        return true;
    }
    bool disable() override {
        enabled = false;
        return true;
    }
    bool isEnabled() const override { return enabled; }

    bool reject_move = false;

    bool moveToMm(float target, float speed, float accel, float decel) override {
        if (reject_move) return false;
        moves.push_back({target, speed, accel, decel});
        target_mm = target;
        busy = std::fabs(target - mm) > 1.0e-4f;
        return true;
    }
    bool moveRelativeMm(float delta, float speed, float accel, float decel) override {
        return moveToMm(mm + delta, speed, accel, decel);
    }
    bool stopMotion() override {
        ++stop_count;
        busy = false;
        target_mm = mm;
        return true;
    }
    float getCurrentMm() const override { return mm; }
    float getTargetMm() const override { return target_mm; }
    bool isBusy() const override { return busy; }

    bool moveToPulses(uint32_t target_pulses, uint32_t speed_pps, uint32_t,
                      uint32_t) override {
        const float tmm = static_cast<float>(target_pulses) / static_cast<float>(ppm);
        const float spd = (ppm > 0.0) ? static_cast<float>(speed_pps) / static_cast<float>(ppm)
                                      : 0.0f;
        return moveToMm(tmm, spd, 0.0f, 0.0f);
    }
    uint32_t getCurrentPulses() const override {
        return static_cast<uint32_t>(mm * ppm + (mm >= 0.0f ? 0.5f : -0.5f));
    }
    int32_t getEncoderPulses() const override {
        return static_cast<int32_t>(getCurrentPulses());
    }
    void setCurrentPulses(uint32_t pos) override {
        pulses = pos;
        mm = static_cast<float>(pos) / static_cast<float>(ppm);
        target_mm = mm;
        busy = false;
    }
    bool isMotionActive() const override { return busy; }

    bool isAlarmActive() const override { return alarm; }
    bool clearAlarm() override {
        alarm = false;
        return true;
    }

    bool isA014WarningActive() const override { return a014; }
    bool isA015WarningActive() const override { return a015; }
    void attachLimitSwitches(GantryLimitSwitch*, GantryLimitSwitch*) override {
        ++attach_count;
    }
    void setJointMinWarningA015(bool enable) override { joint_min_a015 = enable; }

    double pulsesPerMm() const override { return ppm; }
    bool isEncoderFeedbackEnabled() const override { return encoder_feedback; }
    void update() override {}
    uint32_t homingSpeedPps() const override { return homing_pps; }
    void setLogTag(const char* tag) override { log_tag = tag; }
    void setLogRateHz(uint32_t hz) override { log_rate_hz = hz; }

    void setMm(float v) { mm = v; }
    void completeMove() {
        mm = target_mm;
        busy = false;
    }
};

class FakeRotaryAxis : public GantryRotaryAxis {
public:
    double ppd = 10000.0;
    bool begun = false;
    bool enabled = false;
    bool alarm = false;
    bool live_fb = true;
    bool capture_ok = true;
    const char* log_tag = nullptr;
    uint32_t log_rate_hz = 0;
    float min_deg = -180.0f;
    float max_deg = 180.0f;

    float deg = 0.0f;
    float target_deg = 0.0f;
    bool busy = false;
    int stop_count = 0;
    int capture_count = 0;
    std::vector<RotaryMoveRecord> moves;

    bool begin() override {
        begun = true;
        return true;
    }
    bool enable() override {
        enabled = true;
        return true;
    }
    bool disable() override {
        enabled = false;
        return true;
    }
    bool isEnabled() const override { return enabled; }

    bool moveToDeg(float target, float speed, float accel, float decel) override {
        if (std::fabs(target - deg) <= 0.5f) {
            target_deg = target;
            busy = false;
            return false;
        }
        moves.push_back({target, speed, accel, decel});
        target_deg = target;
        busy = true;
        return true;
    }
    bool stopMotion() override {
        ++stop_count;
        busy = false;
        target_deg = deg;
        return true;
    }
    float getCurrentDeg() const override { return deg; }
    float getTargetDeg() const override { return target_deg; }
    float getDriveAbsDeg() const override { return deg; }
    bool isDriveOriginAligned() const override { return capture_count > 0 && std::fabs(deg) <= 2.0f; }
    float getMinDeg() const override { return min_deg; }
    float getMaxDeg() const override { return max_deg; }
    bool isBusy() const override { return busy; }
    bool isMotionActive() const override { return busy; }

    bool isAlarmActive() const override { return alarm; }
    bool clearAlarm() override {
        alarm = false;
        return true;
    }

    double pulsesPerDeg() const override { return ppd; }
    bool captureSoftHome() override {
        ++capture_count;
        return capture_ok;
    }
    bool hasLiveFeedback() const override { return live_fb; }
    void update() override {}
    void setAngleRange(float min_d, float max_d) override {
        min_deg = min_d;
        max_deg = max_d;
    }
    void setLogTag(const char* tag) override { log_tag = tag; }
    void setLogRateHz(uint32_t hz) override { log_rate_hz = hz; }

    void setDeg(float v) { deg = v; }
    void completeMove() {
        deg = target_deg;
        busy = false;
    }
};

}  // namespace Test
}  // namespace Gantry
