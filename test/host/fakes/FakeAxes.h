/**
 * @file FakeAxes.h
 * @brief Scriptable GantryLinearAxis / GantryRotaryAxis for host orchestration tests.
 */
#pragma once

#include "GantryLinearAxis.h"
#include "GantryRotaryAxis.h"
#include "host_test_clock.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace Gantry {
namespace Test {

/// Distance covered t seconds into a trapezoid (or triangle) move.
inline float trapezoidDistance(float t, float dist, float v, float a, float d) {
    if (dist <= 0.0f || t <= 0.0f) {
        return 0.0f;
    }
    if (v <= 0.0f) {
        return dist;
    }
    if (a <= 0.0f || d <= 0.0f) {
        return std::fmin(dist, v * t);
    }
    float peak = v;
    float da = v * v / (2.0f * a);
    float dd = v * v / (2.0f * d);
    if (da + dd > dist) {
        peak = std::sqrt(2.0f * dist / (1.0f / a + 1.0f / d));
        da = peak * peak / (2.0f * a);
        dd = peak * peak / (2.0f * d);
    }
    const float ta = peak / a;
    const float tc = (dist - da - dd) / peak;
    const float td = peak / d;
    if (t <= ta) {
        return 0.5f * a * t * t;
    }
    if (t <= ta + tc) {
        return da + peak * (t - ta);
    }
    if (t < ta + tc + td) {
        const float u = t - ta - tc;
        return da + peak * tc + peak * u - 0.5f * d * u * u;
    }
    return dist;
}

/// Trapezoid-timed motion on the host clock; advanced from update().
struct TimedMove {
    bool enabled = false;
    float start = 0.0f;
    float v = 0.0f;
    float a = 0.0f;
    float d = 0.0f;
    int64_t start_us = 0;

    void begin(float from, float speed, float accel, float decel) {
        start = from;
        v = speed;
        a = accel;
        d = decel;
        start_us = HostTest::nowUs();
    }
    /// Returns true once the target is reached.
    bool advance(float target, float& pos) const {
        const float dist = std::fabs(target - start);
        const float t = static_cast<float>(HostTest::nowUs() - start_us) * 1.0e-6f;
        const float s = trapezoidDistance(t, dist, v, a, d);
        if (s >= dist) {
            pos = target;
            return true;
        }
        pos = start + (target >= start ? s : -s);
        return false;
    }
};

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

    TimedMove timed;

    bool moveToMm(float target, float speed, float accel, float decel) override {
        if (reject_move) return false;
        moves.push_back({target, speed, accel, decel});
        target_mm = target;
        busy = std::fabs(target - mm) > 1.0e-4f;
        if (timed.enabled) {
            timed.begin(mm, speed, accel, decel);
        }
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

    bool moveToPulses(uint32_t target_pulses, uint32_t speed_pps, uint32_t accel_pps2,
                      uint32_t decel_pps2) override {
        const float tmm = static_cast<float>(target_pulses) / static_cast<float>(ppm);
        const float k = (ppm > 0.0) ? 1.0f / static_cast<float>(ppm) : 0.0f;
        return moveToMm(tmm, static_cast<float>(speed_pps) * k,
                        static_cast<float>(accel_pps2) * k, static_cast<float>(decel_pps2) * k);
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
    DrivePositionRef drive_ref = DrivePositionRef::kHomed;
    DrivePositionRef getDrivePositionRef() const override { return drive_ref; }
    void update() override {
        if (timed.enabled && busy && timed.advance(target_mm, mm)) {
            busy = false;
        }
    }
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

    TimedMove timed;

    bool moveToDeg(float target, float speed, float accel, float decel) override {
        if (std::fabs(target - deg) < 0.1f) {
            target_deg = target;
            busy = false;
            return false;
        }
        moves.push_back({target, speed, accel, decel});
        target_deg = target;
        busy = true;
        if (timed.enabled) {
            timed.begin(deg, speed, accel, decel);
        }
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
    DrivePositionRef drive_ref = DrivePositionRef::kHomed;
    DrivePositionRef getDrivePositionRef() const override { return drive_ref; }
    void update() override {
        if (timed.enabled && busy && timed.advance(target_deg, deg)) {
            busy = false;
        }
    }
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
