/**
 * @file GantryEipRotaryAxis.h
 * @brief Rotary axis backed by EtherNet/IP HCS01 assemblies 101/102.
 *
 * Theta axis over IndraDrive Multi-Ethernet. Shares an EipProcessImage with
 * the scanner task for cyclic I/O.
 */

#ifndef GANTRY_EIP_ROTARY_AXIS_H
#define GANTRY_EIP_ROTARY_AXIS_H

#include "GantryRotaryAxis.h"
#include "EipProcessImage.h"
#include "Hcs01Assembly.h"

#include <atomic>
#include <cmath>
#include <cstdint>

namespace Gantry {

// Drive scaling from the HCS01 export (tools/Parameter_default_1.par, 2026-10-13):
//   S-0-0044 = 0b0010, S-0-0045 = 1, S-0-0046 = -4: velocity 0.0001 rpm, motor side.
//   S-0-0160 = 0b0010, S-0-0161 = 1, S-0-0162 = -3: acceleration 0.001 rad/s2.
//   S-0-0076 = 0b0010, S-0-0079 = 3600000: position 0.0001 deg (10000 PUU/deg).
// ERD04 is direct drive, so motor rpm = theta rpm.
constexpr double kHcs01VelocityUnitsPerRpm = 10000.0;
constexpr double kHcs01VelocityUnitsPerDegS = kHcs01VelocityUnitsPerRpm * 60.0 / 360.0;
// S-0-0259 cap: S-0-0113 maximum motor speed, 600 rpm (3600 deg/s).
constexpr double kHcs01MaxVelocityUnits = 600.0 * kHcs01VelocityUnitsPerRpm;
// S-0-0260 / S-0-0359 cap: S-0-0138 acceleration limit, 1000 rad/s2.
constexpr double kHcs01MaxAccelUnits = 1000000.0;
// Move timeout: never below this, and twice the expected profile time beyond it.
constexpr uint32_t kThetaMoveTimeoutMinUs = 15000000u;
constexpr double kThetaMoveTimeoutScale = 2.0;
constexpr double kThetaMoveTimeoutMaxUs = 4.0e9;

/// deg/s -> S-0-0259 units (0.0001 rpm; 1 deg/s = 1666.67). Independent of PUU/deg.
inline int32_t hcs01VelocityUnits(double deg_per_s) {
    double v = deg_per_s * kHcs01VelocityUnitsPerDegS;
    if (v < 0.0) {
        v = 0.0;
    }
    if (v > kHcs01MaxVelocityUnits) {
        v = kHcs01MaxVelocityUnits;
    }
    return static_cast<int32_t>(std::llround(v));
}

/// deg/s2 -> S-0-0260 / S-0-0359 units (0.001 rad/s2). Independent of PUU/deg.
inline int32_t hcs01AccelUnits(double deg_per_s2) {
    double a = deg_per_s2 * (1000.0 * 3.141592653589793 / 180.0);
    if (a < 0.0) {
        a = 0.0;
    }
    if (a > kHcs01MaxAccelUnits) {
        a = kHcs01MaxAccelUnits;
    }
    return static_cast<int32_t>(std::llround(a));
}

/// Timeout for one theta move: the 15 s floor, or twice distance / speed for
/// long slow moves (e.g. a 300 deg return at the 15 deg/s home speed).
inline uint32_t thetaMoveTimeoutUs(double distance_deg, double speed_deg_per_s) {
    if (speed_deg_per_s <= 0.0) {
        return kThetaMoveTimeoutMinUs;
    }
    const double expected_us = std::fabs(distance_deg) / speed_deg_per_s * 1.0e6;
    double timeout_us = expected_us * kThetaMoveTimeoutScale;
    if (timeout_us < static_cast<double>(kThetaMoveTimeoutMinUs)) {
        return kThetaMoveTimeoutMinUs;
    }
    if (timeout_us > kThetaMoveTimeoutMaxUs) {
        timeout_us = kThetaMoveTimeoutMaxUs;
    }
    return static_cast<uint32_t>(timeout_us);
}

struct EipRotaryAxisConfig {
    // Live HCS01 S-0-0051/S-0-0282 are 0.0001 deg increments (3600000 / rev).
    double puu_per_deg = 10000.0;
    // S-0-0259 is 0.0001 rpm (see kHcs01VelocityUnitsPerRpm).
    int32_t default_velocity_puu = 16667;  // 10 deg/s = 1.6667 rpm
    // S-0-0260 / S-0-0359 are 0.001 rad/s2.
    int32_t default_accel_puu = 31416;     // 1800 deg/s2 = 31.416 rad/s2
    int32_t default_decel_puu = 31416;     // 1800 deg/s2 = 31.416 rad/s2
};

/// Axis config from deg-based limits, with the defaults in drive units.
inline EipRotaryAxisConfig eipRotaryAxisConfigFromDeg(double puu_per_deg,
                                                      double max_speed_deg_per_s,
                                                      double accel_deg_per_s2,
                                                      double decel_deg_per_s2) {
    EipRotaryAxisConfig cfg;
    cfg.puu_per_deg = puu_per_deg;
    cfg.default_velocity_puu = hcs01VelocityUnits(max_speed_deg_per_s);
    cfg.default_accel_puu = hcs01AccelUnits(accel_deg_per_s2);
    cfg.default_decel_puu = hcs01AccelUnits(decel_deg_per_s2);
    return cfg;
}

class GantryEipRotaryAxis : public GantryRotaryAxis {
 public:
  GantryEipRotaryAxis(eip::EipProcessImage& image, const EipRotaryAxisConfig& cfg);

  bool begin() override;
  bool enable() override;
  bool disable() override;
  bool isEnabled() const override;

  bool moveToDeg(float target_deg, float speed_deg_per_s, float accel_deg_per_s2,
                 float decel_deg_per_s2) override;
  bool stopMotion() override;
  float getCurrentDeg() const override;
  float getTargetDeg() const override;
  bool isBusy() const override;
  bool isMotionActive() const override;

  bool isAlarmActive() const override;
  bool clearAlarm() override;
  bool isLinkOnline() const override { return image_.isOnline(); }
  bool isServoActive() const override;
  bool getDriveAlarmSummary(char* buf, size_t n) const override;
  bool formatCipStatus(char* buf, size_t n) const override;

  double pulsesPerDeg() const override;
  bool setPuuPerDeg(double puu_per_deg) override;
  bool captureSoftHome() override;
  bool hasLiveFeedback() const override;
  float getDriveAbsDeg() const override;
  bool isDriveOriginAligned() const override;
  float getMinDeg() const override { return min_deg_; }
  float getMaxDeg() const override { return max_deg_; }
  void update() override;
  void setAngleRange(float min_deg, float max_deg) override;

  void setLogTag(const char* tag) override;
  void setLogRateHz(uint32_t hz) override;
  DrivePositionRef getDrivePositionRef() const override;
  bool sampleTrace(RotaryTraceSample& out) const override;
  bool getMoveStamp(RotaryMoveStamp& out) const override;

 private:
  enum class MovePhase : uint8_t {
    kIdle,
    kPreload,
    kReleaseHalt,
    kStart,
    kRun
  };

  bool publishCommand(const eip::hcs01::Hcs01PositioningCommand& cmd);
  bool readFeedback(eip::hcs01::Hcs01PositioningActual& out) const;
  int32_t degToPuu(float deg) const;
  float puuToDeg(int32_t puu) const;
  int32_t feedbackAbsPuu() const;
  eip::hcs01::Hcs01PositioningCommand buildMoveCommand(bool start_edge) const;
  bool publishHold();
  bool publishClearFaults();
  bool publishMove(bool start_edge);
  void finishMoveHold(RotaryMoveEnd end);
  void stampMove();
  void stampEnd(RotaryMoveEnd end);
  const char* tag() const;
  void logAfStatus(const char* why, const eip::hcs01::Hcs01PositioningActual& fb) const;

  eip::EipProcessImage& image_;
  EipRotaryAxisConfig config_;
  bool initialized_;
  bool enabled_;
  bool stop_requested_;
  bool motion_active_;
  float target_deg_;
  float min_deg_;
  float max_deg_;
  float envelope_min_deg_;
  float envelope_max_deg_;
  bool origin_aligned_;
  bool command_accept_level_;
  const char* log_tag_;
  uint32_t log_rate_hz_;
  int64_t last_axislog_us_;
  MovePhase move_phase_;
  enum class ArmPhase : uint8_t { kIdle, kClearFaults, kWaitAf, kHolding };
  ArmPhase arm_phase_;
  uint32_t move_phase_us_;
  uint32_t arm_phase_us_;
  int32_t cmd_pos_puu_;
  int32_t cmd_vel_puu_;
  int32_t cmd_acc_puu_;
  int32_t cmd_dec_puu_;
  uint8_t stuck_log_div_;
  float start_deg_;
  float prev_deg_ = 0.0f;
  bool in_pos_armed_;
  uint32_t arrival_stable_us_;
  uint32_t stiction_stable_us_ = 0;
  uint32_t run_us_;
  uint32_t move_timeout_us_ = kThetaMoveTimeoutMinUs;
  // Move stamp for the theta trace, read lock-free from another task.
  // stamp_seq_ is odd while stampMove() rewrites the move fields.
  std::atomic<uint32_t> stamp_seq_{0};
  std::atomic<int64_t> stamp_cmd_us_{0};
  std::atomic<int64_t> stamp_start_us_{0};
  std::atomic<int64_t> stamp_done_us_{0};
  std::atomic<uint8_t> stamp_end_{0};
  std::atomic<float> stamp_start_deg_{0.0f};
  std::atomic<float> stamp_target_deg_{0.0f};
  std::atomic<float> stamp_vel_deg_s_{0.0f};
  std::atomic<float> stamp_acc_deg_s2_{0.0f};
  std::atomic<float> stamp_dec_deg_s2_{0.0f};
};

}  // namespace Gantry

#endif  // GANTRY_EIP_ROTARY_AXIS_H
