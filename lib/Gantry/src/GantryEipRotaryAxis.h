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

#include <cstdint>

namespace Gantry {

struct EipRotaryAxisConfig {
  // Live HCS01 S-0-0051/S-0-0282 are 0.0001 deg increments (3600000 / rev).
  double puu_per_deg = 10000.0;
  int32_t default_velocity_puu = 100000;  // 10 deg/s at 10000 PUU/deg
  int32_t default_accel_puu = 1000000;
  int32_t default_decel_puu = 1000000;
};

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
  void finishMoveHold();
  const char* tag() const;
  void logAfStatus(const char* why, const eip::hcs01::Hcs01PositioningActual& fb) const;

  eip::EipProcessImage& image_;
  EipRotaryAxisConfig config_;
  bool initialized_;
  bool enabled_;
  bool stop_requested_;
  bool motion_active_;
  int32_t zero_puu_;
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
  uint8_t move_phase_ticks_;
  uint16_t arm_phase_ticks_;
  int32_t cmd_pos_puu_;
  int32_t cmd_vel_puu_;
  int32_t cmd_acc_puu_;
  int32_t cmd_dec_puu_;
  uint8_t stuck_log_div_;
  float start_deg_;
  bool in_pos_armed_;
  uint8_t arrival_stable_ticks_;
  uint16_t run_ticks_;
};

}  // namespace Gantry

#endif  // GANTRY_EIP_ROTARY_AXIS_H
