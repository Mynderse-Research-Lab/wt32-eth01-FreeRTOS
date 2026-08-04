/**
 * @file GantryEipLinearAxis.h
 * @brief Linear axis backed by EtherNet/IP Kinetix 5100 assemblies 104/154.
 *
 * Production moves: OperatingMode=Position, TravelMode=2, Absolute
 * (drive-profiled PTP). See docs/LOW_LEVEL_GANTRY_CONTROL.md.
 */

#ifndef GANTRY_EIP_LINEAR_AXIS_H
#define GANTRY_EIP_LINEAR_AXIS_H

#include "GantryLinearAxis.h"
#include "GantryLimitSwitch.h"
#include "EipProcessImage.h"
#include "Kinetix5100Assembly.h"

#include <cstddef>

namespace Gantry {

struct EipLinearAxisConfig {
  double puu_per_mm = 1000.0;
  // Kinetix speed/accel/decel refs are 0.1 RPM (or 0.1 RPM/s).
  // Prefer speed_ref_per_mm_s = 600.0 * ratio / lead_mm_per_rev.
  double speed_ref_per_mm_s = 10.0;
  double accel_ref_per_mm_s2 = 10.0;
  double decel_ref_per_mm_s2 = 10.0;
  double lead_mm_per_rev = 20.0;
};

class GantryEipLinearAxis : public GantryLinearAxis {
 public:
  GantryEipLinearAxis(eip::EipProcessImage& image, const EipLinearAxisConfig& cfg);

  bool begin() override;
  bool enable() override;
  bool disable() override;
  bool isEnabled() const override;

  bool moveToMm(float target_mm, float speed_mm_per_s, float accel_mm_per_s2,
                float decel_mm_per_s2) override;
  bool moveRelativeMm(float delta_mm, float speed_mm_per_s, float accel_mm_per_s2,
                      float decel_mm_per_s2) override;
  bool stopMotion() override;
  float getCurrentMm() const override;
  float getTargetMm() const override;
  bool isBusy() const override;

  bool moveToPulses(uint32_t target_pulses, uint32_t speed_pps, uint32_t accel_pps2,
                    uint32_t decel_pps2) override;
  uint32_t getCurrentPulses() const override;
  int32_t getEncoderPulses() const override;
  void setCurrentPulses(uint32_t pos) override;
  bool isMotionActive() const override;

  bool isAlarmActive() const override;
  bool clearAlarm() override;
  bool getDriveAlarmSummary(char* buf, size_t n) const override;
  uint16_t getDriveFaultCode() const override;
  uint16_t getDriveWarningCode() const override;
  /// True only while warning_present and code is A014/A015 (clear-edge safe).
  bool isA014WarningActive() const;
  bool isA015WarningActive() const;

  /// True when the drive reports motion stopped (bit6) or actual speed is
  /// near zero. Use after hold-here / settle to verify the axis is physically
  /// stationary before declaring sequence complete.
  bool isAxisPhysicallyStopped() const;

  double pulsesPerMm() const override;
  bool isEncoderFeedbackEnabled() const override;

  void update() override;
  uint32_t homingSpeedPps() const override;

  void setLogTag(const char* tag) override;
  void setLogRateHz(uint32_t hz) override;

  /// Cancel Absolute by StartMotion to the live feedback position (stay OM=1).
  /// Prefer this over OM=0 StopMotion mid-creep — avoids A603 + coast with busy=0.
  bool cancelAbsoluteToFeedback();

  /// StopMotion + feedback wait: enter abort stop, then block until measured
  /// encoder position holds still for 200 ms. The drive's stopped bit and speed
  /// word both read "stopped" during a 1 mm/s creep, so position delta is the
  /// only reliable gate. Returns true on success, false on timeout (≈3 s);
  /// caller decides fallback. Blocks in the GantryUpdate task (one-off per
  /// home/cal clear edge).
  bool stopAndWaitForPhysicalStop();

  /// Optional: sync drive overtravel heuristic into GantryLimitSwitch objects.
  void attachLimitSwitches(GantryLimitSwitch* min_sw, GantryLimitSwitch* max_sw);

 private:
  bool publishCommand(const eip::k5100::OutputAssembly104& cmd);
  bool readFeedback(eip::k5100::InputAssembly154& out) const;
  int32_t mmToPuu(float mm) const;
  float puuToMm(int32_t puu) const;
  int32_t toJointPuu(int32_t abs_puu) const;
  int32_t toAbsPuu(int32_t joint_puu) const;
  void logAlarmEdge(const eip::k5100::InputAssembly154& fb);

  eip::EipProcessImage& image_;
  EipLinearAxisConfig config_;
  bool initialized_;
  bool enabled_;
  bool stop_requested_;
  float target_mm_;
  int32_t zero_puu_;
  int32_t command_position_puu_;
  bool start_motion_level_;
  const char* log_tag_;
  uint32_t log_rate_hz_;
  bool last_fault_latched_;
  bool last_warning_latched_;
  uint16_t last_fault_code_;
  uint16_t last_warning_code_;
  bool motion_commanded_;
  bool drive_homed_;

  // ServoOn edge: settle OM=0 TM=10 (never Position before Active — A603).
  // Then HomingMethod=34 if HomedStatus clear (unlock Absolute / clear E237).
  enum class ArmPhase : uint8_t {
    kIdle = 0,
    kServoLow,
    kServoSettle,
    kHomePreload,
    kHomeStart,
    kHomeWait,
    kHolding
  };
  ArmPhase arm_phase_;
  uint8_t arm_phase_ticks_;
  uint8_t a603_clear_attempts_;
  uint16_t home_wait_ticks_;

  // Position Absolute PTP (OM=1 TM=2 NonCyclic=0). Done on AtReference.
  enum class MovePhase : uint8_t {
    kIdle = 0,
    kPreload,
    kStart,
    kRun,
    kStopping
  };
  MovePhase move_phase_;
  uint8_t move_phase_ticks_;
  uint8_t fault_reset_pulse_ticks_;
  uint8_t stop_motion_pulse_ticks_;
  float last_speed_mm_s_;
  float last_accel_mm_s2_;
  float last_decel_mm_s2_;
  int32_t move_speed_ref_;  // unsigned cruise magnitude (0.1 RPM)
  uint16_t run_ticks_;
  uint8_t stop_stable_ticks_;
  int32_t last_stop_sample_puu_;
  bool stop_sample_valid_;

  GantryLimitSwitch* limit_min_sw_ = nullptr;
  GantryLimitSwitch* limit_max_sw_ = nullptr;
  // When starting a move already on A014/A015 (escape/creep), do not abort on
  // that same warning until it clears — otherwise back-off never runs.
  bool ignore_a014_abort_ = false;
  bool ignore_a015_abort_ = false;

  eip::k5100::OutputAssembly104 buildSettleCommand(bool servo_on) const;
  eip::k5100::OutputAssembly104 buildHomeCommand(bool start_motion_edge) const;
  eip::k5100::OutputAssembly104 buildMoveCommand(bool start_motion_edge) const;
  eip::k5100::OutputAssembly104 buildHoldCommand() const;
  eip::k5100::OutputAssembly104 buildStopCommand(bool stop_motion) const;
  void republishMoveCommand(bool start_motion_edge);
  void enterAbortStop();
  void finishMoveHold();
  void advanceArmPhase();
  void advanceMovePhase(const eip::k5100::InputAssembly154& fb);
  bool inPositionBand(const eip::k5100::InputAssembly154& fb) const;
};

}  // namespace Gantry

#endif  // GANTRY_EIP_LINEAR_AXIS_H
