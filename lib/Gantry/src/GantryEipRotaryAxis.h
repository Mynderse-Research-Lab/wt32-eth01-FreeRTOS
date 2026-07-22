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

namespace Gantry {

struct EipRotaryAxisConfig {
  double puu_per_deg = 100.0;
  int32_t default_velocity_puu = 1000;
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

  double pulsesPerDeg() const override;
  void update() override;
  void setAngleRange(float min_deg, float max_deg) override;

  void setLogTag(const char* tag) override;
  void setLogRateHz(uint32_t hz) override;

 private:
  bool publishCommand(const eip::hcs01::Hcs01PositioningCommand& cmd);
  bool readFeedback(eip::hcs01::Hcs01PositioningActual& out) const;
  int32_t degToPuu(float deg) const;
  float puuToDeg(int32_t puu) const;

  eip::EipProcessImage& image_;
  EipRotaryAxisConfig config_;
  bool initialized_;
  bool enabled_;
  bool stop_requested_;
  float target_deg_;
  float min_deg_;
  float max_deg_;
  bool command_accept_level_;
  const char* log_tag_;
  uint32_t log_rate_hz_;
};

}  // namespace Gantry

#endif  // GANTRY_EIP_ROTARY_AXIS_H
