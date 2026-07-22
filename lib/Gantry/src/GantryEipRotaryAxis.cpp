/**
 * @file GantryEipRotaryAxis.cpp
 */

#include "GantryEipRotaryAxis.h"

#include "Hcs01Assembly.h"
#include "Hcs01ControlStatus.h"

#include <cmath>

namespace Gantry {

GantryEipRotaryAxis::GantryEipRotaryAxis(eip::EipProcessImage& image,
                                         const EipRotaryAxisConfig& cfg)
    : image_(image),
      config_(cfg),
      initialized_(false),
      enabled_(false),
      stop_requested_(false),
      target_deg_(0.0f),
      min_deg_(-180.0f),
      max_deg_(180.0f),
      command_accept_level_(false),
      log_tag_(""),
      log_rate_hz_(0) {}

bool GantryEipRotaryAxis::begin() {
  initialized_ = true;
  eip::hcs01::Hcs01PositioningCommand idle;
  idle.control = eip::hcs01::Hcs01ControlWord::makeBusFailureSafe();
  return publishCommand(idle);
}

bool GantryEipRotaryAxis::enable() {
  if (!initialized_) return false;
  enabled_ = true;
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveEnable();
  return publishCommand(cmd);
}

bool GantryEipRotaryAxis::disable() {
  enabled_ = false;
  stop_requested_ = false;
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeBusFailureSafe();
  return publishCommand(cmd);
}

bool GantryEipRotaryAxis::isEnabled() const { return enabled_; }

int32_t GantryEipRotaryAxis::degToPuu(float deg) const {
  return static_cast<int32_t>(llround(static_cast<double>(deg) * config_.puu_per_deg));
}

float GantryEipRotaryAxis::puuToDeg(int32_t puu) const {
  if (config_.puu_per_deg <= 0.0) return 0.0f;
  return static_cast<float>(static_cast<double>(puu) / config_.puu_per_deg);
}

bool GantryEipRotaryAxis::publishCommand(
    const eip::hcs01::Hcs01PositioningCommand& cmd) {
  image_.setCommand(cmd.serialize());
  return true;
}

bool GantryEipRotaryAxis::readFeedback(
    eip::hcs01::Hcs01PositioningActual& out) const {
  eip::Bytes fb;
  if (!image_.getFeedback(fb)) return false;
  return out.deserialize(fb);
}

bool GantryEipRotaryAxis::moveToDeg(float target_deg, float speed_deg_per_s,
                                    float accel_deg_per_s2,
                                    float decel_deg_per_s2) {
  (void)accel_deg_per_s2;
  (void)decel_deg_per_s2;
  if (!initialized_ || !enabled_ || config_.puu_per_deg <= 0.0) return false;
  if (!image_.isOnline()) return false;

  if (target_deg < min_deg_) target_deg = min_deg_;
  if (target_deg > max_deg_) target_deg = max_deg_;
  target_deg_ = target_deg;
  stop_requested_ = false;

  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveEnable();
  command_accept_level_ = !command_accept_level_;
  cmd.control.command_value_accept = command_accept_level_;
  cmd.positioning_command_value = degToPuu(target_deg);
  if (speed_deg_per_s > 0.0f) {
    cmd.positioning_velocity =
        static_cast<int32_t>(llround(speed_deg_per_s * config_.puu_per_deg));
  } else {
    cmd.positioning_velocity = config_.default_velocity_puu;
  }
  return publishCommand(cmd);
}

bool GantryEipRotaryAxis::stopMotion() {
  stop_requested_ = true;
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveEnable();
  cmd.control.drive_halt = true;
  return publishCommand(cmd);
}

float GantryEipRotaryAxis::getCurrentDeg() const {
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return 0.0f;
  return puuToDeg(fb.position_feedback);
}

float GantryEipRotaryAxis::getTargetDeg() const { return target_deg_; }

bool GantryEipRotaryAxis::isBusy() const {
  if (stop_requested_) return false;
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return false;
  return !fb.status.command_value_reached;
}

bool GantryEipRotaryAxis::isMotionActive() const { return isBusy(); }

bool GantryEipRotaryAxis::isAlarmActive() const {
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return false;
  return fb.status.class1_error;
}

bool GantryEipRotaryAxis::clearAlarm() {
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveEnable();
  cmd.control.clear_errors = true;
  return publishCommand(cmd);
}

double GantryEipRotaryAxis::pulsesPerDeg() const { return config_.puu_per_deg; }

void GantryEipRotaryAxis::update() {
  if (image_.feedbackFresh()) {
    image_.consumeFeedbackFresh();
  }
}

void GantryEipRotaryAxis::setAngleRange(float min_deg, float max_deg) {
  if (min_deg < max_deg) {
    min_deg_ = min_deg;
    max_deg_ = max_deg;
  }
}

void GantryEipRotaryAxis::setLogTag(const char* tag) {
  log_tag_ = (tag != nullptr) ? tag : "";
}

void GantryEipRotaryAxis::setLogRateHz(uint32_t hz) { log_rate_hz_ = hz; }

}  // namespace Gantry
