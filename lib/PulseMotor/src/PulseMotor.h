/**
 * @file PulseMotor.h
 * @brief Generic pulse-train motor driver for ESP32 (LEDC pulse output + PCNT encoder feedback)
 * @version 2.0.0
 *
 * FEATURES:
 * - Pulse/Direction position control with deterministic trapezoidal profile.
 * - Optional open-loop or closed-loop control using hardware encoder feedback.
 * - Status monitoring (position, speed, alarm, in-position, brake).
 * - Thread-safe (optional FreeRTOS mutex).
 *
 * LIMITATIONS:
 * - Only supports the PULSE_DIRECTION pulse mode today.
 */

#ifndef PULSE_MOTOR_H
#define PULSE_MOTOR_H

#ifndef HOME_ON_BOOT
#define HOME_ON_BOOT 1
#endif

#include <Arduino.h>
#include <driver/pulse_cnt.h>
#include <esp_timer.h>

#ifndef PULSE_MOTOR_USE_FREERTOS
#define PULSE_MOTOR_USE_FREERTOS 1
#endif

#if PULSE_MOTOR_USE_FREERTOS
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

namespace PulseMotor {

/**
 * @brief Pulse input mode selection at the driver.
 */
enum class PulseMode : uint8_t {
  PULSE_DIRECTION = 0,
  CW_CCW = 1,
  QUADRATURE = 2
};

/**
 * @brief Mechanical drivetrain category.
 *
 * Used by the higher-level Gantry/Axis layer for mm/deg <-> pulse conversion.
 * The driver itself operates strictly in pulse units.
 */
enum class DrivetrainType : uint8_t {
  BALLSCREW = 1,
  BELT = 2,
  RACKPINION = 3,
  ROTARY_DIRECT = 4
};

/**
 * @brief Mechanical parameters coupled to a pulse-motor output.
 */
struct DrivetrainConfig {
  DrivetrainType type = DrivetrainType::BALLSCREW;
  float lead_mm = 40.0f;
  float belt_lead_mm_per_rev = 40.0f;
  float pinion_pitch_diameter_mm = 20.0f;
  float output_gear_ratio = 1.0f;
  float encoder_ppr = 6000.0f;
  float motor_reducer_ratio = 1.0f;
};

double pulsesPerMm(const DrivetrainConfig &config);
double pulsesPerDeg(const DrivetrainConfig &config);

/**
 * @brief Electrical configuration of the pulse-train motor driver.
 */
struct DriverConfig {
  // ---- Named GPIO assignments ----------------------------------------------
  int pulse_pin;
  int dir_pin;
  int enable_pin;
  int alarm_reset_pin;
  int alarm_pin;
  int in_position_pin;
  int brake_pin;
  int encoder_a_pin;
  int encoder_b_pin;
  int encoder_z_pin;
  int limit_min_pin;
  int limit_max_pin;

  // ---- Pulse and encoder tuning --------------------------------------------
  PulseMode pulse_mode;
  uint32_t max_pulse_freq;
  uint32_t encoder_ppr;

  // Driver-side electronic gear (consumer-facing only; firmware does not write it).
  double gear_numerator;
  double gear_denominator;

  // ---- LEDC pulse generator ------------------------------------------------
  int ledc_channel;
  int ledc_pulse_pin;
  uint8_t ledc_resolution;

  // ---- Feature flags -------------------------------------------------------
  bool enable_encoder_feedback;
  bool enable_closed_loop_control;
  bool invert_output_logic;
  bool invert_dir_pin;
  int pcnt_unit;
  bool home_on_boot;
  uint32_t homing_speed_pps;

  // ---- Limit switch debounce ----------------------------------------------
  uint8_t limit_debounce_cycles;
  uint16_t limit_sample_interval_ms;
  bool limit_log_changes;

  DriverConfig()
      : pulse_pin(-1), dir_pin(-1), enable_pin(-1), alarm_reset_pin(-1),
        alarm_pin(-1), in_position_pin(-1), brake_pin(-1),
        encoder_a_pin(-1), encoder_b_pin(-1), encoder_z_pin(-1),
        limit_min_pin(-1), limit_max_pin(-1),
        pulse_mode(PulseMode::PULSE_DIRECTION),
        max_pulse_freq(600000),
        encoder_ppr(12000),
        gear_numerator(1.0),
        gear_denominator(1.0),
        ledc_channel(0),
        ledc_pulse_pin(-1),
        ledc_resolution(2),
        enable_encoder_feedback(false),
        enable_closed_loop_control(false),
        invert_output_logic(true),
        invert_dir_pin(false),
        pcnt_unit(0),
        home_on_boot(HOME_ON_BOOT),
        homing_speed_pps(6000),
        limit_debounce_cycles(10),
        limit_sample_interval_ms(3),
        limit_log_changes(true) {}
};

/**
 * @brief Real-time driver status snapshot.
 */
struct DriveStatus {
  bool servo_enabled;
  bool position_reached;
  bool brake_released;
  bool alarm_active;
  uint32_t current_position;
  int32_t encoder_position;
  int32_t position_error;
  uint32_t current_speed;
  uint32_t last_update_ms;
};

/**
 * @brief Trapezoidal motion profile state used by the ramp-timer ISR.
 */
struct MotionProfile {
  enum Phase : uint8_t { IDLE = 0, ACCEL, CRUISE, DECEL };

  uint32_t total_pulses;
  uint32_t accel_pulses;
  uint32_t decel_pulses;
  uint32_t cruise_pulses;
  uint32_t pulses_generated;
  uint32_t target_position;
  double current_freq;
  double max_freq;
  double accel_rate;
  double decel_rate;
  double fractional_pulses;
  int64_t last_update_us;
  Phase phase;
  bool direction;
};

using AlarmCallback = void (*)(const char *alarm_code);
using PositionReachedCallback = void (*)(uint32_t position);
using StatusUpdateCallback = void (*)(const DriveStatus &status);

/**
 * @class PulseMotorDriver
 * @brief Pulse-train motion engine (LEDC pulse output + PCNT encoder feedback).
 */
class PulseMotorDriver {
public:
  explicit PulseMotorDriver(const DriverConfig &config);
  ~PulseMotorDriver();

  bool initialize();

  bool enable();
  bool disable();
  bool isEnabled() const;
  bool isMotionActive() const;

  bool moveToPosition(uint32_t target_position, uint32_t max_speed = 10000,
                      uint32_t acceleration = 5000,
                      uint32_t deceleration = 5000);
  bool moveRelative(int64_t delta_counts, uint32_t max_speed = 10000,
                    uint32_t acceleration = 5000,
                    uint32_t deceleration = 5000);
  bool stopMotion(uint32_t deceleration = 50000);

  bool startHoming(uint32_t speed = 10000);
  bool isHoming() const { return homing_active_; }

  bool measureTravelDistance(uint32_t speed = 10000,
                             uint32_t acceleration = 10000,
                             uint32_t deceleration = 10000,
                             uint32_t timeout_ms = 60000);
  bool hasTravelDistance() const;
  uint32_t getTravelDistance() const;

  bool eStop();

  DriveStatus getStatus() const;
  uint32_t getPosition() const;
  void setPosition(uint32_t position);
  uint32_t getSpeed() const;
  bool isAlarmActive() const;
  bool clearAlarm();

  int32_t getEncoderPosition() const;
  void resetEncoderPosition();

  void setAlarmCallback(AlarmCallback callback);
  void setPositionReachedCallback(PositionReachedCallback callback);
  void setStatusUpdateCallback(StatusUpdateCallback callback);

  const DriverConfig &getConfig() const;
  void setConfig(const DriverConfig &config);
  String getConfigStatus() const;

  static String getVersion();
  String getDriverInfo() const;

private:
  DriverConfig config_;
  DriveStatus status_;
  bool initialized_, enabled_;

  // Encoder tracking
  int64_t encoder_accumulator_;
  int last_pcnt_count_;
  pcnt_unit_handle_t pcnt_unit_handle_;
  pcnt_channel_handle_t pcnt_chan_a_handle_;
  pcnt_channel_handle_t pcnt_chan_b_handle_;

  // Callbacks
  AlarmCallback alarm_callback_;
  PositionReachedCallback position_reached_callback_;
  StatusUpdateCallback status_update_callback_;

  // State
  uint32_t current_position_, target_position_, current_speed_pps_,
      last_status_update_ms_;
  bool motion_active_, current_direction_;
  bool homing_active_;
  bool travel_distance_valid_;
  uint32_t travel_distance_steps_;

  // Pin-level helpers
  bool writeNamedOutputPin(int pin, bool state, bool invert_dir_instead);
  bool setDirectionPin(bool state);
  bool setEnablePin(bool state);
  void stopLEDC();
  void stopMotionFromIsr(const char *reason, bool set_home);
  void handleMoveComplete();

  // Motion profile
  MotionProfile profile_;
  esp_timer_handle_t ramp_timer_ = NULL;
  static constexpr uint32_t RAMP_INTERVAL_US = 5000;

  static void rampTimerCallback(void *arg);
  void updateMotionProfile();
  void startMotionProfile(uint32_t total_pulses, double max_freq,
                          double accel_rate, double decel_rate, bool direction);

#if PULSE_MOTOR_USE_FREERTOS
  SemaphoreHandle_t mutex_ = NULL;
#endif
};

} // namespace PulseMotor

#endif // PULSE_MOTOR_H
