/**
 * @file SDF08NK8X.cpp
 * @brief Implementation of SDF08NK8X ServoDriver and GPIOManager for
 * Arduino/ESP32
 */

#include "SDF08NK8X.h"
#include <cmath>

using namespace BergerdaServo;

// version constants
#define SDF08NK8X_VERSION_MAJOR "1"
#define SDF08NK8X_VERSION_MINOR "1"
#define SDF08NK8X_VERSION_PATCH "0"
#define SERVO_DRIVER_VERSION                                                   \
  SDF08NK8X_VERSION_MAJOR "." SDF08NK8X_VERSION_MINOR                          \
                          "." SDF08NK8X_VERSION_PATCH

// Debug logging
#ifndef SDF08NK8X_DEBUG
#define SDF08NK8X_DEBUG 1
#endif

#if SDF08NK8X_DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// ============================================================================
// SERVO DRIVER CONSTRUCTOR
// ============================================================================

ServoDriver::ServoDriver(const DriverConfig &config)
    : config_(config), initialized_(false), enabled_(false),
      current_position_(0), motion_active_(false), current_speed_pps_(0),
      current_direction_(true), last_status_update_ms_(0),
      encoder_accumulator_(0), last_pcnt_count_(0), alarm_callback_(NULL),
      position_reached_callback_(NULL), status_update_callback_(NULL) {
  // Initialize status structure
  status_.position_reached = false;
  status_.brake_released = false;
  status_.alarm_active = false;
  status_.servo_enabled = false;
  status_.current_position = 0;
  status_.encoder_position = 0;
  status_.position_error = 0;
  status_.current_speed = 0;
  status_.last_update_ms = 0;

  // Create periodic timer for motion profile ramping
  esp_timer_create_args_t ramp_args = {};
  ramp_args.callback = &ServoDriver::rampTimerCallback;
  ramp_args.arg = this;
  ramp_args.name = "servo_ramp_timer";
  esp_err_t timer_err = esp_timer_create(&ramp_args, &ramp_timer_);
  if (timer_err != ESP_OK) {
    DEBUG_PRINTLN("ServoDriver: Failed to create timer!");
    ramp_timer_ = NULL;
  }

  // Initialize motion profile
  profile_.phase = MotionProfile::IDLE;

#if SDF08NK8X_USE_FREERTOS
  // Create FreeRTOS mutex for thread safety
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == NULL) {
    DEBUG_PRINTLN("ServoDriver: Failed to create mutex!");
  }
#endif
}

ServoDriver::~ServoDriver() {
  // Stop any active motion
  if (motion_active_) {
    stopMotion(0);  // Hard stop
  }

  // Disable motor
  if (enabled_) {
    disable();
  }

  // Clean up timer
  if (ramp_timer_) {
    esp_timer_stop(ramp_timer_);
    esp_timer_delete(ramp_timer_);
    ramp_timer_ = NULL;
  }

  // Stop LEDC output
  stopLEDC();

  // Stop encoder counting
  if (config_.enable_encoder_feedback) {
    pcnt_counter_pause(config_.pcnt_unit);
    // Legacy driver doesn't have explicit deinit, but pause is good practice
  }

#if SDF08NK8X_USE_FREERTOS
  if (mutex_) {
    vSemaphoreDelete(mutex_);
    mutex_ = NULL;
  }
#endif
}

// ============================================================================
// INITIALIZATION & SHUTDOWN
// ============================================================================

bool ServoDriver::initialize() {
  if (initialized_) {
    DEBUG_PRINTLN("ServoDriver: Already initialized");
    return false;
  }

  // Check that timer was created successfully
  if (!ramp_timer_) {
    DEBUG_PRINTLN("ServoDriver: Timer not created - initialization failed");
    return false;
  }

#if SDF08NK8X_USE_FREERTOS
  // Check that mutex was created successfully
  if (!mutex_) {
    DEBUG_PRINTLN("ServoDriver: Mutex not created - initialization failed");
    return false;
  }
#endif

  // Validate configuration (Library currently only supports Pulse+Dir Position
  // Mode)
  if (config_.pulse_mode != PulseMode::PULSE_DIRECTION) {
    DEBUG_PRINTLN("ServoDriver: Error - Only PULSE_DIRECTION mode is "
                  "supported in this version.");
    return false;
  }
  if (config_.control_mode != ControlMode::POSITION) {
    DEBUG_PRINTLN("ServoDriver: Error - Only POSITION control mode is "
                  "supported in this version.");
    return false;
  }

  // Validate required pins are configured
  if (config_.output_pin_nos[6] < 0) {
    DEBUG_PRINTLN("ServoDriver: Error - Pulse pin (output_pin_nos[6]) must be configured");
    return false;
  }
  if (config_.output_pin_nos[7] < 0) {
    DEBUG_PRINTLN("ServoDriver: Error - Direction pin (output_pin_nos[7]) must be configured");
    return false;
  }
  if (config_.output_pin_nos[0] < 0) {
    DEBUG_PRINTLN("ServoDriver: Error - Enable pin (output_pin_nos[0]) must be configured");
    return false;
  }

  // Validate LEDC channel is in valid range (0-15 for ESP32)
  if (config_.ledc_channel < 0 || config_.ledc_channel > 15) {
    DEBUG_PRINTLN("ServoDriver: Error - LEDC channel must be 0-15");
    return false;
  }

  // Validate encoder PPR
  if (config_.encoder_ppr == 0 && config_.enable_encoder_feedback) {
    DEBUG_PRINTLN("ServoDriver: Warning - encoder_ppr is 0 but encoder feedback enabled");
  }

  // Configure GPIO pins
  for (size_t i = 0; i < DriverConfig::OUTPUT_PIN_COUNT; i++) {
    if (config_.output_pin_nos[i] >= 0) {
      pinMode(config_.output_pin_nos[i], OUTPUT);
      digitalWrite(config_.output_pin_nos[i], HIGH);
    }
  }
  for (size_t i = 0; i < DriverConfig::INPUT_PIN_COUNT; i++) {
    if (config_.input_pin_nos[i] >= 0) {
      pinMode(config_.input_pin_nos[i], INPUT);
    }
  }

  // Initialize LEDC channel for pulse output
  if (config_.output_pin_nos[6] >= 0) {
    // Ensure ledc_pulse_pin is synced with output_pin_nos[6] for internal use
    config_.ledc_pulse_pin = config_.output_pin_nos[6];

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // Arduino ESP32 v3.0+ API - attach with initial frequency
    if (!ledcAttach(config_.output_pin_nos[6], 1000, config_.ledc_resolution)) {
      DEBUG_PRINTLN("ServoDriver: LEDC Attach Failed!");
      return false;
    }
    ledcWrite(config_.output_pin_nos[6], 0); // Start idle (0% duty)
#else
    // Arduino ESP32 v2.x API
    ledcSetup(config_.ledc_channel, 1000, config_.ledc_resolution);
    ledcAttachPin(config_.output_pin_nos[6], config_.ledc_channel);
    ledcWrite(config_.ledc_channel, 0); // Start idle
#endif
    DEBUG_PRINTLN("ServoDriver: LEDC channel initialized");
  }

  // Initialize PCNT for encoder quadrature decoding
  if (config_.enable_encoder_feedback && config_.input_pin_nos[3] > 0 &&
      config_.input_pin_nos[4] > 0) {
    // Configure channel 0 for A signal (count on A edges, use B for direction)
    pcnt_config_t pcnt_config_a = {};
    pcnt_config_a.pulse_gpio_num = config_.input_pin_nos[3]; // A+
    pcnt_config_a.ctrl_gpio_num = config_.input_pin_nos[4];  // B+
    pcnt_config_a.channel = PCNT_CHANNEL_0;
    pcnt_config_a.unit = config_.pcnt_unit;
    pcnt_config_a.pos_mode = PCNT_COUNT_INC; // Count up on A rising when B=LOW
    pcnt_config_a.neg_mode =
        PCNT_COUNT_DEC; // Count down on A falling when B=LOW
    pcnt_config_a.lctrl_mode = PCNT_MODE_REVERSE; // Reverse on B=LOW
    pcnt_config_a.hctrl_mode = PCNT_MODE_KEEP;    // Keep on B=HIGH
    pcnt_config_a.counter_h_lim = 32767;
    pcnt_config_a.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config_a);

    // Configure channel 1 for B signal (count on B edges, use A for direction)
    pcnt_config_t pcnt_config_b = {};
    pcnt_config_b.pulse_gpio_num = config_.input_pin_nos[4]; // B+
    pcnt_config_b.ctrl_gpio_num = config_.input_pin_nos[3];  // A+
    pcnt_config_b.channel = PCNT_CHANNEL_1;
    pcnt_config_b.unit = config_.pcnt_unit;
    pcnt_config_b.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_b.neg_mode = PCNT_COUNT_INC;
    pcnt_config_b.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_b.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_b.counter_h_lim = 32767;
    pcnt_config_b.counter_l_lim = -32768;
    pcnt_unit_config(&pcnt_config_b);

    // Enable glitch filter (10us pulse width filter)
    pcnt_set_filter_value(config_.pcnt_unit, 100); // 100 * 80MHz = 1.25us
    pcnt_filter_enable(config_.pcnt_unit);

    // Clear and start counter
    pcnt_counter_pause(config_.pcnt_unit);
    pcnt_counter_clear(config_.pcnt_unit);
    pcnt_counter_resume(config_.pcnt_unit);

    DEBUG_PRINTLN("ServoDriver: Encoder PCNT initialized");
  }

  initialized_ = true;
  last_status_update_ms_ = millis();
  DEBUG_PRINTLN("ServoDriver: Initialized successfully");
  return true;
}

// ============================================================================
// SERVO ENABLE/DISABLE
// ============================================================================

bool ServoDriver::enable() {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || enabled_) {
    DEBUG_PRINTLN("ServoDriver: Not initialized or already enabled");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  } else if (isAlarmActive()) {
    DEBUG_PRINTLN("ServoDriver: Cannot enable - alarm active");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  setEnablePin(true);
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for servo lockup (FreeRTOS-safe)
  xSemaphoreTake(mutex_, portMAX_DELAY);
#else
  delay(100); // Wait for servo lockup
#endif
  enabled_ = true;
  status_.servo_enabled = true;
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  DEBUG_PRINTLN("ServoDriver: Enabled");
  return true;
}

bool ServoDriver::disable() {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || !enabled_) {
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  setEnablePin(false);
  enabled_ = false;
  motion_active_ = false;
  status_.servo_enabled = false;
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  DEBUG_PRINTLN("ServoDriver: Disabled");
  return true;
}

bool ServoDriver::isEnabled() const { return enabled_; }

bool ServoDriver::isAlarmActive() const {
  if (!initialized_ || config_.input_pin_nos[2] <= 0) {
    return false;
  }
  return !digitalRead(config_.input_pin_nos[2]); // Active LOW
}

bool ServoDriver::clearAlarm() {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || config_.output_pin_nos[1] < 0) {
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  // Pulse alarm reset pin to clear alarm
  // Note: delayMicroseconds is blocking but acceptable here as this function
  // should not be called from ISR context. If needed, use non-blocking delay.
  digitalWrite(config_.output_pin_nos[1], LOW);
  delayMicroseconds(100);
  digitalWrite(config_.output_pin_nos[1], HIGH);

#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  DEBUG_PRINTLN("ServoDriver: Alarm clear signal sent");
  return true;
}

void ServoDriver::stopLEDC() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(config_.ledc_pulse_pin, 0);
#else
  ledcWrite(config_.ledc_channel, 0);
#endif
}

void ServoDriver::handleMoveComplete() {
  motion_active_ = false;
  DEBUG_PRINT("ServoDriver: Move Complete. Position: ");
  DEBUG_PRINTLN(current_position_);

  // Notify callback if registered
  if (position_reached_callback_) {
    position_reached_callback_(current_position_);
  }
}

// ============================================================================
// MOTION PROFILE (ACCELERATION/DECELERATION)
// ============================================================================

void ServoDriver::rampTimerCallback(void *arg) {
  ServoDriver *driver = static_cast<ServoDriver *>(arg);
  driver->updateMotionProfile();
}

/**
 * @brief Update motion profile state machine
 * @note This function runs from ESP timer ISR context. All operations must be
 *       ISR-safe. GPIO reads (digitalRead) are safe on ESP32 from ISR context.
 *       Callbacks are invoked here, so they must also be ISR-safe.
 */
void ServoDriver::updateMotionProfile() {
  if (profile_.phase == MotionProfile::IDLE) {
    return;
  }

  int64_t now_us = esp_timer_get_time();
  double dt_s = (now_us - profile_.last_update_us) / 1000000.0;
  profile_.last_update_us = now_us;

  // Track fractional pulses to reduce truncation error
  double pulses_exact =
      profile_.current_freq * dt_s + profile_.fractional_pulses;
  uint32_t pulses_this_interval = (uint32_t)pulses_exact;
  profile_.fractional_pulses = pulses_exact - pulses_this_interval;
  profile_.pulses_generated += pulses_this_interval;

  // Update position tracking
  if (profile_.direction) {
    current_position_ += pulses_this_interval;
  } else {
    current_position_ -= pulses_this_interval;
  }

  // Determine remaining pulses
  uint32_t remaining =
      (profile_.pulses_generated >= profile_.total_pulses)
          ? 0
          : (profile_.total_pulses - profile_.pulses_generated);

  // Phase state machine
  switch (profile_.phase) {
  case MotionProfile::ACCEL:
    profile_.current_freq += profile_.accel_rate * dt_s;
    if (profile_.current_freq >= profile_.max_freq) {
      profile_.current_freq = profile_.max_freq;
      profile_.phase = MotionProfile::CRUISE;
      DEBUG_PRINTLN("ServoDriver: Phase -> CRUISE");
    }
    // Check if we need to start decelerating early (short move)
    if (remaining <= profile_.decel_pulses) {
      profile_.phase = MotionProfile::DECEL;
      DEBUG_PRINTLN("ServoDriver: Phase -> DECEL (early)");
    }
    break;

  case MotionProfile::CRUISE:
    // Check if it's time to decelerate
    if (remaining <= profile_.decel_pulses) {
      profile_.phase = MotionProfile::DECEL;
      DEBUG_PRINTLN("ServoDriver: Phase -> DECEL");
    }
    break;

  case MotionProfile::DECEL:
    profile_.current_freq -= profile_.decel_rate * dt_s;
    if (profile_.current_freq <= 0 || remaining == 0) {
      profile_.current_freq = 0;
      profile_.phase = MotionProfile::IDLE;
      // Clamp position to exact target to eliminate accumulated drift
      // Only clamp if we completed normally (not stopped early)
      // Check if target_position was set by normal move completion
      if (profile_.pulses_generated >= profile_.total_pulses) {
        current_position_ = profile_.target_position;
      }
      // Otherwise keep current_position_ as-is (stopped early via stopMotion)
      esp_timer_stop(ramp_timer_);
      stopLEDC();
      handleMoveComplete();
      DEBUG_PRINTLN("ServoDriver: Phase -> IDLE (complete)");
      return;
    }
    break;

  default:
    break;
  }

  // Update LEDC frequency
  if (profile_.current_freq > 0) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcChangeFrequency(config_.ledc_pulse_pin, (uint32_t)profile_.current_freq,
                        config_.ledc_resolution);
#else
    ledcSetup(config_.ledc_channel, profile_.current_freq,
              config_.ledc_resolution);
#endif
    current_speed_pps_ = (uint32_t)profile_.current_freq;
  }

  // Sync status structure
  status_.current_position = current_position_;
  status_.current_speed = current_speed_pps_;
  if (config_.enable_encoder_feedback) {
    int16_t count = 0;
    pcnt_get_counter_value(config_.pcnt_unit, &count);

    // Calculate delta and handle 16-bit wrap-around
    // PCNT counter limits: -32768 to 32767
    static constexpr int32_t PCNT_HALF_RANGE = 32768 / 2;
    static constexpr int32_t PCNT_FULL_RANGE = 32768 * 2;
    int32_t delta = count - last_pcnt_count_;
    if (delta < -PCNT_HALF_RANGE) {
      delta += PCNT_FULL_RANGE; // Underflow wrapped to high positive
    } else if (delta > PCNT_HALF_RANGE) {
      delta -= PCNT_FULL_RANGE; // Overflow wrapped to low negative
    }

    encoder_accumulator_ += delta;
    last_pcnt_count_ = count;

    status_.encoder_position = (int32_t)encoder_accumulator_;
  } else {
    status_.encoder_position = 0;
  }
  status_.position_error =
      (int32_t)profile_.target_position - (int32_t)current_position_;
  status_.last_update_ms = millis();

  // Read status input pins (active LOW)
  if (config_.input_pin_nos[0] >= 0) {
    status_.position_reached = !digitalRead(config_.input_pin_nos[0]);
  }
  if (config_.input_pin_nos[1] >= 0) {
    status_.brake_released = !digitalRead(config_.input_pin_nos[1]);
  }

  // Check for alarm condition and invoke callback
  // Note: isAlarmActive() reads GPIO which is safe from ISR context on ESP32
  bool alarm_now = isAlarmActive();
  if (alarm_now && !status_.alarm_active && alarm_callback_) {
    alarm_callback_("ALM");  // const char* is ISR-safe
  }
  status_.alarm_active = alarm_now;

  // Invoke status update callback if registered
  if (status_update_callback_) {
    status_update_callback_(status_);
  }
}

void ServoDriver::startMotionProfile(uint32_t total_pulses, double max_freq,
                                     double accel_rate, double decel_rate,
                                     bool direction) {
  // Validate rates to avoid division by zero
  if (accel_rate <= 0.0 || decel_rate <= 0.0) {
    DEBUG_PRINTLN("ServoDriver: Invalid accel/decel rates in startMotionProfile");
    return;
  }

  // Calculate kinematic profile
  // Time to accelerate to max_freq: t_accel = max_freq / accel_rate
  // Pulses during accel: P_accel = 0.5 * accel_rate * t_accel^2
  //                    = 0.5 * max_freq^2 / accel_rate
  double t_accel = max_freq / accel_rate;
  double t_decel = max_freq / decel_rate;
  uint32_t accel_pulses = (uint32_t)(0.5 * accel_rate * t_accel * t_accel);
  uint32_t decel_pulses = (uint32_t)(0.5 * decel_rate * t_decel * t_decel);

  // Handle short moves (triangular profile)
  if (accel_pulses + decel_pulses > total_pulses) {
    // Scale down - won't reach max speed
    double scale = sqrt((double)total_pulses / (accel_pulses + decel_pulses));
    accel_pulses = (uint32_t)(accel_pulses * scale * scale);
    decel_pulses = total_pulses - accel_pulses;
    max_freq = sqrt(2.0 * accel_rate * accel_pulses); // Reduced max freq
    DEBUG_PRINT("ServoDriver: Triangular profile, max_freq=");
    DEBUG_PRINTLN(max_freq);
  }

  uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;

  // Initialize profile state
  profile_.total_pulses = total_pulses;
  profile_.accel_pulses = accel_pulses;
  profile_.decel_pulses = decel_pulses;
  profile_.cruise_pulses = cruise_pulses;
  profile_.pulses_generated = 0;
  profile_.fractional_pulses = 0.0;
  profile_.current_freq = 1.0; // Start with minimal frequency
  profile_.max_freq = max_freq;
  profile_.accel_rate = accel_rate;
  profile_.decel_rate = decel_rate;
  profile_.last_update_us = esp_timer_get_time();
  profile_.phase = MotionProfile::ACCEL;
  profile_.direction = direction;
  profile_.target_position = target_position_;

  motion_active_ = true;
  current_direction_ = direction;
  setDirectionPin(direction);
  // Small delay to ensure direction pin settles before starting pulses
  // This is called from moveToPosition which already holds mutex, so blocking is OK
  delayMicroseconds(10);

  // Start LEDC at initial low frequency
  uint8_t max_duty = (1 << config_.ledc_resolution) - 1;
  uint8_t half_duty = max_duty / 2;

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcChangeFrequency(config_.ledc_pulse_pin, 1, config_.ledc_resolution);
  ledcWrite(config_.ledc_pulse_pin, half_duty);
#else
  ledcSetup(config_.ledc_channel, 1, config_.ledc_resolution);
  ledcWrite(config_.ledc_channel, half_duty);
#endif

  // Start periodic ramp timer
  esp_timer_start_periodic(ramp_timer_, RAMP_INTERVAL_US);

  DEBUG_PRINT("ServoDriver: Profile started. Total=");
  DEBUG_PRINT(total_pulses);
  DEBUG_PRINT(", Accel=");
  DEBUG_PRINT(accel_pulses);
  DEBUG_PRINT(", Cruise=");
  DEBUG_PRINT(cruise_pulses);
  DEBUG_PRINT(", Decel=");
  DEBUG_PRINTLN(decel_pulses);
}

// ============================================================================
// POSITION CONTROL
// ============================================================================

bool ServoDriver::moveToPosition(uint32_t target_position, uint32_t speed,
                                 uint32_t acceleration, uint32_t deceleration) {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || !enabled_) {
    DEBUG_PRINTLN("ServoDriver: Motor not enabled");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  if (motion_active_) {
    DEBUG_PRINTLN("ServoDriver: Busy (Movement in progress)");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  // Validate parameters
  if (speed == 0) {
    DEBUG_PRINTLN("ServoDriver: Invalid speed (must be > 0)");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (acceleration == 0) {
    DEBUG_PRINTLN("ServoDriver: Invalid acceleration (must be > 0)");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (deceleration == 0) {
    DEBUG_PRINTLN("ServoDriver: Invalid deceleration (must be > 0)");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  target_position_ = target_position;
  int64_t delta = (int64_t)target_position - (int64_t)current_position_;
  uint32_t pulse_count = (uint32_t)std::abs(delta);
  bool direction = (delta >= 0);

  if (pulse_count == 0) {
    DEBUG_PRINTLN("ServoDriver: Already at target position");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return true;
  }

  status_.position_error = delta;

  DEBUG_PRINT("ServoDriver: Moving to ");
  DEBUG_PRINT(target_position);
  DEBUG_PRINT(" (delta: ");
  DEBUG_PRINT(delta);
  DEBUG_PRINTLN(")");

  // Use motion profile with acceleration/deceleration
  startMotionProfile(pulse_count, (double)speed, (double)acceleration,
                     (double)deceleration, direction);
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  return true;
}

bool ServoDriver::moveRelative(int64_t delta_counts, uint32_t speed,
                               uint32_t acceleration, uint32_t deceleration) {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || !enabled_) {
    DEBUG_PRINTLN("ServoDriver: Motor not enabled");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  // Calculate target position and validate it doesn't overflow
  int64_t target_signed = (int64_t)current_position_ + delta_counts;
  if (target_signed < 0) {
    DEBUG_PRINTLN("ServoDriver: moveRelative would result in negative position");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (target_signed > UINT32_MAX) {
    DEBUG_PRINTLN("ServoDriver: moveRelative would overflow position");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  uint32_t target = (uint32_t)target_signed;
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  return moveToPosition(target, speed, acceleration, deceleration);
}

bool ServoDriver::stopMotion(uint32_t deceleration) {
  // Gracefully stop motion. If deceleration > 0, ramps down.
  // Otherwise stops immediately.
  (void)deceleration; // Suppress unused warning if logic below skipped
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_) {
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  if (motion_active_ && deceleration > 0 && current_speed_pps_ > 0) {
    // Smooth stop: Transition directly to DECEL phase
    profile_.decel_rate = (double)deceleration;
    profile_.phase = MotionProfile::DECEL;
    // Don't modify total_pulses or target_position - let decel complete naturally
    // updateMotionProfile() will check if we actually completed all pulses
    // (pulses_generated >= total_pulses) before clamping to target_position.
    // If we stopped early, pulses_generated < total_pulses, so we won't clamp.

    DEBUG_PRINTLN("ServoDriver: Ramping down...");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return true;
  }

  // Hard stop (or already stopped)
  motion_active_ = false;
  current_speed_pps_ = 0;
  profile_.phase = MotionProfile::IDLE;

  if (ramp_timer_) {
    esp_timer_stop(ramp_timer_);
  }
  stopLEDC();
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif

  DEBUG_PRINTLN("ServoDriver: Motion stopped immediately");
  return true;
}

bool ServoDriver::eStop() {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_) {
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  stopMotion(0);  // Hard stop immediately
  disable();

  DEBUG_PRINTLN("ServoDriver: EMERGENCY STOP");
  return true;
}

// ============================================================================
// STATUS & MONITORING
// ============================================================================

DriveStatus ServoDriver::getStatus() const { return status_; }

uint32_t ServoDriver::getPosition() const { return current_position_; }

void ServoDriver::setPosition(uint32_t position) {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  current_position_ = position;
  status_.current_position = position;
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
}

uint32_t ServoDriver::getSpeed() const {
  if (motion_active_) {
    return current_speed_pps_;
  }
  return 0;
}

bool ServoDriver::isMotionActive() const { return motion_active_; }

// ============================================================================
// ENCODER FEEDBACK
// ============================================================================

int32_t ServoDriver::getEncoderPosition() const {
  if (!config_.enable_encoder_feedback) {
    return 0;
  }
  return (int32_t)encoder_accumulator_;
}

void ServoDriver::resetEncoderPosition() {
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (config_.enable_encoder_feedback) {
    pcnt_counter_pause(config_.pcnt_unit);
    pcnt_counter_clear(config_.pcnt_unit);
    pcnt_counter_resume(config_.pcnt_unit);
    encoder_accumulator_ = 0;
    last_pcnt_count_ = 0;
  }
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
}

// ============================================================================
// CALLBACKS
// ============================================================================

void ServoDriver::setAlarmCallback(AlarmCallback callback) {
  alarm_callback_ = callback;
}

void ServoDriver::setPositionReachedCallback(PositionReachedCallback callback) {
  position_reached_callback_ = callback;
}

void ServoDriver::setStatusUpdateCallback(StatusUpdateCallback callback) {
  status_update_callback_ = callback;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

const DriverConfig &ServoDriver::getConfig() const { return config_; }

void ServoDriver::setConfig(const DriverConfig &config) { config_ = config; }

String ServoDriver::getConfigStatus() const {
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "BergerdaServoDriver v%s\nInitialized: %s\nEnabled: %s\nPosition: "
           "%u\nSpeed: %u pps",
           SERVO_DRIVER_VERSION, initialized_ ? "Yes" : "No",
           enabled_ ? "Yes" : "No", current_position_, current_speed_pps_);
  return String(buffer);
}

// ============================================================================
// ADVANCED FEATURES
// ============================================================================

String ServoDriver::getVersion() { return String(SERVO_DRIVER_VERSION); }

String ServoDriver::getDriverInfo() const {
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "Bergerda Servo Driver v%s\n"
           "Pulse Pin: %d | Dir Pin: %d | Enable Pin: %d | Alarm Pin: %d\n"
           "Max Freq: %u Hz | Encoder PPR: %u",
           SERVO_DRIVER_VERSION, config_.output_pin_nos[6],
           config_.output_pin_nos[7], config_.output_pin_nos[0],
           config_.input_pin_nos[2], config_.max_pulse_freq,
           config_.encoder_ppr);
  return String(buffer);
}

// ============================================================================
// PRIVATE HELPER METHODS
// ============================================================================

void ServoDriver::setDirectionPin(bool state) {
  if (!initialized_ || config_.output_pin_nos[7] < 0) {
    return;
  }
  digitalWrite(config_.output_pin_nos[7], state ? HIGH : LOW);
}

void ServoDriver::setEnablePin(bool state) {
  if (!initialized_ || config_.output_pin_nos[0] < 0) {
    return;
  }
  digitalWrite(config_.output_pin_nos[0], state ? HIGH : LOW);
}