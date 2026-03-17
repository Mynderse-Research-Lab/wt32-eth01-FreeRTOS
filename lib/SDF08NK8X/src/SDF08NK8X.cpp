/**
 * @file SDF08NK8X.cpp
 * @brief Implementation of SDF08NK8X ServoDriver and GPIOManager for
 * Arduino/ESP32
 */

#include "SDF08NK8X.h"
#include "gpio_expander.h"
#include "driver/gpio.h"
#include <cmath>
#include <esp_log.h>
#include <inttypes.h>

using namespace BergerdaServo;
static const char *TAG = "SDF08NK8X";

namespace {
inline bool isEncodedDirectPin(int pin) {
  return (pin & GPIO_EXPANDER_DIRECT_FLAG) != 0;
}

inline bool isMcpLogicalPin(int pin) {
  return pin >= 0 && pin < GPIO_DIRECT_PIN_BASE && !isEncodedDirectPin(pin);
}

inline int resolveDirectGpioPin(int pin) {
  if (pin < 0) {
    return -1;
  }
  if (isEncodedDirectPin(pin)) {
    return pin & GPIO_EXPANDER_DIRECT_MASK;
  }
  if (pin >= GPIO_DIRECT_PIN_BASE) {
    return pin;
  }
  return -1;
}

bool configurePinDirectionSafe(int pin, bool isOutput) {
  if (pin < 0) {
    return true;
  }
  if (isMcpLogicalPin(pin)) {
    return gpio_expander_set_direction(pin, isOutput) == ESP_OK;
  }
  const int gpio = resolveDirectGpioPin(pin);
  if (gpio < 0) {
    return false;
  }
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << gpio);
  io_conf.mode = isOutput ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  return gpio_config(&io_conf) == ESP_OK;
}

bool writePinSafe(int pin, bool high) {
  if (pin < 0) {
    return false;
  }
  if (isMcpLogicalPin(pin)) {
    return gpio_expander_write(pin, high ? 1 : 0) == ESP_OK;
  }
  const int gpio = resolveDirectGpioPin(pin);
  if (gpio < 0) {
    return false;
  }
  return gpio_set_level((gpio_num_t)gpio, high ? 1 : 0) == ESP_OK;
}

int readPinSafe(int pin) {
  if (pin < 0) {
    return 0;
  }
  if (isMcpLogicalPin(pin)) {
    return gpio_expander_read(pin);
  }
  const int gpio = resolveDirectGpioPin(pin);
  if (gpio < 0) {
    return 0;
  }
  return gpio_get_level((gpio_num_t)gpio);
}
} // namespace

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
#define DEBUG_PRINT(...) \
  do { ESP_LOGD(TAG, "%s", String(__VA_ARGS__).c_str()); } while (0)
#define DEBUG_PRINTLN(...) \
  do { ESP_LOGD(TAG, "%s", String(__VA_ARGS__).c_str()); } while (0)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// ============================================================================
// SERVO DRIVER CONSTRUCTOR
// ============================================================================

ServoDriver::ServoDriver(const DriverConfig &config)
    : config_(config),
      status_(),
      initialized_(false),
      enabled_(false),
      encoder_accumulator_(0),
      last_pcnt_count_(0),
      pcnt_unit_handle_(nullptr),
      pcnt_chan_a_handle_(nullptr),
      pcnt_chan_b_handle_(nullptr),
      alarm_callback_(NULL),
      position_reached_callback_(NULL),
      status_update_callback_(NULL),
      current_position_(0),
      target_position_(0),
      current_speed_pps_(0),
      last_status_update_ms_(0),
      motion_active_(false),
      current_direction_(true),
      homing_active_(false),
      travel_distance_valid_(false),
      travel_distance_steps_(0) {
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
  ESP_LOGI(TAG, "[%lu] ServoDriver: esp_timer_create returned %d, ramp_timer_=%p", millis(), timer_err, ramp_timer_);
  if (timer_err != ESP_OK) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: ERROR - Failed to create timer!", millis());
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
  if (pcnt_unit_handle_) {
    pcnt_unit_stop(pcnt_unit_handle_);
    pcnt_unit_disable(pcnt_unit_handle_);
    if (pcnt_chan_a_handle_) {
      pcnt_del_channel(pcnt_chan_a_handle_);
      pcnt_chan_a_handle_ = nullptr;
    }
    if (pcnt_chan_b_handle_) {
      pcnt_del_channel(pcnt_chan_b_handle_);
      pcnt_chan_b_handle_ = nullptr;
    }
    pcnt_del_unit(pcnt_unit_handle_);
    pcnt_unit_handle_ = nullptr;
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

#if SDF08NK8X_DEBUG
  DEBUG_PRINT("ServoDriver: ESP_ARDUINO_VERSION=");
  DEBUG_PRINTLN(ESP_ARDUINO_VERSION);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  DEBUG_PRINTLN("ServoDriver: LEDC API v3+");
#else
  DEBUG_PRINTLN("ServoDriver: LEDC API v2");
#endif
#endif

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
  // TODO: Add support for CW/CCW and quadrature pulse modes.
  if (config_.pulse_mode != PulseMode::PULSE_DIRECTION) {
    DEBUG_PRINTLN("ServoDriver: Error - Only PULSE_DIRECTION mode is "
                  "supported in this version.");
    return false;
  }
  // TODO: Add support for SPEED/TORQUE/JOG control modes.
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

  // Validate closed-loop control requires encoder feedback
  if (config_.enable_closed_loop_control && !config_.enable_encoder_feedback) {
    DEBUG_PRINTLN("ServoDriver: Error - closed-loop control requires encoder feedback to be enabled");
    return false;
  }

  // Configure IO pins via gpio_expander abstraction so MCP logical pins
  // (0..15) and direct GPIO pins (>=16) are both handled safely.
  for (size_t i = 0; i < DriverConfig::OUTPUT_PIN_COUNT; i++) {
    if (config_.output_pin_nos[i] >= 0) {
      if (!configurePinDirectionSafe(config_.output_pin_nos[i], true)) {
        DEBUG_PRINTLN("ServoDriver: Failed to configure output pin direction");
        return false;
      }
      // Default to logical LOW (inactive), respecting output inversion.
      bool idle_level = config_.invert_output_logic ? HIGH : LOW;
      if (!writePinSafe(config_.output_pin_nos[i], idle_level == HIGH)) {
        DEBUG_PRINTLN("ServoDriver: Failed to set output idle level");
        return false;
      }
    }
  }
  for (size_t i = 0; i < DriverConfig::INPUT_PIN_COUNT; i++) {
    if (config_.input_pin_nos[i] >= 0) {
      if (!configurePinDirectionSafe(config_.input_pin_nos[i], false)) {
        DEBUG_PRINTLN("ServoDriver: Failed to configure input pin direction");
        return false;
      }
    }
  }

  // Initialize LEDC channel for pulse output
  if (config_.output_pin_nos[6] >= 0) {
    // Ensure ledc_pulse_pin is synced with output_pin_nos[6] for internal use
    config_.ledc_pulse_pin = resolveDirectGpioPin(config_.output_pin_nos[6]);
    if (config_.ledc_pulse_pin < 0) {
      DEBUG_PRINTLN("ServoDriver: Invalid pulse pin mapping for LEDC");
      return false;
    }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // Arduino ESP32 v3.0+ API - attach with initial frequency
    if (!ledcAttach(config_.ledc_pulse_pin, 1000, config_.ledc_resolution)) {
      DEBUG_PRINTLN("ServoDriver: LEDC Attach Failed!");
      return false;
    }
    ledcWrite(config_.ledc_pulse_pin, 0); // Start idle (0% duty)
#else
    // Arduino ESP32 v2.x API
    ledcSetup(config_.ledc_channel, 1000, config_.ledc_resolution);
    ledcAttachPin(config_.ledc_pulse_pin, config_.ledc_channel);
    ledcWrite(config_.ledc_channel, 0); // Start idle
#endif
    DEBUG_PRINTLN("ServoDriver: LEDC channel initialized");
  }

  // Initialize modern pulse counter (PCNT v2) for encoder quadrature decoding
  const int encoder_a_gpio = resolveDirectGpioPin(config_.input_pin_nos[3]);
  const int encoder_b_gpio = resolveDirectGpioPin(config_.input_pin_nos[4]);
  if (config_.enable_encoder_feedback && encoder_a_gpio > 0 &&
      encoder_b_gpio > 0) {
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32768;
    unit_config.high_limit = 32767;
    if (pcnt_new_unit(&unit_config, &pcnt_unit_handle_) != ESP_OK) {
      DEBUG_PRINTLN("ServoDriver: Failed to allocate PCNT unit");
      return false;
    }

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = encoder_a_gpio;
    chan_a_config.level_gpio_num = encoder_b_gpio;
    if (pcnt_new_channel(pcnt_unit_handle_, &chan_a_config, &pcnt_chan_a_handle_) != ESP_OK) {
      DEBUG_PRINTLN("ServoDriver: Failed to allocate PCNT channel A");
      return false;
    }

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = encoder_b_gpio;
    chan_b_config.level_gpio_num = encoder_a_gpio;
    if (pcnt_new_channel(pcnt_unit_handle_, &chan_b_config, &pcnt_chan_b_handle_) != ESP_OK) {
      DEBUG_PRINTLN("ServoDriver: Failed to allocate PCNT channel B");
      return false;
    }

    // Match legacy quadrature behavior:
    // - A channel: inc on posedge, dec on negedge
    // - B low inverts direction, B high keeps direction
    pcnt_channel_set_edge_action(pcnt_chan_a_handle_,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_a_handle_,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // B channel complements A channel edges for full quadrature decoding.
    pcnt_channel_set_edge_action(pcnt_chan_b_handle_,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_b_handle_,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1250;  // ~1.25us filter, equivalent to legacy value 100
    pcnt_unit_set_glitch_filter(pcnt_unit_handle_, &filter_config);

    pcnt_unit_enable(pcnt_unit_handle_);
    pcnt_unit_clear_count(pcnt_unit_handle_);
    pcnt_unit_start(pcnt_unit_handle_);
    last_pcnt_count_ = 0;

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
  ESP_LOGI(TAG, "[%lu] ServoDriver: enable() called, initialized_=%d, enabled_=%d", millis(), initialized_, enabled_);
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: enable() failed - not initialized", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (enabled_) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: enable() failed - already enabled", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (isAlarmActive()) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: enable() failed - alarm active", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  ESP_LOGI(TAG, "[%lu] ServoDriver: Calling setEnablePin(true)", millis());
  if (!setEnablePin(true)) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: enable() failed - setEnablePin returned false", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  enabled_ = true;
  status_.servo_enabled = true;
  ESP_LOGI(TAG, "[%lu] ServoDriver: enable() SUCCESS", millis());
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

  if (!setEnablePin(false)) {
    DEBUG_PRINTLN("ServoDriver: Failed to disable enable pin");
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  } 
  enabled_ = false;
  status_.servo_enabled = false;
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  DEBUG_PRINTLN("ServoDriver: Disabled");
  return true;
}

bool ServoDriver::isEnabled() const { return enabled_; }

bool ServoDriver::isAlarmActive() const {
  if (!initialized_ || config_.input_pin_nos[2] < 0) {
    return false;
  }
  // ALM is active LOW: electrical LOW means logical alarm active.
  return !readPinSafe(config_.input_pin_nos[2]);
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
  writeOutputPin(1, true);
  delayMicroseconds(100);
  writeOutputPin(1, false);

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

// Centralized ISR-safe stop sequence used by limit/alarm/homing checks.
// This avoids duplicated stop logic and guarantees a consistent stop path:
// - stop timer
// - stop LEDC pulses
// - update motion state and callbacks
void ServoDriver::stopMotionFromIsr(const char *reason, bool set_home) {
  if (reason) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: %s", millis(), reason);
  }
  if (set_home) {
    homing_active_ = false;
    current_position_ = 0;
  }
  motion_active_ = false;
  profile_.phase = MotionProfile::IDLE;
  if (ramp_timer_) {
    esp_timer_stop(ramp_timer_);
  }
  stopLEDC();
  handleMoveComplete();
}

void ServoDriver::handleMoveComplete() {
  motion_active_ = false;
  DEBUG_PRINT("ServoDriver: Move Complete. Position: ");
  DEBUG_PRINTLN(current_position_);
  ESP_LOGI(TAG, "[%lu] ServoDriver: Motion stopped at position=%lu",
               millis(), (unsigned long)current_position_);

  // Notify callback if registered
  if (position_reached_callback_) {
    position_reached_callback_(current_position_);
  }
}

// ============================================================================
// MOTION PROFILE (ACCELERATION/DECELERATION)
// ============================================================================

void ServoDriver::rampTimerCallback(void *arg) {
  static uint32_t callback_count = 0;
  callback_count++;
  if (callback_count % 100 == 0) {
    ESP_LOGI(TAG, "[CB] rampTimerCallback: call #%lu", (unsigned long)callback_count);
  }
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

  // Stop motion immediately if alarm is active
  bool alarm_now = isAlarmActive();
  if (alarm_now) {
    status_.alarm_active = true;
    stopMotionFromIsr("ALARM active - stopping motion", false);
    return;
  }

  int64_t now_us = esp_timer_get_time();
  double dt_s = (now_us - profile_.last_update_us) / 1000000.0;
  profile_.last_update_us = now_us;

  // Track fractional pulses so low-frequency updates do not lose steps.
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

  // CRITICAL: Absolute minimum frequency to prevent LEDC divider overflow.
  // The Arduino core clamps internally, but doing it here keeps the
  // motion profile and status in sync with actual hardware output.
  // Based on div_param <= 262143 with 80MHz APB clock:
  // 1-bit min ≈ 152.6 Hz, 2-bit min ≈ 76.3 Hz, 3-bit min ≈ 38.1 Hz, 4-bit min ≈ 19.1 Hz
  // We enforce the strictest (1-bit) minimum to avoid underflow in all cases.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  const double ABSOLUTE_MIN_FREQ = 5000.0;
#else
  const double ABSOLUTE_MIN_FREQ = 40000.0;
#endif

  // Phase state machine drives trapezoid/triangle profile:
  // ACCEL -> CRUISE -> DECEL -> IDLE
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
    // CRITICAL: Clamp frequency to minimum safe value during deceleration
    if (profile_.current_freq > 0 && profile_.current_freq < ABSOLUTE_MIN_FREQ) {
      profile_.current_freq = ABSOLUTE_MIN_FREQ;
    }
    if (profile_.current_freq <= 0 || remaining == 0) {
      profile_.current_freq = 0;
      profile_.phase = MotionProfile::IDLE;
      // Clamp position to exact target to eliminate accumulated drift
      // Only clamp if we completed normally (not stopped early)
      // Check if target_position was set by normal move completion
      if (profile_.pulses_generated >= profile_.total_pulses) {
        // In closed-loop mode, keep current_position_ as commanded (for tracking)
        // but don't force it to target since encoder may show different position
        // In open-loop mode, clamp to target position
        if (!config_.enable_closed_loop_control || !config_.enable_encoder_feedback) {
          // Open-loop: clamp to target position
          current_position_ = profile_.target_position;
        }
        // Closed-loop: current_position_ remains as commanded, encoder_position
        // reflects actual position (allows for error monitoring)
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

  // Update LEDC frequency based on current profile frequency.
  // We clamp to a minimum safe value and force 1-bit resolution to avoid
  // div_param overflow on ESP32 (v2 needs a much higher minimum).
  // CRITICAL: profile_.current_freq can be very low (even < 1 Hz) during acceleration/deceleration
  // We must always clamp it to a safe minimum and select appropriate resolution
  if (profile_.current_freq > 0) {
    // LEDC minimum frequency depends on resolution:
    // Resolution 1: min ~122 Hz, Resolution 2: min ~30.5 Hz, Resolution 3: min ~7.6 Hz
    // ESP32 APB clock is 80 MHz, so div_param = (80e6 / freq) / (1 << resolution)
    // div_param must be <= 0x3FFFF (262143) for ESP32
    // Maximum div_param = 262143, so: freq_min = 80e6 / (262143 * (1 << resolution))
    
    double freq = profile_.current_freq;
    uint8_t resolution;
    
    // Calculate minimum frequencies for each resolution
    const double MIN_FREQ_1BIT = 80e6 / (262143.0 * 2.0);   // ≈ 152.6 Hz
    const double MIN_FREQ_2BIT = 80e6 / (262143.0 * 4.0);   // ≈ 76.3 Hz
    const double MIN_FREQ_3BIT = 80e6 / (262143.0 * 8.0);  // ≈ 38.1 Hz
    const double MIN_FREQ_4BIT = 80e6 / (262143.0 * 16.0);  // ≈ 19.1 Hz
    (void)MIN_FREQ_1BIT;
    (void)MIN_FREQ_2BIT;
    (void)MIN_FREQ_3BIT;
    (void)MIN_FREQ_4BIT;
    
    // Simplify: always use 1-bit resolution at a safe minimum to prevent div_param overflow
    resolution = 1;
    // Hard minimum to ensure div_param stays below limit (extra margin above 1-bit theoretical min)
// Conservative floor; core v2 uses scaled divider (<<8) so needs higher min.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    const double LEDC_MIN_HZ = 1000.0; // lowered to allow slow movements
#else
    const double LEDC_MIN_HZ = 40000.0;
#endif
    if (freq < LEDC_MIN_HZ) {
      freq = LEDC_MIN_HZ;
      profile_.current_freq = LEDC_MIN_HZ; // keep status in sync
    }
    
    // Convert to integer frequency (LEDC requires uint32_t)
    uint32_t freq_hz = (uint32_t)(freq + 0.5);  // Round to nearest
    if (freq_hz < (uint32_t)LEDC_MIN_HZ) {
      freq_hz = (uint32_t)LEDC_MIN_HZ;
    }
    
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // v3 API: force hardware-generated 50% pulse train at requested frequency.
    (void)resolution;
    (void)ledcWriteTone(config_.ledc_pulse_pin, freq_hz);
#else
    // For ESP32 v2.x, ledcSetup reconfigures the channel
    // Track resolution changes and re-attach pin when resolution changes
    static uint8_t last_resolution = 255;  // Initialize to invalid value
    if (last_resolution != resolution) {
      // Resolution changed - re-attach pin to ensure it's properly configured
      ledcSetup(config_.ledc_channel, freq_hz, resolution);
      ledcAttachPin(config_.ledc_pulse_pin, config_.ledc_channel);
      last_resolution = resolution;
    } else {
      // Same resolution - just update frequency (ledcSetup should handle this)
      ledcSetup(config_.ledc_channel, freq_hz, resolution);
    }
    // Re-apply duty cycle after frequency change
    uint8_t max_duty_v2 = (1 << resolution) - 1;
    uint8_t duty_v2 = (max_duty_v2 > 0) ? ((max_duty_v2 + 1) / 2) : 1;
    ledcWrite(config_.ledc_channel, duty_v2);
#endif
    current_speed_pps_ = (uint32_t)profile_.current_freq;  // Keep original for reporting
  }

  // Sync status structure
  status_.current_position = current_position_;
  status_.current_speed = current_speed_pps_;
  if (config_.enable_encoder_feedback && pcnt_unit_handle_) {
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_handle_, &count);

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

  // Calculate position error
  // In closed-loop mode: use encoder position as actual position
  // In open-loop mode: use commanded position
  if (config_.enable_closed_loop_control && config_.enable_encoder_feedback) {
    // Closed-loop: error = target - actual encoder position
    status_.position_error =
        (int32_t)profile_.target_position - status_.encoder_position;
    
    // Closed-loop correction: adjust pulse generation based on position error
    // If we're behind (positive error), increase frequency slightly
    // If we're ahead (negative error), decrease frequency slightly
    int32_t error = status_.position_error;
    
    // Simple proportional correction (can be tuned)
    // Only apply correction if error is significant (avoid jitter)
    const int32_t ERROR_THRESHOLD = 10; // pulses
    if (abs(error) > ERROR_THRESHOLD && profile_.phase != MotionProfile::IDLE) {
      // Proportional gain: 0.1% per pulse error (tunable)
      const double KP = 0.001;
      double correction_factor = 1.0 + (KP * error);
      // Limit correction to reasonable bounds (e.g., ±20%)
      if (correction_factor > 1.2) correction_factor = 1.2;
      if (correction_factor < 0.8) correction_factor = 0.8;
      
      // Apply correction to current frequency
      profile_.current_freq *= correction_factor;
      // Clamp to max frequency
      if (profile_.current_freq > profile_.max_freq) {
        profile_.current_freq = profile_.max_freq;
      }
      // Ensure minimum frequency (122 Hz is safe minimum for 1-bit LEDC resolution)
      // Lower frequencies will be clamped in updateMotionProfile when LEDC is configured
      if (profile_.current_freq < 1.0) {
        profile_.current_freq = 1.0;  // Keep low for calculation, will be clamped later
      }
    }
  } else {
    // Open-loop: error = target - commanded position
    status_.position_error =
        (int32_t)profile_.target_position - (int32_t)current_position_;
  }
  status_.last_update_ms = millis();

  // Read status input pins (active LOW)
  if (config_.input_pin_nos[0] >= 0) {
    status_.position_reached = !readPinSafe(config_.input_pin_nos[0]);
  }
  if (config_.input_pin_nos[1] >= 0) {
    status_.brake_released = !readPinSafe(config_.input_pin_nos[1]);
  }

  // Check for alarm condition and invoke callback
  // Note: isAlarmActive() reads GPIO which is safe from ISR context on ESP32
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
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] startMotionProfile ENTER", millis());
  // Validate rates to avoid division by zero
  if (accel_rate <= 0.0 || decel_rate <= 0.0) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: Invalid accel/decel rates", millis());
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
  // CRITICAL: Use same resolution selection logic as updateMotionProfile
  // Calculate minimum frequencies for each resolution
  const double MIN_FREQ_1BIT = 80e6 / (262143.0 * 2.0);   // ≈ 152.6 Hz
  const double MIN_FREQ_2BIT = 80e6 / (262143.0 * 4.0);   // ≈ 76.3 Hz
  const double MIN_FREQ_3BIT = 80e6 / (262143.0 * 8.0);  // ≈ 38.1 Hz
  const double MIN_FREQ_4BIT = 80e6 / (262143.0 * 16.0);  // ≈ 19.1 Hz
  (void)MIN_FREQ_1BIT;
  (void)MIN_FREQ_2BIT;
  (void)MIN_FREQ_3BIT;
  (void)MIN_FREQ_4BIT;
  
  // Ensure max_freq is at least the minimum safe frequency
  // if (max_freq < MIN_FREQ_1BIT) {
  //   max_freq = MIN_FREQ_1BIT;
  //   DEBUG_PRINT("ServoDriver: Clamped max_freq to minimum safe value: ");
  //   DEBUG_PRINTLN(MIN_FREQ_1BIT);
  // }
  
  // Start with minimum safe frequency; force 1-bit to avoid div_param overflow
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)", millis());
  const double LEDC_MIN_HZ = 500.0; // lowered to allow slow movements
#else
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)", millis());
  const double LEDC_MIN_HZ = 40000.0; // higher floor for core v2 divider scaling
#endif
  double init_freq = profile_.current_freq;
  uint8_t init_resolution = 1;
  if (init_freq < LEDC_MIN_HZ) {
    init_freq = LEDC_MIN_HZ;
  }
  // Update profile current_freq to match
  profile_.current_freq = init_freq;
  
  uint8_t max_duty = (1 << init_resolution) - 1;
  // For 1-bit resolution, duty must be 1 (not 0) to generate pulses
  uint8_t duty = (max_duty > 0) ? ((max_duty + 1) / 2) : 1;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  (void)max_duty;
  (void)duty;
#endif
  uint32_t init_freq_hz = (uint32_t)(init_freq + 0.5);

#if SDF08NK8X_DEBUG
  DEBUG_PRINT("ServoDriver: LEDC init freq=");
  DEBUG_PRINT(init_freq_hz);
  DEBUG_PRINT(" Hz, resolution=");
  DEBUG_PRINTLN(init_resolution);
#endif

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // For v3+: stop output, detach, reattach with new frequency
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Stopping LEDC output", millis());
  ledcWrite(config_.ledc_pulse_pin, 0);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Detaching LEDC", millis());
  ledcDetach(config_.ledc_pulse_pin);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] About to call ledcAttach", millis());
  if (!ledcAttach(config_.ledc_pulse_pin, init_freq_hz, init_resolution)) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: ERROR - ledcAttach failed", millis());
    return;
  }
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] ledcAttach OK, calling ledcWriteTone at %lu Hz", millis(), (unsigned long)init_freq_hz);
  (void)ledcWriteTone(config_.ledc_pulse_pin, init_freq_hz);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] ledcWriteTone done", millis());
#else
  // For ESP32 v2.x, stop LEDC output first, then reconfigure
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Calling ledcWrite(0)", millis());
  ledcWrite(config_.ledc_channel, 0);
  delayMicroseconds(100);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Calling ledcSetup", millis());
  ledcSetup(config_.ledc_channel, init_freq_hz, init_resolution);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Calling ledcAttachPin", millis());
  ledcAttachPin(config_.ledc_pulse_pin, config_.ledc_channel);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Calling ledcWrite with duty=%d", millis(), duty);
  ledcWrite(config_.ledc_channel, duty);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] ledcWrite done", millis());
#endif

  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Starting ramp timer, ramp_timer_=%p, interval=%lu us", millis(), ramp_timer_, (unsigned long)RAMP_INTERVAL_US);
  // Start periodic ramp timer
  esp_err_t start_err = esp_timer_start_periodic(ramp_timer_, RAMP_INTERVAL_US);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] esp_timer_start_periodic returned %d", millis(), start_err);
  if (start_err != ESP_OK) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: ERROR - Failed to start timer!", millis());
  }

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
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] moveToPosition ENTER", millis());
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (isAlarmActive()) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: Alarm active - move rejected", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (!initialized_ || !enabled_) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: Motor not enabled", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  if (motion_active_) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: Busy (Movement in progress)", millis());
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
  
  // In closed-loop mode, use encoder position as current position for move calculation
  // In open-loop mode, use commanded position
  uint32_t current_pos = current_position_;
  if (config_.enable_closed_loop_control && config_.enable_encoder_feedback) {
    // Use encoder accumulator for accurate position (already updated in updateMotionProfile)
    // For move calculation, use encoder position as the starting point
    current_pos = (uint32_t)encoder_accumulator_;
    // Sync commanded position to encoder position for consistency
    current_position_ = current_pos;
  }
  
  int64_t delta = (int64_t)target_position - (int64_t)current_pos;
  uint32_t pulse_count = (uint32_t)std::abs(delta);
  bool direction = (delta >= 0);  // Logical direction: positive → true, negative → false
  ESP_LOGI(TAG, "[%lu] ServoDriver: delta=%lld, direction=%d", millis(), delta, direction);

  if (pulse_count == 0) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: Already at target position", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return true;
  }

  status_.position_error = delta;

  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] About to call startMotionProfile", millis());

  // Use motion profile with acceleration/deceleration
  startMotionProfile(pulse_count, (double)speed, (double)acceleration,
                     (double)deceleration, direction);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] startMotionProfile returned", millis());
#if SDF08NK8X_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  return true;
}

bool ServoDriver::moveRelative(int64_t delta_counts, uint32_t speed,
                               uint32_t acceleration, uint32_t deceleration) {
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] moveRelative ENTER, delta=%lld", millis(), delta_counts);
#if SDF08NK8X_USE_FREERTOS
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Taking mutex", millis());
  xSemaphoreTake(mutex_, portMAX_DELAY);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Mutex taken", millis());
#endif
  if (!initialized_ || !enabled_) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: Motor not enabled", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Calculating target", millis());
  // Calculate target position and validate it doesn't overflow
  int64_t target_signed = (int64_t)current_position_ + delta_counts;
  if (target_signed < 0) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: moveRelative would result in negative position", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (target_signed > UINT32_MAX) {
    ESP_LOGI(TAG, "[%lu] ServoDriver: moveRelative would overflow position", millis());
#if SDF08NK8X_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  uint32_t target = (uint32_t)target_signed;
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Target calculated: %lu", millis(), (unsigned long)target);
#if SDF08NK8X_USE_FREERTOS
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Giving mutex", millis());
  xSemaphoreGive(mutex_);
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Mutex given", millis());
#endif
  ESP_LOGI(TAG, "[%lu] ServoDriver: [DBG] Calling moveToPosition", millis());
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

  ESP_LOGI(TAG, "[%lu] ServoDriver: Motion stopped at position=%lu",
                millis(), (unsigned long)current_position_);
  DEBUG_PRINTLN("ServoDriver: Motion stopped immediately");
  return true;
}

bool ServoDriver::startHoming(uint32_t speed) {
  (void)speed;
  ESP_LOGW(TAG, "[%lu] ServoDriver: startHoming is disabled in driver; use Gantry layer limit logic", millis());
  return false;
}

bool ServoDriver::measureTravelDistance(uint32_t speed, uint32_t acceleration,
                                        uint32_t deceleration,
                                        uint32_t timeout_ms) {
  (void)speed;
  (void)acceleration;
  (void)deceleration;
  (void)timeout_ms;
  travel_distance_valid_ = false;
  travel_distance_steps_ = 0;
  ESP_LOGW(TAG, "[%lu] ServoDriver: measureTravelDistance is disabled in driver; use Gantry layer limit logic", millis());
  return false;
}

bool ServoDriver::hasTravelDistance() const { return false; }

uint32_t ServoDriver::getTravelDistance() const { return 0; }

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
    if (pcnt_unit_handle_) {
      pcnt_unit_clear_count(pcnt_unit_handle_);
    }
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
           "%" PRIu32 "\nSpeed: %" PRIu32 " pps",
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
           "Bergerda Servo Driver v%s"
           "Pulse Pin: %d | Dir Pin: %d | Enable Pin: %d | Alarm Pin: %d"
           "Max Freq: %" PRIu32 " Hz | Encoder PPR: %" PRIu32,
           SERVO_DRIVER_VERSION, config_.output_pin_nos[6],
           config_.output_pin_nos[7], config_.output_pin_nos[0],
           config_.input_pin_nos[2], config_.max_pulse_freq,
           config_.encoder_ppr);
  return String(buffer);
}

// ============================================================================
// PRIVATE HELPER METHODS
// ============================================================================

bool ServoDriver::writeOutputPin(size_t index, bool state) {
  if (!initialized_ || index >= DriverConfig::OUTPUT_PIN_COUNT ||
      config_.output_pin_nos[index] < 0) {
    return false;
  }
  // DIR pin (index 7) uses its own inversion flag
  bool physical_state =
      (index == 7) ? (config_.invert_dir_pin ? !state : state)
                   : (config_.invert_output_logic ? !state : state);
  if (index == 7) {  // DIR pin debug
    ESP_LOGI(TAG, "[%lu] writeOutputPin[7/DIR]: logical=%d, invert_dir=%d, physical=%d, pin=%d",
                  millis(), state, config_.invert_dir_pin, physical_state,
                  config_.output_pin_nos[index]);
  }
  return writePinSafe(config_.output_pin_nos[index], physical_state);
}

bool ServoDriver::setDirectionPin(bool state) {
  ESP_LOGI(TAG, "[%lu] setDirectionPin: logical state=%d", millis(), state);
  bool result = writeOutputPin(7, state);
  ESP_LOGI(TAG, "[%lu] setDirectionPin: returned %d", millis(), result);
  return result;
}

bool ServoDriver::setEnablePin(bool state) {
  return writeOutputPin(0, state);
}