/**
 * @file PulseMotor.cpp
 * @brief PulseMotor pulse-train motion engine implementation for ESP32.
 */

#include "PulseMotor.h"
#include "gpio_expander.h"
#include "driver/gpio.h"
#include <cmath>
#include <esp_log.h>
#include <inttypes.h>

using namespace PulseMotor;
static const char *TAG = "PulseMotor";

// ============================================================================
// GPIO HELPERS (MCP23S17 logical pins vs direct ESP32 GPIO)
// ============================================================================

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

// ============================================================================
// DRIVETRAIN CONVERSION HELPERS
// ============================================================================

namespace PulseMotor {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double pulsesPerMm(const DrivetrainConfig &config) {
  const double numerator =
      static_cast<double>(config.encoder_ppr) *
      static_cast<double>(config.motor_reducer_ratio);
  if (numerator <= 0.0) {
    return 0.0;
  }

  switch (config.type) {
  case DrivetrainType::BALLSCREW:
    return config.lead_mm > 0.0f ? (numerator / config.lead_mm) : 0.0;
  case DrivetrainType::BELT:
    return config.belt_lead_mm_per_rev > 0.0f
               ? (numerator / config.belt_lead_mm_per_rev)
               : 0.0;
  case DrivetrainType::RACKPINION: {
    const double lead = kPi * static_cast<double>(config.pinion_pitch_diameter_mm);
    return lead > 0.0 ? (numerator / lead) : 0.0;
  }
  case DrivetrainType::ROTARY_DIRECT:
  default:
    return 0.0;
  }
}

double pulsesPerDeg(const DrivetrainConfig &config) {
  if (config.type != DrivetrainType::ROTARY_DIRECT) {
    return 0.0;
  }
  const double numerator =
      static_cast<double>(config.encoder_ppr) *
      static_cast<double>(config.motor_reducer_ratio) *
      static_cast<double>(config.output_gear_ratio);
  return numerator > 0.0 ? (numerator / 360.0) : 0.0;
}

} // namespace PulseMotor

// ============================================================================
// VERSION / DEBUG MACROS
// ============================================================================

#define PULSE_MOTOR_VERSION_MAJOR "2"
#define PULSE_MOTOR_VERSION_MINOR "0"
#define PULSE_MOTOR_VERSION_PATCH "0"
#define PULSE_MOTOR_VERSION_STRING                                             \
  PULSE_MOTOR_VERSION_MAJOR "." PULSE_MOTOR_VERSION_MINOR                      \
                            "." PULSE_MOTOR_VERSION_PATCH

#ifndef PULSE_MOTOR_DEBUG
#define PULSE_MOTOR_DEBUG 1
#endif

#if PULSE_MOTOR_DEBUG
#define DEBUG_PRINT(...)                                                       \
  do {                                                                         \
    ESP_LOGD(TAG, "%s", String(__VA_ARGS__).c_str());                          \
  } while (0)
#define DEBUG_PRINTLN(...)                                                     \
  do {                                                                         \
    ESP_LOGD(TAG, "%s", String(__VA_ARGS__).c_str());                          \
  } while (0)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

PulseMotorDriver::PulseMotorDriver(const DriverConfig &config)
    : config_(config), status_(), initialized_(false), enabled_(false),
      encoder_accumulator_(0), last_pcnt_count_(0),
      pcnt_unit_handle_(nullptr), pcnt_chan_a_handle_(nullptr),
      pcnt_chan_b_handle_(nullptr),
      alarm_callback_(NULL), position_reached_callback_(NULL),
      status_update_callback_(NULL),
      current_position_(0), target_position_(0), current_speed_pps_(0),
      last_status_update_ms_(0),
      motion_active_(false), current_direction_(true), homing_active_(false),
      travel_distance_valid_(false), travel_distance_steps_(0) {
  status_.position_reached = false;
  status_.brake_released = false;
  status_.alarm_active = false;
  status_.servo_enabled = false;
  status_.current_position = 0;
  status_.encoder_position = 0;
  status_.position_error = 0;
  status_.current_speed = 0;
  status_.last_update_ms = 0;

  esp_timer_create_args_t ramp_args = {};
  ramp_args.callback = &PulseMotorDriver::rampTimerCallback;
  ramp_args.arg = this;
  ramp_args.name = "pulse_motor_ramp_timer";
  esp_err_t timer_err = esp_timer_create(&ramp_args, &ramp_timer_);
  ESP_LOGI(TAG, "[%lu] PulseMotor: esp_timer_create returned %d, ramp_timer_=%p",
           millis(), timer_err, ramp_timer_);
  if (timer_err != ESP_OK) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: ERROR - Failed to create timer!", millis());
    ramp_timer_ = NULL;
  }

  profile_.phase = MotionProfile::IDLE;

#if PULSE_MOTOR_USE_FREERTOS
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == NULL) {
    DEBUG_PRINTLN("PulseMotor: Failed to create mutex!");
  }
#endif
}

PulseMotorDriver::~PulseMotorDriver() {
  if (motion_active_) {
    stopMotion(0);
  }

  if (enabled_) {
    disable();
  }

  if (ramp_timer_) {
    esp_timer_stop(ramp_timer_);
    esp_timer_delete(ramp_timer_);
    ramp_timer_ = NULL;
  }

  stopLEDC();

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

#if PULSE_MOTOR_USE_FREERTOS
  if (mutex_) {
    vSemaphoreDelete(mutex_);
    mutex_ = NULL;
  }
#endif
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool PulseMotorDriver::initialize() {
  if (initialized_) {
    DEBUG_PRINTLN("PulseMotor: Already initialized");
    return false;
  }

#if PULSE_MOTOR_DEBUG
  DEBUG_PRINT("PulseMotor: ESP_ARDUINO_VERSION=");
  DEBUG_PRINTLN(ESP_ARDUINO_VERSION);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  DEBUG_PRINTLN("PulseMotor: LEDC API v3+");
#else
  DEBUG_PRINTLN("PulseMotor: LEDC API v2");
#endif
#endif

  if (!ramp_timer_) {
    DEBUG_PRINTLN("PulseMotor: Timer not created - initialization failed");
    return false;
  }

#if PULSE_MOTOR_USE_FREERTOS
  if (!mutex_) {
    DEBUG_PRINTLN("PulseMotor: Mutex not created - initialization failed");
    return false;
  }
#endif

  if (config_.pulse_mode != PulseMode::PULSE_DIRECTION) {
    DEBUG_PRINTLN("PulseMotor: Error - Only PULSE_DIRECTION mode is supported");
    return false;
  }

  if (config_.pulse_pin < 0) {
    DEBUG_PRINTLN("PulseMotor: Error - pulse_pin must be configured");
    return false;
  }
  if (config_.dir_pin < 0) {
    DEBUG_PRINTLN("PulseMotor: Error - dir_pin must be configured");
    return false;
  }
  if (config_.enable_pin < 0) {
    DEBUG_PRINTLN("PulseMotor: Error - enable_pin must be configured");
    return false;
  }

  if (config_.ledc_channel < 0 || config_.ledc_channel > 15) {
    DEBUG_PRINTLN("PulseMotor: Error - LEDC channel must be 0-15");
    return false;
  }

  if (config_.encoder_ppr == 0 && config_.enable_encoder_feedback) {
    DEBUG_PRINTLN("PulseMotor: Warning - encoder_ppr is 0 but encoder feedback enabled");
  }

  if (config_.enable_closed_loop_control && !config_.enable_encoder_feedback) {
    DEBUG_PRINTLN("PulseMotor: Error - closed-loop control requires encoder feedback");
    return false;
  }

  // Configure output pins (direction, idle level respects invert_output_logic)
  const int output_pins[] = {
      config_.pulse_pin,
      config_.dir_pin,
      config_.enable_pin,
      config_.alarm_reset_pin,
  };
  for (int pin : output_pins) {
    if (pin < 0) continue;
    if (!configurePinDirectionSafe(pin, true)) {
      DEBUG_PRINTLN("PulseMotor: Failed to configure output pin direction");
      return false;
    }
    bool idle_level = config_.invert_output_logic ? HIGH : LOW;
    if (!writePinSafe(pin, idle_level == HIGH)) {
      DEBUG_PRINTLN("PulseMotor: Failed to set output idle level");
      return false;
    }
  }

  // Configure input pins
  const int input_pins[] = {
      config_.in_position_pin,
      config_.brake_pin,
      config_.alarm_pin,
      config_.encoder_a_pin,
      config_.encoder_b_pin,
      config_.encoder_z_pin,
  };
  for (int pin : input_pins) {
    if (pin < 0) continue;
    if (!configurePinDirectionSafe(pin, false)) {
      DEBUG_PRINTLN("PulseMotor: Failed to configure input pin direction");
      return false;
    }
  }

  // LEDC for pulse output
  if (config_.pulse_pin >= 0) {
    config_.ledc_pulse_pin = resolveDirectGpioPin(config_.pulse_pin);
    if (config_.ledc_pulse_pin < 0) {
      DEBUG_PRINTLN("PulseMotor: Invalid pulse pin mapping for LEDC");
      return false;
    }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    if (!ledcAttach(config_.ledc_pulse_pin, 1000, config_.ledc_resolution)) {
      DEBUG_PRINTLN("PulseMotor: LEDC Attach Failed!");
      return false;
    }
    ledcWrite(config_.ledc_pulse_pin, 0);
#else
    ledcSetup(config_.ledc_channel, 1000, config_.ledc_resolution);
    ledcAttachPin(config_.ledc_pulse_pin, config_.ledc_channel);
    ledcWrite(config_.ledc_channel, 0);
#endif
    DEBUG_PRINTLN("PulseMotor: LEDC channel initialized");
  }

  // PCNT quadrature encoder
  const int encoder_a_gpio = resolveDirectGpioPin(config_.encoder_a_pin);
  const int encoder_b_gpio = resolveDirectGpioPin(config_.encoder_b_pin);
  if (config_.enable_encoder_feedback && encoder_a_gpio > 0 && encoder_b_gpio > 0) {
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32768;
    unit_config.high_limit = 32767;
    if (pcnt_new_unit(&unit_config, &pcnt_unit_handle_) != ESP_OK) {
      DEBUG_PRINTLN("PulseMotor: Failed to allocate PCNT unit");
      return false;
    }

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = encoder_a_gpio;
    chan_a_config.level_gpio_num = encoder_b_gpio;
    if (pcnt_new_channel(pcnt_unit_handle_, &chan_a_config, &pcnt_chan_a_handle_) != ESP_OK) {
      DEBUG_PRINTLN("PulseMotor: Failed to allocate PCNT channel A");
      return false;
    }

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = encoder_b_gpio;
    chan_b_config.level_gpio_num = encoder_a_gpio;
    if (pcnt_new_channel(pcnt_unit_handle_, &chan_b_config, &pcnt_chan_b_handle_) != ESP_OK) {
      DEBUG_PRINTLN("PulseMotor: Failed to allocate PCNT channel B");
      return false;
    }

    pcnt_channel_set_edge_action(pcnt_chan_a_handle_,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_a_handle_,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_channel_set_edge_action(pcnt_chan_b_handle_,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_b_handle_,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1250;
    pcnt_unit_set_glitch_filter(pcnt_unit_handle_, &filter_config);

    pcnt_unit_enable(pcnt_unit_handle_);
    pcnt_unit_clear_count(pcnt_unit_handle_);
    pcnt_unit_start(pcnt_unit_handle_);
    last_pcnt_count_ = 0;

    DEBUG_PRINTLN("PulseMotor: Encoder PCNT initialized");
  }

  initialized_ = true;
  last_status_update_ms_ = millis();
  DEBUG_PRINTLN("PulseMotor: Initialized successfully");
  return true;
}

// ============================================================================
// ENABLE / DISABLE
// ============================================================================

bool PulseMotorDriver::enable() {
  ESP_LOGI(TAG, "[%lu] PulseMotor: enable() called, initialized_=%d, enabled_=%d",
           millis(), initialized_, enabled_);
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: enable() failed - not initialized", millis());
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (enabled_) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: enable() failed - already enabled", millis());
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (isAlarmActive()) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: enable() failed - alarm active", millis());
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  ESP_LOGI(TAG, "[%lu] PulseMotor: Calling setEnablePin(true)", millis());
  if (!setEnablePin(true)) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: enable() failed - setEnablePin returned false", millis());
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  enabled_ = true;
  status_.servo_enabled = true;
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  ESP_LOGI(TAG, "[%lu] PulseMotor: enable() SUCCESS", millis());
  return true;
}

bool PulseMotorDriver::disable() {
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || !enabled_) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  if (!setEnablePin(false)) {
    DEBUG_PRINTLN("PulseMotor: Failed to disable enable pin");
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  enabled_ = false;
  status_.servo_enabled = false;
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  DEBUG_PRINTLN("PulseMotor: Disabled");
  return true;
}

bool PulseMotorDriver::isEnabled() const { return enabled_; }

bool PulseMotorDriver::isAlarmActive() const {
  if (!initialized_ || config_.alarm_pin < 0) {
    return false;
  }
  // ALM is active LOW: electrical LOW means logical alarm active.
  return !readPinSafe(config_.alarm_pin);
}

bool PulseMotorDriver::clearAlarm() {
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || config_.alarm_reset_pin < 0) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  writeNamedOutputPin(config_.alarm_reset_pin, true, false);
  delayMicroseconds(100);
  writeNamedOutputPin(config_.alarm_reset_pin, false, false);

#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  DEBUG_PRINTLN("PulseMotor: Alarm clear signal sent");
  return true;
}

void PulseMotorDriver::stopLEDC() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(config_.ledc_pulse_pin, 0);
#else
  ledcWrite(config_.ledc_channel, 0);
#endif
}

void PulseMotorDriver::stopMotionFromIsr(const char *reason, bool set_home) {
  if (reason) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: %s", millis(), reason);
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

void PulseMotorDriver::handleMoveComplete() {
  motion_active_ = false;
  DEBUG_PRINT("PulseMotor: Move Complete. Position: ");
  DEBUG_PRINTLN(current_position_);
  ESP_LOGI(TAG, "[%lu] PulseMotor: Motion stopped at position=%lu",
           millis(), (unsigned long)current_position_);

  if (position_reached_callback_) {
    position_reached_callback_(current_position_);
  }
}

// ============================================================================
// MOTION PROFILE (ACCELERATION / DECELERATION)
// ============================================================================

void PulseMotorDriver::rampTimerCallback(void *arg) {
  static uint32_t callback_count = 0;
  callback_count++;
  if (callback_count % 100 == 0) {
    ESP_LOGI(TAG, "[CB] rampTimerCallback: call #%lu", (unsigned long)callback_count);
  }
  PulseMotorDriver *driver = static_cast<PulseMotorDriver *>(arg);
  driver->updateMotionProfile();
}

void PulseMotorDriver::updateMotionProfile() {
  if (profile_.phase == MotionProfile::IDLE) {
    return;
  }

  bool alarm_now = isAlarmActive();
  if (alarm_now) {
    status_.alarm_active = true;
    stopMotionFromIsr("ALARM active - stopping motion", false);
    return;
  }

  int64_t now_us = esp_timer_get_time();
  double dt_s = (now_us - profile_.last_update_us) / 1000000.0;
  profile_.last_update_us = now_us;

  double pulses_exact =
      profile_.current_freq * dt_s + profile_.fractional_pulses;
  uint32_t pulses_this_interval = (uint32_t)pulses_exact;
  profile_.fractional_pulses = pulses_exact - pulses_this_interval;
  profile_.pulses_generated += pulses_this_interval;

  if (profile_.direction) {
    current_position_ += pulses_this_interval;
  } else {
    current_position_ -= pulses_this_interval;
  }

  uint32_t remaining =
      (profile_.pulses_generated >= profile_.total_pulses)
          ? 0
          : (profile_.total_pulses - profile_.pulses_generated);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  const double ABSOLUTE_MIN_FREQ = 5000.0;
#else
  const double ABSOLUTE_MIN_FREQ = 40000.0;
#endif
  (void)ABSOLUTE_MIN_FREQ;

  switch (profile_.phase) {
  case MotionProfile::ACCEL:
    profile_.current_freq += profile_.accel_rate * dt_s;
    if (profile_.current_freq >= profile_.max_freq) {
      profile_.current_freq = profile_.max_freq;
      profile_.phase = MotionProfile::CRUISE;
      DEBUG_PRINTLN("PulseMotor: Phase -> CRUISE");
    }
    if (remaining <= profile_.decel_pulses) {
      profile_.phase = MotionProfile::DECEL;
      DEBUG_PRINTLN("PulseMotor: Phase -> DECEL (early)");
    }
    break;

  case MotionProfile::CRUISE:
    if (remaining <= profile_.decel_pulses) {
      profile_.phase = MotionProfile::DECEL;
      DEBUG_PRINTLN("PulseMotor: Phase -> DECEL");
    }
    break;

  case MotionProfile::DECEL:
    profile_.current_freq -= profile_.decel_rate * dt_s;
    if (profile_.current_freq > 0 && profile_.current_freq < 1000.0) {
      profile_.current_freq = 1000.0;
    }
    if (profile_.current_freq <= 0 || remaining == 0) {
      profile_.current_freq = 0;
      profile_.phase = MotionProfile::IDLE;
      if (profile_.pulses_generated >= profile_.total_pulses) {
        if (!config_.enable_closed_loop_control || !config_.enable_encoder_feedback) {
          current_position_ = profile_.target_position;
        }
      }
      esp_timer_stop(ramp_timer_);
      stopLEDC();
      handleMoveComplete();
      DEBUG_PRINTLN("PulseMotor: Phase -> IDLE (complete)");
      return;
    }
    break;

  default:
    break;
  }

  if (profile_.current_freq > 0) {
    double freq = profile_.current_freq;
    uint8_t resolution = 1;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    const double LEDC_MIN_HZ = 1000.0;
#else
    const double LEDC_MIN_HZ = 40000.0;
#endif
    if (freq < LEDC_MIN_HZ) {
      freq = LEDC_MIN_HZ;
      profile_.current_freq = LEDC_MIN_HZ;
    }

    uint32_t freq_hz = (uint32_t)(freq + 0.5);
    if (freq_hz < (uint32_t)LEDC_MIN_HZ) {
      freq_hz = (uint32_t)LEDC_MIN_HZ;
    }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    (void)resolution;
    (void)ledcWriteTone(config_.ledc_pulse_pin, freq_hz);
#else
    static uint8_t last_resolution = 255;
    if (last_resolution != resolution) {
      ledcSetup(config_.ledc_channel, freq_hz, resolution);
      ledcAttachPin(config_.ledc_pulse_pin, config_.ledc_channel);
      last_resolution = resolution;
    } else {
      ledcSetup(config_.ledc_channel, freq_hz, resolution);
    }
    uint8_t max_duty_v2 = (1 << resolution) - 1;
    uint8_t duty_v2 = (max_duty_v2 > 0) ? ((max_duty_v2 + 1) / 2) : 1;
    ledcWrite(config_.ledc_channel, duty_v2);
#endif
    current_speed_pps_ = (uint32_t)profile_.current_freq;
  }

  status_.current_position = current_position_;
  status_.current_speed = current_speed_pps_;
  if (config_.enable_encoder_feedback && pcnt_unit_handle_) {
    int count = 0;
    pcnt_unit_get_count(pcnt_unit_handle_, &count);

    static constexpr int32_t PCNT_HALF_RANGE = 32768 / 2;
    static constexpr int32_t PCNT_FULL_RANGE = 32768 * 2;
    int32_t delta = count - last_pcnt_count_;
    if (delta < -PCNT_HALF_RANGE) {
      delta += PCNT_FULL_RANGE;
    } else if (delta > PCNT_HALF_RANGE) {
      delta -= PCNT_FULL_RANGE;
    }

    encoder_accumulator_ += delta;
    last_pcnt_count_ = count;

    status_.encoder_position = (int32_t)encoder_accumulator_;
  } else {
    status_.encoder_position = 0;
  }

  if (config_.enable_closed_loop_control && config_.enable_encoder_feedback) {
    status_.position_error =
        (int32_t)profile_.target_position - status_.encoder_position;
    int32_t error = status_.position_error;
    const int32_t ERROR_THRESHOLD = 10;
    if (abs(error) > ERROR_THRESHOLD && profile_.phase != MotionProfile::IDLE) {
      const double KP = 0.001;
      double correction_factor = 1.0 + (KP * error);
      if (correction_factor > 1.2) correction_factor = 1.2;
      if (correction_factor < 0.8) correction_factor = 0.8;
      profile_.current_freq *= correction_factor;
      if (profile_.current_freq > profile_.max_freq) {
        profile_.current_freq = profile_.max_freq;
      }
      if (profile_.current_freq < 1.0) {
        profile_.current_freq = 1.0;
      }
    }
  } else {
    status_.position_error =
        (int32_t)profile_.target_position - (int32_t)current_position_;
  }
  status_.last_update_ms = millis();

  if (config_.in_position_pin >= 0) {
    status_.position_reached = !readPinSafe(config_.in_position_pin);
  }
  if (config_.brake_pin >= 0) {
    status_.brake_released = !readPinSafe(config_.brake_pin);
  }

  if (alarm_now && !status_.alarm_active && alarm_callback_) {
    alarm_callback_("ALM");
  }
  status_.alarm_active = alarm_now;

  if (status_update_callback_) {
    status_update_callback_(status_);
  }
}

void PulseMotorDriver::startMotionProfile(uint32_t total_pulses, double max_freq,
                                          double accel_rate, double decel_rate,
                                          bool direction) {
  ESP_LOGI(TAG, "[%lu] PulseMotor: [DBG] startMotionProfile ENTER", millis());
  if (accel_rate <= 0.0 || decel_rate <= 0.0) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: Invalid accel/decel rates", millis());
    return;
  }

  double t_accel = max_freq / accel_rate;
  double t_decel = max_freq / decel_rate;
  uint32_t accel_pulses = (uint32_t)(0.5 * accel_rate * t_accel * t_accel);
  uint32_t decel_pulses = (uint32_t)(0.5 * decel_rate * t_decel * t_decel);

  if (accel_pulses + decel_pulses > total_pulses) {
    double scale = sqrt((double)total_pulses / (accel_pulses + decel_pulses));
    accel_pulses = (uint32_t)(accel_pulses * scale * scale);
    decel_pulses = total_pulses - accel_pulses;
    max_freq = sqrt(2.0 * accel_rate * accel_pulses);
    DEBUG_PRINT("PulseMotor: Triangular profile, max_freq=");
    DEBUG_PRINTLN(max_freq);
  }

  uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;

  profile_.total_pulses = total_pulses;
  profile_.accel_pulses = accel_pulses;
  profile_.decel_pulses = decel_pulses;
  profile_.cruise_pulses = cruise_pulses;
  profile_.pulses_generated = 0;
  profile_.fractional_pulses = 0.0;
  profile_.current_freq = 1.0;
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
  delayMicroseconds(10);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  const double LEDC_MIN_HZ = 500.0;
#else
  const double LEDC_MIN_HZ = 40000.0;
#endif
  double init_freq = profile_.current_freq;
  uint8_t init_resolution = 1;
  if (init_freq < LEDC_MIN_HZ) {
    init_freq = LEDC_MIN_HZ;
  }
  profile_.current_freq = init_freq;

  uint8_t max_duty = (1 << init_resolution) - 1;
  uint8_t duty = (max_duty > 0) ? ((max_duty + 1) / 2) : 1;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  (void)max_duty;
  (void)duty;
#endif
  uint32_t init_freq_hz = (uint32_t)(init_freq + 0.5);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ESP_LOGI(TAG, "[%lu] PulseMotor: [DBG] Stopping LEDC output", millis());
  ledcWrite(config_.ledc_pulse_pin, 0);
  ESP_LOGI(TAG, "[%lu] PulseMotor: [DBG] Detaching LEDC", millis());
  ledcDetach(config_.ledc_pulse_pin);
  ESP_LOGI(TAG, "[%lu] PulseMotor: [DBG] About to call ledcAttach", millis());
  if (!ledcAttach(config_.ledc_pulse_pin, init_freq_hz, init_resolution)) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: ERROR - ledcAttach failed", millis());
    return;
  }
  ESP_LOGI(TAG, "[%lu] PulseMotor: [DBG] ledcAttach OK, calling ledcWriteTone at %lu Hz",
           millis(), (unsigned long)init_freq_hz);
  (void)ledcWriteTone(config_.ledc_pulse_pin, init_freq_hz);
  ESP_LOGI(TAG, "[%lu] PulseMotor: [DBG] ledcWriteTone done", millis());
#else
  ESP_LOGI(TAG, "[%lu] PulseMotor: [DBG] Calling ledcWrite(0)", millis());
  ledcWrite(config_.ledc_channel, 0);
  delayMicroseconds(100);
  ledcSetup(config_.ledc_channel, init_freq_hz, init_resolution);
  ledcAttachPin(config_.ledc_pulse_pin, config_.ledc_channel);
  ledcWrite(config_.ledc_channel, duty);
#endif

  esp_err_t start_err = esp_timer_start_periodic(ramp_timer_, RAMP_INTERVAL_US);
  if (start_err != ESP_OK) {
    ESP_LOGI(TAG, "[%lu] PulseMotor: ERROR - Failed to start timer!", millis());
  }
}

// ============================================================================
// POSITION CONTROL
// ============================================================================

bool PulseMotorDriver::moveToPosition(uint32_t target_position, uint32_t speed,
                                      uint32_t acceleration, uint32_t deceleration) {
  ESP_LOGI(TAG, "[%lu] PulseMotor: moveToPosition ENTER", millis());
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (isAlarmActive()) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (!initialized_ || !enabled_) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (motion_active_) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
  if (speed == 0 || acceleration == 0 || deceleration == 0) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  target_position_ = target_position;

  uint32_t current_pos = current_position_;
  if (config_.enable_closed_loop_control && config_.enable_encoder_feedback) {
    current_pos = (uint32_t)encoder_accumulator_;
    current_position_ = current_pos;
  }

  int64_t delta = (int64_t)target_position - (int64_t)current_pos;
  uint32_t pulse_count = (uint32_t)std::abs(delta);
  bool direction = (delta >= 0);

  if (pulse_count == 0) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return true;
  }

  status_.position_error = delta;

  startMotionProfile(pulse_count, (double)speed, (double)acceleration,
                     (double)deceleration, direction);
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  return true;
}

bool PulseMotorDriver::moveRelative(int64_t delta_counts, uint32_t speed,
                                    uint32_t acceleration, uint32_t deceleration) {
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_ || !enabled_) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  int64_t target_signed = (int64_t)current_position_ + delta_counts;
  if (target_signed < 0 || target_signed > UINT32_MAX) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  uint32_t target = (uint32_t)target_signed;
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  return moveToPosition(target, speed, acceleration, deceleration);
}

bool PulseMotorDriver::stopMotion(uint32_t deceleration) {
  (void)deceleration;
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }

  if (motion_active_ && deceleration > 0 && current_speed_pps_ > 0) {
    profile_.decel_rate = (double)deceleration;
    profile_.phase = MotionProfile::DECEL;
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return true;
  }

  motion_active_ = false;
  current_speed_pps_ = 0;
  profile_.phase = MotionProfile::IDLE;

  if (ramp_timer_) {
    esp_timer_stop(ramp_timer_);
  }
  stopLEDC();
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  ESP_LOGI(TAG, "[%lu] PulseMotor: Motion stopped at position=%lu",
           millis(), (unsigned long)current_position_);
  return true;
}

bool PulseMotorDriver::startHoming(uint32_t speed) {
  (void)speed;
  ESP_LOGW(TAG, "[%lu] PulseMotor: startHoming is disabled; use Gantry layer", millis());
  return false;
}

bool PulseMotorDriver::measureTravelDistance(uint32_t speed, uint32_t acceleration,
                                             uint32_t deceleration,
                                             uint32_t timeout_ms) {
  (void)speed;
  (void)acceleration;
  (void)deceleration;
  (void)timeout_ms;
  travel_distance_valid_ = false;
  travel_distance_steps_ = 0;
  ESP_LOGW(TAG, "[%lu] PulseMotor: measureTravelDistance disabled; use Gantry layer", millis());
  return false;
}

bool PulseMotorDriver::hasTravelDistance() const { return false; }
uint32_t PulseMotorDriver::getTravelDistance() const { return 0; }

bool PulseMotorDriver::eStop() {
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (!initialized_) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return false;
  }
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
  stopMotion(0);
  disable();
  DEBUG_PRINTLN("PulseMotor: EMERGENCY STOP");
  return true;
}

// ============================================================================
// STATUS
// ============================================================================

DriveStatus PulseMotorDriver::getStatus() const { return status_; }
uint32_t PulseMotorDriver::getPosition() const { return current_position_; }

void PulseMotorDriver::setPosition(uint32_t position) {
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  current_position_ = position;
  status_.current_position = position;
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
}

uint32_t PulseMotorDriver::getSpeed() const {
  if (motion_active_) {
    return current_speed_pps_;
  }
  return 0;
}

bool PulseMotorDriver::isMotionActive() const { return motion_active_; }

int32_t PulseMotorDriver::getEncoderPosition() const {
  if (!config_.enable_encoder_feedback) {
    return 0;
  }
  return (int32_t)encoder_accumulator_;
}

void PulseMotorDriver::resetEncoderPosition() {
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
  if (config_.enable_encoder_feedback) {
    if (pcnt_unit_handle_) {
      pcnt_unit_clear_count(pcnt_unit_handle_);
    }
    encoder_accumulator_ = 0;
    last_pcnt_count_ = 0;
  }
#if PULSE_MOTOR_USE_FREERTOS
  xSemaphoreGive(mutex_);
#endif
}

// ============================================================================
// CALLBACKS
// ============================================================================

void PulseMotorDriver::setAlarmCallback(AlarmCallback callback) {
  alarm_callback_ = callback;
}
void PulseMotorDriver::setPositionReachedCallback(PositionReachedCallback callback) {
  position_reached_callback_ = callback;
}
void PulseMotorDriver::setStatusUpdateCallback(StatusUpdateCallback callback) {
  status_update_callback_ = callback;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

const DriverConfig &PulseMotorDriver::getConfig() const { return config_; }
void PulseMotorDriver::setConfig(const DriverConfig &config) { config_ = config; }

String PulseMotorDriver::getConfigStatus() const {
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "PulseMotor v%s\nInitialized: %s\nEnabled: %s\nPosition: %" PRIu32
           "\nSpeed: %" PRIu32 " pps",
           PULSE_MOTOR_VERSION_STRING, initialized_ ? "Yes" : "No",
           enabled_ ? "Yes" : "No", current_position_, current_speed_pps_);
  return String(buffer);
}

String PulseMotorDriver::getVersion() { return String(PULSE_MOTOR_VERSION_STRING); }

String PulseMotorDriver::getDriverInfo() const {
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "PulseMotor Driver v%s | Pulse=%d Dir=%d Enable=%d Alarm=%d | "
           "MaxFreq=%" PRIu32 " PPR=%" PRIu32,
           PULSE_MOTOR_VERSION_STRING, config_.pulse_pin, config_.dir_pin,
           config_.enable_pin, config_.alarm_pin, config_.max_pulse_freq,
           config_.encoder_ppr);
  return String(buffer);
}

// ============================================================================
// PRIVATE HELPERS
// ============================================================================

bool PulseMotorDriver::writeNamedOutputPin(int pin, bool state, bool use_dir_invert) {
  if (!initialized_ || pin < 0) {
    return false;
  }
  bool physical_state = use_dir_invert
                            ? (config_.invert_dir_pin ? !state : state)
                            : (config_.invert_output_logic ? !state : state);
  return writePinSafe(pin, physical_state);
}

bool PulseMotorDriver::setDirectionPin(bool state) {
  return writeNamedOutputPin(config_.dir_pin, state, true);
}

bool PulseMotorDriver::setEnablePin(bool state) {
  return writeNamedOutputPin(config_.enable_pin, state, false);
}
