/**
 * @file PulseMotor.cpp
 * @brief Generic pulse+direction motor driver implementation.
 */

#include "PulseMotor.h"
#include "gpio_expander.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <esp_log.h>
#include <esp_timer.h>
#include <inttypes.h>

using namespace PulseMotor;
static const char* TAG = "PulseMotor";

// Version
#define PULSE_MOTOR_VERSION_MAJOR "2"
#define PULSE_MOTOR_VERSION_MINOR "1"
#define PULSE_MOTOR_VERSION_PATCH "0"
#define PULSE_MOTOR_VERSION \
    PULSE_MOTOR_VERSION_MAJOR "." PULSE_MOTOR_VERSION_MINOR "." PULSE_MOTOR_VERSION_PATCH

#ifndef PULSE_MOTOR_DEBUG
#define PULSE_MOTOR_DEBUG 1
#endif

#if PULSE_MOTOR_DEBUG
#define DEBUG_LOGD(fmt, ...) ESP_LOGD(TAG, fmt, ##__VA_ARGS__)
#else
#define DEBUG_LOGD(fmt, ...) ((void)0)
#endif

// Monotonic millisecond clock (replaces Arduino millis()). Returns unsigned
// long so it prints cleanly with "%lu" in existing ESP_LOG call-sites.
static inline unsigned long pm_millis() {
    return (unsigned long)(esp_timer_get_time() / 1000LL);
}

// Blocking microsecond delay that does not pump FreeRTOS scheduler. Safe at
// the short timescales used here (<=1ms).
static inline void pm_delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

// LEDC speed mode: LOW_SPEED works on ESP32 classic and all -S/-C variants.
static constexpr ledc_mode_t PM_LEDC_MODE = LEDC_LOW_SPEED_MODE;

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
    io_conf.mode         = isOutput ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
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
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

PulseMotorDriver::PulseMotorDriver(const DriverConfig& config)
  : config_(config),
    status_(),
    initialized_(false),
    enabled_(false),
    motion_active_(false),
    current_direction_(true),
    homing_active_(false),
    travel_distance_valid_(false),
    travel_distance_steps_(0),
    current_position_(0),
    target_position_(0),
    current_speed_pps_(0),
    last_status_update_ms_(0),
    encoder_accumulator_(0),
    last_pcnt_count_(0),
    pcnt_unit_handle_(nullptr),
    pcnt_chan_a_handle_(nullptr),
    pcnt_chan_b_handle_(nullptr),
    resolved_pulse_gpio_(-1),
    ledc_chan_(LEDC_CHANNEL_0),
    ledc_tmr_(LEDC_TIMER_0),
    ledc_ready_(false),
    ramp_timer_(nullptr),
    alarm_callback_(nullptr),
    position_reached_callback_(nullptr),
    status_update_callback_(nullptr)
#if PULSE_MOTOR_USE_FREERTOS
    , mutex_(nullptr)
#endif
{
    status_.position_reached = false;
    status_.brake_released   = false;
    status_.alarm_active     = false;
    status_.servo_enabled    = false;
    status_.current_position = 0;
    status_.encoder_position = 0;
    status_.position_error   = 0;
    status_.current_speed    = 0;
    status_.last_update_ms   = 0;

    esp_timer_create_args_t ramp_args = {};
    ramp_args.callback = &PulseMotorDriver::rampTimerCallback;
    ramp_args.arg      = this;
    ramp_args.name     = "pulse_motor_ramp";
    esp_err_t timer_err = esp_timer_create(&ramp_args, &ramp_timer_);
    ESP_LOGI(TAG, "[%lu] PulseMotorDriver: esp_timer_create returned %d, ramp_timer_=%p",
             pm_millis(), (int)timer_err, ramp_timer_);
    if (timer_err != ESP_OK) {
        ESP_LOGE(TAG, "[%lu] PulseMotorDriver: Failed to create ramp timer!", pm_millis());
        ramp_timer_ = nullptr;
    }

    profile_.phase = MotionProfile::IDLE;

#if PULSE_MOTOR_USE_FREERTOS
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        DEBUG_LOGD("PulseMotorDriver: Failed to create mutex!");
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
        ramp_timer_ = nullptr;
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
        mutex_ = nullptr;
    }
#endif
}

// ============================================================================
// INITIALISATION
// ============================================================================

bool PulseMotorDriver::initialize() {
    if (initialized_) {
        DEBUG_LOGD("PulseMotorDriver: Already initialized");
        return false;
    }

    DEBUG_LOGD("PulseMotorDriver: initialising (pure ESP-IDF LEDC API)");

    if (!ramp_timer_) {
        DEBUG_LOGD("PulseMotorDriver: Timer not created - init failed");
        return false;
    }

#if PULSE_MOTOR_USE_FREERTOS
    if (!mutex_) {
        DEBUG_LOGD("PulseMotorDriver: Mutex not created - init failed");
        return false;
    }
#endif

    if (config_.pulse_mode != PulseMode::PULSE_DIRECTION) {
        DEBUG_LOGD("PulseMotorDriver: Only PULSE_DIRECTION mode is supported");
        return false;
    }

    if (config_.pulse_pin < 0) {
        DEBUG_LOGD("PulseMotorDriver: pulse_pin must be configured");
        return false;
    }
    if (config_.dir_pin < 0) {
        DEBUG_LOGD("PulseMotorDriver: dir_pin must be configured");
        return false;
    }
    if (config_.enable_pin < 0) {
        DEBUG_LOGD("PulseMotorDriver: enable_pin must be configured");
        return false;
    }

    if (config_.ledc_channel < 0 || config_.ledc_channel > (int)LEDC_CHANNEL_MAX - 1) {
        ESP_LOGE(TAG, "PulseMotorDriver: ledc_channel out of range (%d)",
                 config_.ledc_channel);
        return false;
    }

    if (config_.enable_closed_loop_control && !config_.enable_encoder_feedback) {
        DEBUG_LOGD("PulseMotorDriver: closed-loop requires encoder feedback");
        return false;
    }

    // Configure all outputs (idle LOW, honoring invert_output_logic).
    auto configOutput = [this](int pin) -> bool {
        if (pin < 0) return true;
        if (!configurePinDirectionSafe(pin, true)) return false;
        const bool idle_level = config_.invert_output_logic; // logical low -> physical high
        return writePinSafe(pin, idle_level);
    };

    if (!configOutput(config_.pulse_pin))       return false;
    if (!configOutput(config_.dir_pin))         return false;
    if (!configOutput(config_.enable_pin))      return false;
    if (!configOutput(config_.alarm_reset_pin)) return false;
    if (!configOutput(config_.brake_pin))       return false;
    if (!configOutput(config_.inhibit_pin))     return false;

    // Inputs
    auto configInput = [](int pin) -> bool {
        if (pin < 0) return true;
        return configurePinDirectionSafe(pin, false);
    };
    if (!configInput(config_.in_position_pin))  return false;
    if (!configInput(config_.brake_status_pin)) return false;
    if (!configInput(config_.alarm_pin))        return false;
    if (!configInput(config_.encoder_a_pin))    return false;
    if (!configInput(config_.encoder_b_pin))    return false;
    if (!configInput(config_.encoder_z_pin))    return false;
    if (!configInput(config_.limit_min_pin))    return false;
    if (!configInput(config_.limit_max_pin))    return false;

    // LEDC pulse channel (pure ESP-IDF)
    resolved_pulse_gpio_ = resolveDirectGpioPin(config_.pulse_pin);
    if (resolved_pulse_gpio_ < 0) {
        DEBUG_LOGD("PulseMotorDriver: pulse_pin must resolve to a direct GPIO");
        return false;
    }

    // Resolve LEDC channel + timer. If config_.ledc_timer < 0, auto-derive a
    // timer index from the channel so each axis gets its own timer (required
    // because changing freq on a shared timer affects all sibling channels).
    ledc_chan_ = (ledc_channel_t)config_.ledc_channel;
    int timer_idx = (config_.ledc_timer >= 0)
        ? config_.ledc_timer
        : (config_.ledc_channel % LEDC_TIMER_MAX);
    if (timer_idx < 0 || timer_idx >= (int)LEDC_TIMER_MAX) {
        ESP_LOGE(TAG, "PulseMotorDriver: ledc_timer out of range (%d)", timer_idx);
        return false;
    }
    ledc_tmr_ = (ledc_timer_t)timer_idx;

    uint8_t res_bits = config_.ledc_resolution;
    if (res_bits < 1) res_bits = 1;
    if (res_bits > (uint8_t)LEDC_TIMER_14_BIT) res_bits = (uint8_t)LEDC_TIMER_14_BIT;

    ledc_timer_config_t ledc_tcfg = {};
    ledc_tcfg.speed_mode      = PM_LEDC_MODE;
    ledc_tcfg.timer_num       = ledc_tmr_;
    ledc_tcfg.duty_resolution = (ledc_timer_bit_t)res_bits;
    ledc_tcfg.freq_hz         = 1000;
    ledc_tcfg.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t tc_err = ledc_timer_config(&ledc_tcfg);
    if (tc_err != ESP_OK) {
        ESP_LOGE(TAG, "PulseMotorDriver: ledc_timer_config failed (%d)", (int)tc_err);
        return false;
    }

    ledc_channel_config_t ledc_ccfg = {};
    ledc_ccfg.speed_mode     = PM_LEDC_MODE;
    ledc_ccfg.channel        = ledc_chan_;
    ledc_ccfg.timer_sel      = ledc_tmr_;
    ledc_ccfg.intr_type      = LEDC_INTR_DISABLE;
    ledc_ccfg.gpio_num       = resolved_pulse_gpio_;
    ledc_ccfg.duty           = 0;
    ledc_ccfg.hpoint         = 0;
    esp_err_t cc_err = ledc_channel_config(&ledc_ccfg);
    if (cc_err != ESP_OK) {
        ESP_LOGE(TAG, "PulseMotorDriver: ledc_channel_config failed (%d)", (int)cc_err);
        return false;
    }
    ledc_set_duty(PM_LEDC_MODE, ledc_chan_, 0);
    ledc_update_duty(PM_LEDC_MODE, ledc_chan_);
    ledc_ready_ = true;
    DEBUG_LOGD("PulseMotorDriver: LEDC channel initialized (ch=%d tmr=%d res=%u)",
               (int)ledc_chan_, (int)ledc_tmr_, (unsigned)res_bits);

    // PCNT encoder
    const int enc_a_gpio = resolveDirectGpioPin(config_.encoder_a_pin);
    const int enc_b_gpio = resolveDirectGpioPin(config_.encoder_b_pin);
    if (config_.enable_encoder_feedback && enc_a_gpio > 0 && enc_b_gpio > 0) {
        pcnt_unit_config_t unit_config = {};
        unit_config.low_limit  = -32768;
        unit_config.high_limit = 32767;
        if (pcnt_new_unit(&unit_config, &pcnt_unit_handle_) != ESP_OK) {
            DEBUG_LOGD("%s", "PulseMotorDriver: Failed to allocate PCNT unit");
            return false;
        }

        pcnt_chan_config_t chan_a_config = {};
        chan_a_config.edge_gpio_num  = enc_a_gpio;
        chan_a_config.level_gpio_num = enc_b_gpio;
        if (pcnt_new_channel(pcnt_unit_handle_, &chan_a_config, &pcnt_chan_a_handle_) != ESP_OK) {
            DEBUG_LOGD("%s", "PulseMotorDriver: Failed to allocate PCNT channel A");
            return false;
        }

        pcnt_chan_config_t chan_b_config = {};
        chan_b_config.edge_gpio_num  = enc_b_gpio;
        chan_b_config.level_gpio_num = enc_a_gpio;
        if (pcnt_new_channel(pcnt_unit_handle_, &chan_b_config, &pcnt_chan_b_handle_) != ESP_OK) {
            DEBUG_LOGD("%s", "PulseMotorDriver: Failed to allocate PCNT channel B");
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

        DEBUG_LOGD("PulseMotorDriver: Encoder PCNT initialized");
    }

    initialized_           = true;
    last_status_update_ms_ = pm_millis();
    DEBUG_LOGD("PulseMotorDriver: Initialized successfully");
    return true;
}

// ============================================================================
// ENABLE / DISABLE
// ============================================================================

bool PulseMotorDriver::enable() {
    ESP_LOGI(TAG, "[%lu] PulseMotorDriver: enable() called, init=%d en=%d",
             pm_millis(), initialized_, enabled_);
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
    if (!initialized_ || enabled_ || isAlarmActive()) {
#if PULSE_MOTOR_USE_FREERTOS
        xSemaphoreGive(mutex_);
#endif
        return false;
    }

    if (!setEnablePin(true)) {
#if PULSE_MOTOR_USE_FREERTOS
        xSemaphoreGive(mutex_);
#endif
        return false;
    }
    enabled_              = true;
    status_.servo_enabled = true;
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    ESP_LOGI(TAG, "[%lu] PulseMotorDriver: enable() OK", pm_millis());
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
#if PULSE_MOTOR_USE_FREERTOS
        xSemaphoreGive(mutex_);
#endif
        return false;
    }
    enabled_              = false;
    status_.servo_enabled = false;
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    DEBUG_LOGD("%s", "PulseMotorDriver: Disabled");
    return true;
}

bool PulseMotorDriver::isEnabled() const { return enabled_; }

bool PulseMotorDriver::isAlarmActive() const {
    if (!initialized_ || config_.alarm_pin < 0) {
        return false;
    }
    // ALM is active-low: LOW on wire = logical alarm active.
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
    writePinLogical(config_.alarm_reset_pin, true, false);
    pm_delay_us(100);
    writePinLogical(config_.alarm_reset_pin, false, false);
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    DEBUG_LOGD("%s", "PulseMotorDriver: Alarm reset pulse sent");
    return true;
}

void PulseMotorDriver::stopLEDC() {
    if (!ledc_ready_) return;
    ledc_set_duty(PM_LEDC_MODE, ledc_chan_, 0);
    ledc_update_duty(PM_LEDC_MODE, ledc_chan_);
}

void PulseMotorDriver::stopMotionFromIsr(const char* reason, bool set_home) {
    if (reason) {
        ESP_LOGI(TAG, "[%lu] PulseMotorDriver: %s", pm_millis(), reason);
    }
    if (set_home) {
        homing_active_    = false;
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
    ESP_LOGI(TAG, "[%lu] PulseMotorDriver: Motion stopped at position=%lu",
             pm_millis(), (unsigned long)current_position_);
    if (position_reached_callback_) {
        position_reached_callback_(current_position_);
    }
}

// ============================================================================
// MOTION PROFILE
// ============================================================================

void PulseMotorDriver::rampTimerCallback(void* arg) {
    static uint32_t callback_count = 0;
    callback_count++;
    if (callback_count % 100 == 0) {
        ESP_LOGI(TAG, "[CB] rampTimerCallback: call #%lu",
                 (unsigned long)callback_count);
    }
    auto* drv = static_cast<PulseMotorDriver*>(arg);
    drv->updateMotionProfile();
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

    const int64_t now_us = esp_timer_get_time();
    const double  dt_s   = (now_us - profile_.last_update_us) / 1000000.0;
    profile_.last_update_us = now_us;

    const double   pulses_exact = profile_.current_freq * dt_s + profile_.fractional_pulses;
    const uint32_t pulses_this_interval = (uint32_t)pulses_exact;
    profile_.fractional_pulses = pulses_exact - pulses_this_interval;
    profile_.pulses_generated += pulses_this_interval;

    if (profile_.direction) {
        current_position_ += pulses_this_interval;
    } else {
        current_position_ -= pulses_this_interval;
    }

    const uint32_t remaining =
        (profile_.pulses_generated >= profile_.total_pulses)
            ? 0
            : (profile_.total_pulses - profile_.pulses_generated);

    // Minimum frequency that the LEDC timer can reliably produce at the
    // configured resolution. Below this the ramp finishes the decel phase and
    // the LEDC output is muted by stopLEDC().
    const double ABSOLUTE_MIN_FREQ = 5000.0;

    switch (profile_.phase) {
    case MotionProfile::ACCEL:
        profile_.current_freq += profile_.accel_rate * dt_s;
        if (profile_.current_freq >= profile_.max_freq) {
            profile_.current_freq = profile_.max_freq;
            profile_.phase = MotionProfile::CRUISE;
        }
        if (remaining <= profile_.decel_pulses) {
            profile_.phase = MotionProfile::DECEL;
        }
        break;
    case MotionProfile::CRUISE:
        if (remaining <= profile_.decel_pulses) {
            profile_.phase = MotionProfile::DECEL;
        }
        break;
    case MotionProfile::DECEL:
        profile_.current_freq -= profile_.decel_rate * dt_s;
        if (profile_.current_freq > 0 && profile_.current_freq < ABSOLUTE_MIN_FREQ) {
            profile_.current_freq = ABSOLUTE_MIN_FREQ;
        }
        if (profile_.current_freq <= 0 || remaining == 0) {
            profile_.current_freq = 0;
            profile_.phase        = MotionProfile::IDLE;
            if (profile_.pulses_generated >= profile_.total_pulses) {
                if (!config_.enable_closed_loop_control || !config_.enable_encoder_feedback) {
                    current_position_ = profile_.target_position;
                }
            }
            esp_timer_stop(ramp_timer_);
            stopLEDC();
            handleMoveComplete();
            return;
        }
        break;
    default:
        break;
    }

    if (profile_.current_freq > 0 && ledc_ready_) {
        const double LEDC_MIN_HZ = 1000.0;
        double freq = profile_.current_freq;
        if (freq < LEDC_MIN_HZ) {
            freq                  = LEDC_MIN_HZ;
            profile_.current_freq = LEDC_MIN_HZ;
        }
        uint32_t freq_hz = (uint32_t)(freq + 0.5);
        if (freq_hz < (uint32_t)LEDC_MIN_HZ) {
            freq_hz = (uint32_t)LEDC_MIN_HZ;
        }
        // Update timer frequency only if it materially changed; and keep duty
        // at ~50% (square wave) which is what a pulse-train driver expects.
        ledc_set_freq(PM_LEDC_MODE, ledc_tmr_, freq_hz);
        const uint32_t max_duty = (1u << config_.ledc_resolution) - 1u;
        const uint32_t duty     = (max_duty > 0) ? ((max_duty + 1u) / 2u) : 1u;
        ledc_set_duty(PM_LEDC_MODE, ledc_chan_, duty);
        ledc_update_duty(PM_LEDC_MODE, ledc_chan_);
        current_speed_pps_ = (uint32_t)profile_.current_freq;
    }

    status_.current_position = current_position_;
    status_.current_speed    = current_speed_pps_;
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
        const int32_t error = status_.position_error;
        const int32_t ERROR_THRESHOLD = 10;
        if (abs(error) > ERROR_THRESHOLD && profile_.phase != MotionProfile::IDLE) {
            const double KP = 0.001;
            double correction = 1.0 + (KP * error);
            if (correction > 1.2) correction = 1.2;
            if (correction < 0.8) correction = 0.8;
            profile_.current_freq *= correction;
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
    status_.last_update_ms = pm_millis();

    if (config_.in_position_pin >= 0) {
        status_.position_reached = !readPinSafe(config_.in_position_pin);
    }
    if (config_.brake_status_pin >= 0) {
        status_.brake_released = !readPinSafe(config_.brake_status_pin);
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
    if (accel_rate <= 0.0 || decel_rate <= 0.0) {
        ESP_LOGE(TAG, "[%lu] PulseMotorDriver: Invalid accel/decel rates", pm_millis());
        return;
    }

    double   t_accel      = max_freq / accel_rate;
    double   t_decel      = max_freq / decel_rate;
    uint32_t accel_pulses = (uint32_t)(0.5 * accel_rate * t_accel * t_accel);
    uint32_t decel_pulses = (uint32_t)(0.5 * decel_rate * t_decel * t_decel);

    if (accel_pulses + decel_pulses > total_pulses) {
        double scale = sqrt((double)total_pulses / (accel_pulses + decel_pulses));
        accel_pulses = (uint32_t)(accel_pulses * scale * scale);
        decel_pulses = total_pulses - accel_pulses;
        max_freq     = sqrt(2.0 * accel_rate * accel_pulses);
    }

    uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;

    profile_.total_pulses     = total_pulses;
    profile_.accel_pulses     = accel_pulses;
    profile_.decel_pulses     = decel_pulses;
    profile_.cruise_pulses    = cruise_pulses;
    profile_.pulses_generated = 0;
    profile_.fractional_pulses = 0.0;
    profile_.current_freq     = 1.0;
    profile_.max_freq         = max_freq;
    profile_.accel_rate       = accel_rate;
    profile_.decel_rate       = decel_rate;
    profile_.last_update_us   = esp_timer_get_time();
    profile_.phase            = MotionProfile::ACCEL;
    profile_.direction        = direction;
    profile_.target_position  = target_position_;

    motion_active_     = true;
    current_direction_ = direction;
    setDirectionPin(direction);
    pm_delay_us(10);

    if (!ledc_ready_) {
        ESP_LOGE(TAG, "[%lu] PulseMotorDriver: LEDC not ready, cannot start motion",
                 pm_millis());
        return;
    }

    const double LEDC_MIN_HZ = 500.0;
    double init_freq = profile_.current_freq;
    if (init_freq < LEDC_MIN_HZ) {
        init_freq = LEDC_MIN_HZ;
    }
    profile_.current_freq = init_freq;

    const uint32_t init_freq_hz = (uint32_t)(init_freq + 0.5);
    const uint32_t max_duty     = (1u << config_.ledc_resolution) - 1u;
    const uint32_t duty         = (max_duty > 0) ? ((max_duty + 1u) / 2u) : 1u;

    // Silence the channel, retune the timer to the start-of-ramp frequency,
    // then re-arm the channel at ~50% duty. Pure ESP-IDF path - no Arduino.
    ledc_set_duty(PM_LEDC_MODE, ledc_chan_, 0);
    ledc_update_duty(PM_LEDC_MODE, ledc_chan_);
    pm_delay_us(100);
    esp_err_t freq_err = ledc_set_freq(PM_LEDC_MODE, ledc_tmr_, init_freq_hz);
    if (freq_err != ESP_OK) {
        ESP_LOGE(TAG, "[%lu] PulseMotorDriver: ledc_set_freq failed (%d)",
                 pm_millis(), (int)freq_err);
        return;
    }
    ledc_set_duty(PM_LEDC_MODE, ledc_chan_, duty);
    ledc_update_duty(PM_LEDC_MODE, ledc_chan_);

    esp_err_t start_err = esp_timer_start_periodic(ramp_timer_, RAMP_INTERVAL_US);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "[%lu] PulseMotorDriver: timer start failed (%d)",
                 pm_millis(), start_err);
    }
}

// ============================================================================
// POSITION CONTROL
// ============================================================================

bool PulseMotorDriver::moveToPosition(uint32_t target_position, uint32_t speed,
                                      uint32_t acceleration, uint32_t deceleration) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
    if (isAlarmActive() || !initialized_ || !enabled_ || motion_active_ ||
        speed == 0 || acceleration == 0 || deceleration == 0) {
#if PULSE_MOTOR_USE_FREERTOS
        xSemaphoreGive(mutex_);
#endif
        return false;
    }

    target_position_ = target_position;

    uint32_t current_pos = current_position_;
    if (config_.enable_closed_loop_control && config_.enable_encoder_feedback) {
        current_pos       = (uint32_t)encoder_accumulator_;
        current_position_ = current_pos;
    }
    const int64_t  delta       = (int64_t)target_position - (int64_t)current_pos;
    const uint32_t pulse_count = (uint32_t)std::abs(delta);
    const bool     direction   = (delta >= 0);

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
    const int64_t target_signed = (int64_t)current_position_ + delta_counts;
    if (target_signed < 0 || target_signed > UINT32_MAX) {
#if PULSE_MOTOR_USE_FREERTOS
        xSemaphoreGive(mutex_);
#endif
        return false;
    }
    const uint32_t target = (uint32_t)target_signed;
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return moveToPosition(target, speed, acceleration, deceleration);
}

bool PulseMotorDriver::stopMotion(uint32_t deceleration) {
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
        profile_.phase      = MotionProfile::DECEL;
#if PULSE_MOTOR_USE_FREERTOS
        xSemaphoreGive(mutex_);
#endif
        return true;
    }
    motion_active_     = false;
    current_speed_pps_ = 0;
    profile_.phase     = MotionProfile::IDLE;
    if (ramp_timer_) {
        esp_timer_stop(ramp_timer_);
    }
    stopLEDC();
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
    return true;
}

bool PulseMotorDriver::startHoming(uint32_t speed) {
    (void)speed;
    ESP_LOGW(TAG, "[%lu] PulseMotorDriver: startHoming not supported - use gantry layer",
             pm_millis());
    return false;
}

bool PulseMotorDriver::measureTravelDistance(uint32_t speed, uint32_t acceleration,
                                             uint32_t deceleration, uint32_t timeout_ms) {
    (void)speed;
    (void)acceleration;
    (void)deceleration;
    (void)timeout_ms;
    travel_distance_valid_ = false;
    travel_distance_steps_ = 0;
    ESP_LOGW(TAG, "[%lu] PulseMotorDriver: measureTravelDistance not supported - use gantry layer",
             pm_millis());
    return false;
}

bool     PulseMotorDriver::hasTravelDistance() const { return false; }
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
    DEBUG_LOGD("%s", "PulseMotorDriver: EMERGENCY STOP");
    return true;
}

// ============================================================================
// STATUS & MONITORING
// ============================================================================

DriveStatus PulseMotorDriver::getStatus() const { return status_; }
uint32_t    PulseMotorDriver::getPosition() const { return current_position_; }

void PulseMotorDriver::setPosition(uint32_t position) {
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreTake(mutex_, portMAX_DELAY);
#endif
    current_position_        = position;
    status_.current_position = position;
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
}

uint32_t PulseMotorDriver::getSpeed() const {
    return motion_active_ ? current_speed_pps_ : 0;
}

bool PulseMotorDriver::isMotionActive() const { return motion_active_; }

int32_t PulseMotorDriver::getEncoderPosition() const {
    if (!config_.enable_encoder_feedback) return 0;
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
        last_pcnt_count_     = 0;
    }
#if PULSE_MOTOR_USE_FREERTOS
    xSemaphoreGive(mutex_);
#endif
}

// ============================================================================
// CALLBACKS
// ============================================================================

void PulseMotorDriver::setAlarmCallback(AlarmCallback cb)                     { alarm_callback_           = cb; }
void PulseMotorDriver::setPositionReachedCallback(PositionReachedCallback cb) { position_reached_callback_ = cb; }
void PulseMotorDriver::setStatusUpdateCallback(StatusUpdateCallback cb)       { status_update_callback_    = cb; }

// ============================================================================
// CONFIG
// ============================================================================

const DriverConfig& PulseMotorDriver::getConfig() const           { return config_; }
void                PulseMotorDriver::setConfig(const DriverConfig& c) { config_ = c; }

std::string PulseMotorDriver::getConfigStatus() const {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "PulseMotor v%s\nInitialized: %s\nEnabled: %s\nPosition: %" PRIu32 "\nSpeed: %" PRIu32 " pps",
             PULSE_MOTOR_VERSION, initialized_ ? "Yes" : "No",
             enabled_ ? "Yes" : "No", current_position_, current_speed_pps_);
    return std::string(buf);
}

std::string PulseMotorDriver::getVersion() { return std::string(PULSE_MOTOR_VERSION); }

std::string PulseMotorDriver::getDriverInfo() const {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "PulseMotor v%s\nPulse: %d | Dir: %d | Enable: %d | Alarm: %d\nMax Freq: %" PRIu32 " Hz | Encoder PPR: %" PRIu32,
             PULSE_MOTOR_VERSION, config_.pulse_pin, config_.dir_pin,
             config_.enable_pin, config_.alarm_pin, config_.max_pulse_freq,
             config_.encoder_ppr);
    return std::string(buf);
}

// ============================================================================
// PIN HELPERS
// ============================================================================

bool PulseMotorDriver::writePinLogical(int pin, bool logical_high, bool is_dir_pin) {
    if (!initialized_ || pin < 0) {
        return false;
    }
    const bool physical = is_dir_pin
        ? (config_.invert_dir_pin ? !logical_high : logical_high)
        : (config_.invert_output_logic ? !logical_high : logical_high);
    return writePinSafe(pin, physical);
}

bool PulseMotorDriver::setDirectionPin(bool state) {
    ESP_LOGI(TAG, "[%lu] setDirectionPin: logical=%d", pm_millis(), state);
    return writePinLogical(config_.dir_pin, state, true);
}

bool PulseMotorDriver::setEnablePin(bool state) {
    return writePinLogical(config_.enable_pin, state, false);
}
