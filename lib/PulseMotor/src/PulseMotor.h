/**
 * @file PulseMotor.h
 * @brief Generic pulse+direction motor driver library for ESP32
 * @version 2.0.0
 *
 * Supports any motor driver whose command interface is a pulse train
 * plus a direction line. Verified hardware targets:
 *   - Bergerda SDF08NK8X servo driver
 *   - Allen-Bradley Kinetix 5100 servo driver (PTI mode)
 *   - Custom pulse-train driver for SCHUNK ERD 04-40-D-H-N rotary module
 *
 * FEATURES:
 * - Pulse/direction position control (deterministic, LEDC-generated).
 * - Open-loop or closed-loop position control (with encoder feedback via PCNT).
 * - Trapezoidal velocity profile (accel / cruise / decel).
 * - Named-field pin configuration (no SDF08 slot-index encoding).
 * - Optional DrivetrainConfig for mm/deg scaling (consumed by caller, not driver).
 * - Thread-safe via FreeRTOS mutex when PULSE_MOTOR_USE_FREERTOS=1.
 *
 * LIMITATIONS:
 * - Only pulse+direction mode is implemented. CW/CCW and quadrature pulse modes
 *   are not yet supported. (Matches previous SDF08NK8X library scope.)
 */

#ifndef PULSE_MOTOR_H
#define PULSE_MOTOR_H

#ifndef PULSE_MOTOR_HOME_ON_BOOT
#define PULSE_MOTOR_HOME_ON_BOOT 1
#endif

#include <driver/pulse_cnt.h>
#include <driver/ledc.h>
#include <esp_timer.h>
#include <stdint.h>
#include <string>

// Optional FreeRTOS support - define PULSE_MOTOR_USE_FREERTOS=1 for thread safety
#ifndef PULSE_MOTOR_USE_FREERTOS
#define PULSE_MOTOR_USE_FREERTOS 1
#endif

// Backward-compat: honor the legacy SDF08NK8X_USE_FREERTOS flag if the caller
// still sets it (e.g. legacy build_flags) so the build does not flip silently
// during the rename.
#if defined(SDF08NK8X_USE_FREERTOS) && !defined(PULSE_MOTOR_FORCE_FREERTOS)
#undef  PULSE_MOTOR_USE_FREERTOS
#define PULSE_MOTOR_USE_FREERTOS SDF08NK8X_USE_FREERTOS
#endif

#if PULSE_MOTOR_USE_FREERTOS
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

namespace PulseMotor {

// ============================================================================
// ENUMS
// ============================================================================

/**
 * @enum PulseMode
 * @brief Pulse input mode selection (driver parameter; typically only
 *        PULSE_DIRECTION is wired in this project).
 */
enum class PulseMode : uint8_t {
    PULSE_DIRECTION = 0, // Standard pulse+direction (2-wire)
    CW_CCW          = 1, // CW/CCW dual pulse (not yet supported)
    QUADRATURE      = 2  // A/B quadrature (not yet supported)
};

/**
 * @enum DrivetrainType
 * @brief Enumerates mechanical drivetrain topologies for unit conversion.
 *        The driver itself does not dispatch on this; it is carried in
 *        DrivetrainConfig for consumer-side mm/deg scaling.
 */
enum DrivetrainType : uint8_t {
    DRIVETRAIN_BALLSCREW     = 1, // Linear, mm per rev = lead
    DRIVETRAIN_BELT          = 2, // Linear, mm per rev = belt_lead_mm_per_rev or pulley_teeth*pitch
    DRIVETRAIN_RACKPINION    = 3, // Linear, mm per rev = pi * pinion_pitch_diameter
    DRIVETRAIN_ROTARY_DIRECT = 4  // Rotary, deg per rev = 360 / output_gear_ratio
};

// ============================================================================
// CONFIGURATION
// ============================================================================

/**
 * @struct DriverConfig
 * @brief Named-field configuration for PulseMotorDriver.
 *
 * All pin fields accept either:
 *   - A direct-ESP32-GPIO pin (plain GPIO number), or
 *   - A GPIO-expander logical pin (MCP23S17 0..15), or
 *   - A GPIO_EXPANDER_DIRECT_PIN()-encoded direct pin when the caller wants to
 *     share the expander's pin-abstraction path.
 *
 * Pins set to -1 are treated as "not wired" and the driver skips them.
 */
struct DriverConfig {
    // -------------------- Output pins (driven by ESP32) --------------------
    int pulse_pin;          // LEDC pulse output (must be a direct GPIO or DIRECT-encoded)
    int dir_pin;            // Direction pin
    int enable_pin;         // Servo-on / enable line
    int alarm_reset_pin;    // ARST / fault-clear output (pulsed on clearAlarm)
    int brake_pin;          // Optional external brake drive
    int inhibit_pin;        // Optional INH / command-pulse prohibit

    // -------------------- Input pins (read from driver) --------------------
    int in_position_pin;    // POS_reached signal (active low)
    int brake_status_pin;   // Brake released signal (active low)
    int alarm_pin;          // ALM signal (active low)
    int encoder_a_pin;      // Quadrature A
    int encoder_b_pin;      // Quadrature B
    int encoder_z_pin;      // Optional index pulse

    // -------------------- Limit switches (optional) --------------------
    // The Gantry library performs its own limit-switch handling, so these are
    // typically left at -1 in the gantry configuration. They exist here for
    // standalone use of the driver.
    int limit_min_pin;
    int limit_max_pin;

    // -------------------- Pulse-train / motor parameters --------------------
    PulseMode pulse_mode;
    uint32_t  max_pulse_freq;   // Hz
    uint32_t  encoder_ppr;      // Pulses per motor revolution (quadrature-decoded)
    double    gear_numerator;   // Driver-side electronic gear
    double    gear_denominator;

    // -------------------- LEDC / PCNT allocation --------------------
    int       ledc_channel;
    int       ledc_timer;       // LEDC timer index (0..3). -1 -> derive from ledc_channel.
    uint8_t   ledc_resolution;  // bits (1..14); driver forces 1-bit at runtime
    int       pcnt_unit;        // Logical PCNT id for app-level bookkeeping

    // -------------------- Feature flags --------------------
    bool enable_encoder_feedback;
    bool enable_closed_loop_control;
    bool invert_output_logic;   // Treat LOW as logical HIGH for outputs
    bool invert_dir_pin;        // Invert DIR logic (swap direction)
    bool home_on_boot;

    // -------------------- Homing & debounce --------------------
    uint32_t homing_speed_pps;
    uint8_t  limit_debounce_cycles;
    uint16_t limit_sample_interval_ms;
    bool     limit_log_changes;

    DriverConfig()
      : pulse_pin(-1), dir_pin(-1), enable_pin(-1), alarm_reset_pin(-1),
        brake_pin(-1), inhibit_pin(-1),
        in_position_pin(-1), brake_status_pin(-1), alarm_pin(-1),
        encoder_a_pin(-1), encoder_b_pin(-1), encoder_z_pin(-1),
        limit_min_pin(-1), limit_max_pin(-1),
        pulse_mode(PulseMode::PULSE_DIRECTION),
        max_pulse_freq(600000), encoder_ppr(12000),
        gear_numerator(1.0), gear_denominator(1.0),
        ledc_channel(0), ledc_timer(-1), ledc_resolution(2), pcnt_unit(0),
        enable_encoder_feedback(false),
        enable_closed_loop_control(false),
        invert_output_logic(true),
        invert_dir_pin(false),
        home_on_boot(PULSE_MOTOR_HOME_ON_BOOT),
        homing_speed_pps(6000),
        limit_debounce_cycles(10),
        limit_sample_interval_ms(3),
        limit_log_changes(true) {}
};

/**
 * @struct DrivetrainConfig
 * @brief Mechanical drivetrain description used by consumers (e.g. Gantry
 *        library axis wrappers) for mm/deg <-> pulses conversion.
 *
 * The driver does NOT consume this struct; it is pulse-domain only. Kept in
 * the PulseMotor header so non-gantry consumers can share the same scaling
 * math.
 */
struct DrivetrainConfig {
    DrivetrainType type;

    // Ballscrew
    float ballscrew_lead_mm;         // mm traveled per screw revolution
    uint32_t ballscrew_critical_rpm; // 0 = unspecified; informational

    // Belt
    float belt_lead_mm_per_rev;      // primary; used if non-zero
    uint32_t belt_pulley_teeth;      // metadata / fallback
    float belt_pitch_mm;             // metadata / fallback

    // Rack & pinion
    float pinion_pitch_diameter_mm;

    // Rotary direct
    float output_gear_ratio;         // output_rev / motor_rev (1.0 = direct)

    // Shared
    uint32_t encoder_ppr;            // Mirrors DriverConfig.encoder_ppr for self-contained math
    float    motor_reducer_ratio;    // Physical reducer at motor output (motor_rev / drivetrain_rev)

    DrivetrainConfig()
      : type(DRIVETRAIN_BALLSCREW),
        ballscrew_lead_mm(0.0f), ballscrew_critical_rpm(0),
        belt_lead_mm_per_rev(0.0f), belt_pulley_teeth(0), belt_pitch_mm(0.0f),
        pinion_pitch_diameter_mm(0.0f),
        output_gear_ratio(1.0f),
        encoder_ppr(0),
        motor_reducer_ratio(1.0f) {}
};

// ---------------------------------------------------------------------------
// Unit conversion helpers (header-only)
// ---------------------------------------------------------------------------

/**
 * @brief Encoder pulses per mm of linear travel.
 * @return 0.0 if the drivetrain is rotary or mis-configured.
 */
inline double pulsesPerMm(const DrivetrainConfig& d) {
    const double ratio = (double)d.encoder_ppr * (double)d.motor_reducer_ratio;
    switch (d.type) {
        case DRIVETRAIN_BALLSCREW:
            return (d.ballscrew_lead_mm > 0.0f) ? ratio / (double)d.ballscrew_lead_mm : 0.0;
        case DRIVETRAIN_BELT: {
            double lead = (d.belt_lead_mm_per_rev > 0.0f)
                ? (double)d.belt_lead_mm_per_rev
                : (double)d.belt_pulley_teeth * (double)d.belt_pitch_mm;
            return (lead > 0.0) ? ratio / lead : 0.0;
        }
        case DRIVETRAIN_RACKPINION: {
            const double circ = 3.14159265358979323846 * (double)d.pinion_pitch_diameter_mm;
            return (circ > 0.0) ? ratio / circ : 0.0;
        }
        default:
            return 0.0;
    }
}

/**
 * @brief Encoder pulses per degree of angular travel.
 * @return 0.0 if the drivetrain is linear or mis-configured.
 */
inline double pulsesPerDeg(const DrivetrainConfig& d) {
    if (d.type != DRIVETRAIN_ROTARY_DIRECT) return 0.0;
    const double ratio = (double)d.encoder_ppr
                       * (double)d.motor_reducer_ratio
                       * (double)d.output_gear_ratio;
    return ratio / 360.0;
}

// ============================================================================
// DRIVE STATUS & MOTION PROFILE
// ============================================================================

struct DriveStatus {
    bool servo_enabled;
    bool position_reached;   // Mirrors in_position_pin (active low on wire)
    bool brake_released;     // Mirrors brake_status_pin (active low on wire)
    bool alarm_active;       // Mirrors alarm_pin (active low on wire)

    uint32_t current_position; // Commanded (pulses)
    int32_t  encoder_position; // Feedback accumulator (pulses)
    int32_t  position_error;   // target - current

    uint32_t current_speed;    // pps
    uint32_t last_update_ms;
};

struct MotionProfile {
    enum Phase : uint8_t { IDLE = 0, ACCEL, CRUISE, DECEL };

    uint32_t total_pulses;
    uint32_t accel_pulses;
    uint32_t decel_pulses;
    uint32_t cruise_pulses;
    uint32_t pulses_generated;
    uint32_t target_position;
    double   current_freq;
    double   max_freq;
    double   accel_rate;
    double   decel_rate;
    double   fractional_pulses;
    int64_t  last_update_us;
    Phase    phase;
    bool     direction;
};

// ============================================================================
// CALLBACKS
// ============================================================================

using AlarmCallback           = void (*)(const char* alarm_code);
using PositionReachedCallback = void (*)(uint32_t position);
using StatusUpdateCallback    = void (*)(const DriveStatus& status);

// ============================================================================
// PULSE MOTOR DRIVER
// ============================================================================

/**
 * @class PulseMotorDriver
 * @brief Generic pulse+direction motor driver.
 */
class PulseMotorDriver {
public:
    explicit PulseMotorDriver(const DriverConfig& config);
    ~PulseMotorDriver();

    // ---------- Initialisation ----------
    bool initialize();

    // ---------- Servo control ----------
    bool enable();
    bool disable();
    bool isEnabled() const;
    bool isMotionActive() const;

    bool moveToPosition(uint32_t target_position,
                        uint32_t max_speed    = 10000,
                        uint32_t acceleration = 5000,
                        uint32_t deceleration = 5000);
    bool moveRelative(int64_t delta_counts,
                      uint32_t max_speed    = 10000,
                      uint32_t acceleration = 5000,
                      uint32_t deceleration = 5000);
    bool stopMotion(uint32_t deceleration = 50000);

    // Legacy hooks retained but no-op (gantry layer owns homing/travel logic)
    bool startHoming(uint32_t speed = 10000);
    bool isHoming() const { return homing_active_; }
    bool measureTravelDistance(uint32_t speed        = 10000,
                               uint32_t acceleration = 10000,
                               uint32_t deceleration = 10000,
                               uint32_t timeout_ms   = 60000);
    bool     hasTravelDistance() const;
    uint32_t getTravelDistance() const;

    bool eStop();

    // ---------- Monitoring ----------
    DriveStatus getStatus() const;
    uint32_t    getPosition() const;
    void        setPosition(uint32_t position);
    uint32_t    getSpeed() const;
    bool        isAlarmActive() const;
    bool        clearAlarm();

    // ---------- Encoder feedback ----------
    int32_t getEncoderPosition() const;
    void    resetEncoderPosition();

    // ---------- Callbacks ----------
    void setAlarmCallback(AlarmCallback callback);
    void setPositionReachedCallback(PositionReachedCallback callback);
    void setStatusUpdateCallback(StatusUpdateCallback callback);

    // ---------- Config management ----------
    const DriverConfig& getConfig() const;
    void                setConfig(const DriverConfig& config);
    std::string         getConfigStatus() const;

    // ---------- Misc ----------
    static std::string getVersion();
    std::string        getDriverInfo() const;

private:
    // ---------- Config ----------
    DriverConfig config_;
    DriveStatus  status_;

    // ---------- Runtime state ----------
    bool     initialized_;
    bool     enabled_;
    bool     motion_active_;
    bool     current_direction_;
    bool     homing_active_;
    bool     travel_distance_valid_;
    uint32_t travel_distance_steps_;

    uint32_t current_position_;
    uint32_t target_position_;
    uint32_t current_speed_pps_;
    uint32_t last_status_update_ms_;

    // ---------- Encoder ----------
    int64_t                 encoder_accumulator_;
    int                     last_pcnt_count_;
    pcnt_unit_handle_t      pcnt_unit_handle_;
    pcnt_channel_handle_t   pcnt_chan_a_handle_;
    pcnt_channel_handle_t   pcnt_chan_b_handle_;
    int                     resolved_pulse_gpio_; // resolved native GPIO for LEDC
    ledc_channel_t          ledc_chan_;
    ledc_timer_t            ledc_tmr_;
    bool                    ledc_ready_;

    // ---------- Profile ----------
    MotionProfile        profile_;
    esp_timer_handle_t   ramp_timer_;
    static constexpr uint32_t RAMP_INTERVAL_US = 5000;

    // ---------- Callbacks ----------
    AlarmCallback           alarm_callback_;
    PositionReachedCallback position_reached_callback_;
    StatusUpdateCallback    status_update_callback_;

#if PULSE_MOTOR_USE_FREERTOS
    SemaphoreHandle_t mutex_;
#endif

    // ---------- Helpers ----------
    bool writePinLogical(int pin, bool logical_high, bool is_dir_pin);
    bool setDirectionPin(bool state);
    bool setEnablePin(bool state);
    void stopLEDC();
    void stopMotionFromIsr(const char* reason, bool set_home);
    void handleMoveComplete();

    static void rampTimerCallback(void* arg);
    void        updateMotionProfile();
    void        startMotionProfile(uint32_t total_pulses, double max_freq,
                                   double accel_rate, double decel_rate,
                                   bool direction);
};

} // namespace PulseMotor

#endif // PULSE_MOTOR_H
