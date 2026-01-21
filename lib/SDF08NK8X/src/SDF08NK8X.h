/**
 * @file SDF08NK8X.h
 * @brief Bergerda SDF-08-N-K-8X Servo Driver Library for ESP32
 * @version 1.1.0
 *
 * FEATURES:
 * - Pulse/Direction Position Control (Deterministic)
 * - Open-loop control (default) - position based on commanded pulses
 * - Closed-loop control (optional) - position correction using encoder feedback
 * - Trapezoidal Velocity Profiling (Accel/Cruise/Decel)
 * - Hardware Encoder Feedback via PCNT (16-bit wrap-around)
 * - Status Monitoring (Position, Speed, Alarm, In-Position, Brake)
 * - Thread-safe (FreeRTOS Mutex support)
 *
 * LIMITATIONS:
 * - Only supports PULSE_DIRECTION mode and POSITION control mode.
 */

#ifndef SDF08NK8X_H
#define SDF08NK8X_H

#ifndef HOME_ON_BOOT
#define HOME_ON_BOOT 1
#endif

#include <Arduino.h>
#include <driver/pcnt.h>
#include <esp_timer.h>

// Optional FreeRTOS support - define SDF08NK8X_USE_FREERTOS=1 for thread safety
#ifndef SDF08NK8X_USE_FREERTOS
#define SDF08NK8X_USE_FREERTOS 1 // Enabled by default on ESP32
#endif

#if SDF08NK8X_USE_FREERTOS
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

namespace BergerdaServo {

/**
 * @enum PulseMode
 * @brief Pulse input mode selection (PN8 parameter)
 */
enum class PulseMode : uint8_t {
  PULSE_DIRECTION = 0, // Standard pulse+direction (2-wire)
  CW_CCW = 1,          // CW/CCW dual pulse (2-wire)
  QUADRATURE = 2       // A/B Quadrature (2-wire)
};

/**
 * @enum ControlMode
 * @brief Servo control mode selection (PN4 parameter)
 */
enum class ControlMode : uint8_t {
  POSITION = 0, // Position control via pulse input
  SPEED = 1,    // Speed control via pulse frequency
  TORQUE = 2,   // Torque control (analog input)
  JOG = 3       // JOG mode (manual control)
};

/**
 * @struct DriverConfig
 * @brief Configuration structure for servo driver initialization
 */
struct DriverConfig {

  /**
   * @brief GPIO inputs from servo driver
   * @param [0]: OUT1/1(POS_reached)
   * @param [1]: OUT2/14(BRAKE)
   * @param [2]: OUT3/15(ALM)
   * @param [3]: A+/23 (Encoder A+ signal)
   * @param [4]: B+/24 (Encoder B+ signal)
   * @param [5]: Z+/25 (Encoder Z+ signal)
   */
  static constexpr size_t INPUT_PIN_COUNT = 6;
  int input_pin_nos[INPUT_PIN_COUNT];

  /**
   * @brief GPIO outputs to servo driver
   * @param [0]: IN0/21 (SON)
   * @param [1]: IN1/9 (ARST)
   * @param [2]: IN3/17 (CW/CCW prohibition)
   * @param [3]: IN4/22 (CLE - deviation counter clear)
   * @param [4]: IN5/10 (INH - command pulse prohibition)
   * @param [5]: IN6/4 (GEARI)
   * @param [6]: PULS/18 (Pulse input)
   * @param [7]: SIGN/19 (Direction input)
   */
  static constexpr size_t OUTPUT_PIN_COUNT = 8;
  int output_pin_nos[OUTPUT_PIN_COUNT];

  /**
   * @brief Control Modes
   * @param pulse_mode: Pulse input mode (PN8)
   * @param control_mode: Control mode (PN4)
   */
  PulseMode pulse_mode;
  ControlMode control_mode;

  /**
   * @brief Parameters
   * @param max_pulse_freq: Maximum pulse frequency (Hz)
   * @param encoder_ppr: Encoder pulses per revolution
   */
  uint32_t max_pulse_freq;
  uint32_t encoder_ppr;

  /**
   * @brief Electronic Gear Ratio
   * @param gear_numerator: Electronic gear ratio numerator (PN9)
   * @param gear_denominator: Electronic gear ratio denominator (PN10)
   */
  double gear_numerator;
  double gear_denominator;

  /**
   * @brief LEDC Configuration for pulse output
   * @param ledc_channel: LEDC channel (0-15) for PWM generation
   * @param ledc_pulse_pin: GPIO pin for LEDC PWM output (typically same as
   * output_pins_nos[6])
   * @param ledc_resolution: PWM resolution in bits (1-14, default
   * 2 for high freq)
   */
  int ledc_channel;
  int ledc_pulse_pin;
  uint8_t ledc_resolution;

  /**
   * @brief Feature Flags
   * @param enable_encoder_feedback: Enable encoder signal monitoring
   * @param enable_closed_loop_control: Enable closed-loop position control using encoder feedback
   *                                    (requires enable_encoder_feedback to be true)
   * @param invert_output_logic: Treat LOW (GND) as logical HIGH for outputs
   * @param invert_dir_pin: Invert DIR pin logic (true = swap direction)
   * @param pcnt_unit: ESP32 PCNT unit (0-7) for encoder counting
   * @param home_on_boot: Automatically home to MIN limit switch on first boot
   */
  bool enable_encoder_feedback;
  bool enable_closed_loop_control;
  bool invert_output_logic;
  bool invert_dir_pin;
  pcnt_unit_t pcnt_unit;
  bool home_on_boot;
  uint32_t homing_speed_pps;

  /**
   * @brief Limit Switch Debounce
   * @param limit_debounce_cycles: Number of stable samples required
   * @param limit_sample_interval_ms: Minimum time between samples
   */
  uint8_t limit_debounce_cycles;
  uint16_t limit_sample_interval_ms;
  bool limit_log_changes;

  /**
   * @brief Limit Switch Configuration
   * @param limit_min_pin: GPIO for minimum limit switch (active LOW with pullup)
   * @param limit_max_pin: GPIO for maximum limit switch (active LOW with pullup)
   */
  int limit_min_pin;
  int limit_max_pin;

  /**
   * @brief Default constructor with defaults
   */
  DriverConfig()
      : input_pin_nos{-1, -1, -1, -1, -1, -1},
        output_pin_nos{-1, -1, -1, -1, -1, -1, -1, -1},
        pulse_mode(PulseMode::PULSE_DIRECTION),
        control_mode(ControlMode::POSITION),
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
        pcnt_unit(PCNT_UNIT_0),
        home_on_boot(HOME_ON_BOOT),
        homing_speed_pps(6000),
        limit_debounce_cycles(10),
        limit_sample_interval_ms(3),
        limit_log_changes(true),
        limit_min_pin(-1),
        limit_max_pin(-1) {}
};

/**
 * @struct DriveStatus
 * @brief Real-time status information from the servo drive
 */
struct DriveStatus {
  bool servo_enabled; // True if servo is enabled (SON)

  /**
   * @brief Status signals
   * @param position_reached: True if target position reached (OUT1/POS_reached)
   * @param brake_released:   True if brake released (OUT2/BRAKE)
   * @param alarm_active:     True if alarm condition exists (OUT3/ALM)
   *  signals are all active-low
   */
  bool position_reached;
  bool brake_released;
  bool alarm_active;

  /**
   * @brief Position and Error
   * @param current_position: Current commanded position (pulses)
   * @param encoder_position: Current encoder position (raw feedback)
   * @param position_error: Distance to target (target - current_cmd)
   */
  uint32_t current_position;
  int32_t encoder_position;
  int32_t position_error;

  /**
   * @brief Speed
   * @param current_speed: Current motor speed (pps)
   */
  uint32_t current_speed;

  /**
   * @brief Timestamp
   * @param last_update_ms: Timestamp of last status update (millis)
   */
  uint32_t last_update_ms;
};

/**
 * @struct MotionProfile
 * @brief State for trapezoidal velocity profile execution
 */
struct MotionProfile {
  enum Phase : uint8_t { IDLE = 0, ACCEL, CRUISE, DECEL };

  uint32_t total_pulses;     // Total pulses for move
  uint32_t accel_pulses;     // Pulses during acceleration
  uint32_t decel_pulses;     // Pulses during deceleration
  uint32_t cruise_pulses;    // Pulses at max speed
  uint32_t pulses_generated; // Running count of pulses generated
  uint32_t target_position;  // Target position for clamping on completion
  double current_freq;       // Current frequency (Hz)
  double max_freq;           // Target max frequency (Hz)
  double accel_rate;         // Acceleration (Hz/s = pps/s)
  double decel_rate;         // Deceleration (Hz/s)
  double fractional_pulses; // Accumulated fractional pulses (reduces truncation
                            // error)
  int64_t last_update_us;   // Timestamp of last ramp update (64-bit)
  Phase phase;              // Current motion phase
  bool direction;           // Direction for position clamping
};

/**
 * @brief Callback type definitions
 * @note Callbacks are invoked from timer ISR context. Keep them short and
 *       ISR-safe. Avoid blocking operations, heap allocations, or FreeRTOS
 *       API calls that are not ISR-safe (use FromISR variants if needed).
 * @note AlarmCallback uses const char* instead of String to avoid heap
 *       allocation in ISR context.
 */
using AlarmCallback = void (*)(const char *alarm_code);
using PositionReachedCallback = void (*)(uint32_t position);
using StatusUpdateCallback = void (*)(const DriveStatus &status);

/**
 * @class ServoDriver
 * @brief Main class for controlling Bergerda F series servo drivers via CN1
 */
class ServoDriver {
public:
  // ====================================================================
  // INITIALIZATION
  // ====================================================================

  /**
   * @brief Constructor
   * @param config Driver configuration structure
   */
  explicit ServoDriver(const DriverConfig &config);

  /**
   * @brief Destructor
   */
  ~ServoDriver();

  /**
   * @brief Initialize hardware GPIO pins and driver state
   * @return true if initialization successful, false otherwise
   */
  bool initialize();

  // ====================================================================
  // SERVO CONTROL
  // ====================================================================

  /**
   * @brief Enable servo motor (set SON signal HIGH)
   * @return true if successful, false if already enabled or error
   */
  bool enable();

  /**
   * @brief Disable servo motor (set SON signal LOW, motor freewheels)
   * @return true if successful, false if already disabled
   */
  bool disable();

  /**
   * @brief Check if servo is currently enabled
   * @return true if enabled, false otherwise
   */
  bool isEnabled() const;

  /**
   * @brief Check if motion is currently in progress
   * @return true if moving, false otherwise
   */
  bool isMotionActive() const;

  /**
   * @brief Move motor to absolute position
   * @param target_position Target position in encoder counts
   * @param max_speed Maximum speed during movement (pps - pulses per second)
   * @param acceleration Acceleration rate (pps²)
   * @param deceleration Deceleration rate (pps²)
   * @return true if command accepted, false on error or motor disabled
   */
  bool moveToPosition(uint32_t target_position, uint32_t max_speed = 10000,
                      uint32_t acceleration = 5000,
                      uint32_t deceleration = 5000);

  /**
   * @brief Move motor by relative distance
   * @param delta_counts Relative distance in encoder counts
   * @param max_speed Maximum speed during movement (pps)
   * @param acceleration Acceleration rate (pps²)
   * @param deceleration Deceleration rate (pps²)
   * @return true if command accepted, false on error
   */
  bool moveRelative(int64_t delta_counts, uint32_t max_speed = 10000,
                    uint32_t acceleration = 5000, uint32_t deceleration = 5000);

  /**
   * @brief Stop all motor motion with deceleration
   * @param deceleration Deceleration rate (pps²)
   * @return true if successful
   */
  bool stopMotion(uint32_t deceleration = 50000);

  /**
   * @brief Start homing sequence - move negative until MIN limit switch triggers
   * @param speed Homing speed in pps (default 10000)
   * @return true if homing started, false if already at limit or not configured
   */
  bool startHoming(uint32_t speed = 10000);

  /**
   * @brief Check if homing sequence is active
   * @return true if currently homing
   */
  bool isHoming() const { return homing_active_; }

  /**
   * @brief Measure travel distance between MIN and MAX limit switches
   * @param speed Speed in pps
   * @param acceleration Acceleration in pps^2
   * @param deceleration Deceleration in pps^2
   * @param timeout_ms Timeout in milliseconds
   * @return true if measurement succeeded
   */
  bool measureTravelDistance(uint32_t speed = 10000,
                             uint32_t acceleration = 10000,
                             uint32_t deceleration = 10000,
                             uint32_t timeout_ms = 60000);

  /**
   * @brief Check if a valid travel distance has been measured
   */
  bool hasTravelDistance() const;

  /**
   * @brief Get measured travel distance in steps
   */
  uint32_t getTravelDistance() const;

  /**
   * @brief Notify driver of a limit switch GPIO interrupt
   * @note Call from an ISR to force debounce update on next loop/timer tick.
   */
  void notifyLimitIrq();

  /**
   * @brief Update limit switch debounce state
   * @param force When true, update immediately regardless of sample interval
   */
  void updateLimitDebounce(bool force = false);

  /**
   * @brief Read debounced limit switch states
   */
  bool getLimitMinDebounced() const;
  bool getLimitMaxDebounced() const;

  /**
   * @brief Emergency stop - immediately disable motor
   * @return true if successful
   */
  bool eStop();

  // ====================================================================
  // MONITORING
  // ====================================================================

  /**
   * @brief Get current motor status
   * @return DriveStatus structure with current state
   */
  DriveStatus getStatus() const;

  /**
   * @brief Get current position (encoder counts)
   * @return Current position from encoder feedback
   */
  uint32_t getPosition() const;

  /**
   * @brief Set current position (for position calibration)
   * @param position New position value to set as reference
   */
  void setPosition(uint32_t position);

  /**
   * @brief Get current motor speed (pps)
   * @return Current speed in pulses per second
   */
  uint32_t getSpeed() const;

  /**
   * @brief Check if alarm condition is active
   * @return true if ALM=LOW (alarm active), false if ALM=HIGH (no alarm)
   */
  bool isAlarmActive() const;

  /**
   * @brief Clear alarm condition by toggling IN1 (ARST)
   * @return true if alarm cleared, false if persistent
   */
  bool clearAlarm();

  // ====================================================================
  // ENCODER FEEDBACK
  // ====================================================================

  /**
   * @brief Get encoder position (hardware quadrature count)
   * @return Current encoder count (signed, supports bidirectional)
   */
  int32_t getEncoderPosition() const;

  /**
   * @brief Reset encoder position counter to zero
   */
  void resetEncoderPosition();

  // ====================================================================
  // CALLBACKS AND EVENT HANDLING
  // ====================================================================

  /**
   * @brief Register alarm event callback
   * @param callback Function to call when alarm occurs
   */
  void setAlarmCallback(AlarmCallback callback);

  /**
   * @brief Register position reached callback
   * @param callback Function to call when target position reached
   */
  void setPositionReachedCallback(PositionReachedCallback callback);

  /**
   * @brief Register status update callback
   * @param callback Function to call on status updates
   */
  void setStatusUpdateCallback(StatusUpdateCallback callback);

  // ====================================================================
  // CONFIGURATION MANAGEMENT
  // ====================================================================

  /**
   * @brief Get current driver configuration
   * @return Reference to config structure
   */
  const DriverConfig &getConfig() const;

  /**
   * @brief Update driver configuration (may require re-initialization)
   * @param config New configuration
   */
  void setConfig(const DriverConfig &config);

  /**
   * @brief Get configuration status string
   * @return Formatted configuration information
   */
  String getConfigStatus() const;

  // ====================================================================
  // ADVANCED FEATURES
  // ====================================================================

  /**
   * @brief Get library version
   * @return Version string "X.Y.Z"
   */
  static String getVersion();

  /**
   * @brief Get detailed driver information
   * @return Driver info string including pinout and configuration
   */
  String getDriverInfo() const;

private:
  // Private implementation
  DriverConfig config_;
  DriveStatus status_;
  bool initialized_, enabled_;
  // Encoder Tracking
  int64_t encoder_accumulator_;
  int16_t last_pcnt_count_;
  
  // Callbacks
  AlarmCallback alarm_callback_;
  PositionReachedCallback position_reached_callback_;
  StatusUpdateCallback status_update_callback_;

  // State variables
  uint32_t current_position_, target_position_, current_speed_pps_,
      last_status_update_ms_;
  bool motion_active_, current_direction_;
  bool homing_active_;
  // Travel measurement state (MIN->MAX distance)
  bool travel_distance_valid_;
  uint32_t travel_distance_steps_;

  // Limit switch debounce state
  // These track the last sampled value and the number of consecutive
  // matching samples required before accepting a change.
  uint8_t limit_min_sample_, limit_max_sample_;
  uint8_t limit_min_stable_, limit_max_stable_;
  bool limit_min_state_, limit_max_state_;
  uint32_t last_limit_sample_ms_;
  volatile bool limit_irq_pending_;



  // Private helper methods
  bool writeOutputPin(size_t index, bool state);
  bool setDirectionPin(bool state), setEnablePin(bool state);
  void stopLEDC();
  void stopMotionFromIsr(const char *reason, bool set_home);

  void handleMoveComplete();

  // Motion Profile (Accel/Decel)
  MotionProfile profile_;
  esp_timer_handle_t ramp_timer_ = NULL;
  static constexpr uint32_t RAMP_INTERVAL_US = 5000; // 5ms update interval

  static void rampTimerCallback(void *arg);
  void updateMotionProfile();
  void startMotionProfile(uint32_t total_pulses, double max_freq,
                          double accel_rate, double decel_rate, bool direction);

  // FreeRTOS Thread Safety (only when SDF08NK8X_USE_FREERTOS is enabled)
#if SDF08NK8X_USE_FREERTOS
  SemaphoreHandle_t mutex_ = NULL;
#endif

}; // class ServoDriver

} // namespace BergerdaServo

#endif // SDF08NK8X_H