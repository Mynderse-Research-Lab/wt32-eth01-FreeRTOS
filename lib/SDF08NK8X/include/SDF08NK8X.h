#ifndef SDF08NK8X_H
#define SDF08NK8X_H

#include <Arduino.h>

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
 * @enum InputFunction
 * @brief Programmable digital input functions (PN110-PN116)
 */
enum class InputFunction : uint8_t {
  SERVO_ON = 1,       // IN0 (SON) - Servo enable
  ALARM_RESET = 2,    // IN1 (ARST) - Alarm reset
  CW_PROHIBIT = 16,   // IN3 (CW) - Reverse drive prohibit
  POSITION_CLEAR = 8, // IN4 (CLE) - Position deviation clear
  INHIBIT = 4,        // IN5 (INH) - Pulse inhibit
  GEAR_SELECT = 32,   // IN6 (GEARI) - Electronic gear selection
  // IN2 is NC per datasheet
};

/**
 * @enum OutputFunction
 * @brief Programmable digital output functions (PN70-PN72)
 */
enum class OutputFunction : uint8_t {
  POSITION_REACH = 1, // OUT1 (COIN) - Position reached
  BRAKE_RELEASE = 2,  // OUT2 (HOLD) - Brake release (servo enabled)
  ALARM = 3           // OUT3 (ALM) - Servo alarm status
};

/**
 * @struct DriverConfig
 * @brief Configuration structure for servo driver initialization
 */
struct DriverConfig {
  // GPIO Pins
  int pulse_pin;       // GPIO pin for pulse output
  int direction_pin;   // GPIO pin for direction output
  int enable_pin;      // GPIO pin for servo enable (sinks to COM+)
  int alarm_input_pin; // GPIO pin for alarm input (read from OUT3)
  int encoder_a_pin;   // GPIO pin for encoder A+ signal (optional)
  int encoder_b_pin;   // GPIO pin for encoder B+ signal (optional)
  int encoder_z_pin;   // GPIO pin for encoder Z+ signal (optional)

  // Control Modes
  PulseMode pulse_mode;     // Pulse input mode (PN8)
  ControlMode control_mode; // Control mode (PN4)

  // Parameters
  uint32_t max_pulse_freq; // Maximum pulse frequency (Hz)
  uint32_t encoder_ppr;    // Encoder pulses per revolution

  // Electronic Gear Ratio
  double gear_numerator;   // Electronic gear ratio numerator (PN9)
  double gear_denominator; // Electronic gear ratio denominator (PN10)

  // Feature Flags
  bool enable_encoder_feedback; // Enable encoder signal monitoring
  bool inverted_direction;      // Invert direction logic

  /**
   * @brief Default constructor with defaults
   */
  DriverConfig()
      : pulse_pin(-1), direction_pin(-1), enable_pin(-1), alarm_input_pin(-1),
        encoder_a_pin(-1), encoder_b_pin(-1), encoder_z_pin(-1),
        pulse_mode(PulseMode::PULSE_DIRECTION),
        control_mode(ControlMode::POSITION), max_pulse_freq(1000000),
        encoder_ppr(131072), gear_numerator(1.0), gear_denominator(1.0),
        enable_encoder_feedback(false), inverted_direction(false) {}
};

/**
 * @struct DriveStatus
 * @brief Real-time status information from the servo drive
 */
struct DriveStatus {
  bool servo_enabled;    // True if servo is enabled (SON=HIGH)
  bool alarm_active;     // True if alarm condition exists (ALM=LOW)
  bool position_reached; // True if target position reached (OUT1=HIGH)
  bool brake_released;   // True if brake released (OUT2=HIGH)

  uint32_t current_position; // Current position (encoder counts)
  int32_t position_error;    // Position error (target - current)
  uint16_t current_speed;    // Current motor speed (rpm)
  uint8_t load_percentage;   // Load percentage (0-100)

  uint32_t last_update_ms; // Timestamp of last status update (millis)
};

// Callback type definitions
using AlarmCallback = void (*)(const String &alarm_code);
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
  // DIGITAL I/O CONTROL
  // ====================================================================

  /**
   * @brief Set digital input state (for external signal simulation)
   * @param input_number Input number (0-6 for IN0-IN6)
   * @param state true=active (sink to COM+), false=inactive
   */
  void setDigitalInput(uint8_t input_number, bool state);

  /**
   * @brief Get current state of digital output
   * @param output_number Output number (1-3 for OUT1-OUT3)
   * @return true=HIGH, false=LOW
   */
  bool getDigitalOutput(uint8_t output_number) const;

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
  bool moveRelative(int32_t delta_counts, uint32_t max_speed = 10000,
                    uint32_t acceleration = 5000, uint32_t deceleration = 5000);

  /**
   * @brief Generate N pulses at specified frequency and direction
   * @param pulse_count Number of pulses to generate
   * @param frequency Pulse frequency (Hz)
   * @param direction true=forward, false=backward
   * @return true if successful, false on error
   */
  bool generatePulses(uint32_t pulse_count, uint32_t frequency,
                      bool direction = true);

  /**
   * @brief Stop all motor motion with deceleration
   * @param deceleration Deceleration rate (pps²)
   * @return true if successful
   */
  bool stopMotion(uint32_t deceleration = 50000);

  /**
   * @brief Emergency stop - immediately disable motor
   * @return true if successful
   */
  bool eStop();

  // ====================================================================
  // REAL-TIME MONITORING
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
   * @brief Get current motor speed (rpm or pps depending on mode)
   * @return Current speed from encoder frequency
   */
  uint16_t getSpeed() const;

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
  bool initialized_;
  bool enabled_;

  // Callbacks
  AlarmCallback alarm_callback_;
  PositionReachedCallback position_reached_callback_;
  StatusUpdateCallback status_update_callback_;

  // State variables
  uint32_t current_position_;
  uint32_t target_position_;
  uint32_t encoder_frequency_;
  bool motion_active_;
  uint32_t current_speed_pps_;
  bool current_direction_;
  uint32_t last_pulse_time_us_;
  uint32_t last_status_update_ms_;

  // Private methods
  void updateStatus();
  void readEncoderSignals();
  void setPulsePin(bool state);
  void setDirectionPin(bool state);
  void setEnablePin(bool state);
  void calculateSpeed();
}; // class ServoDriver

} // namespace BergerdaServo

#endif // SDF08NK8X_H