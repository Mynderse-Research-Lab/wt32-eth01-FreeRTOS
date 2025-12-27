#ifndef SDF08NK8X_H
#define SDF08NK8X_H

#include <chrono>     // For std::chrono::system_clock
#include <cstdint>    // For uint8_t, uint16_t, uint32_t, int32_t
#include <functional> // For std::function
#include <string>     // For std::string
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

  std::chrono::system_clock::time_point
      last_update; // Timestamp of last status update
};

// Callback type definitions
using AlarmCallback = std::function<void(const std::string &alarm_code)>;
using PositionReachedCallback = std::function<void(uint32_t position)>;
using StatusUpdateCallback = std::function<void(const DriveStatus &status)>;

} // namespace BergerdaServo

#endif // SDF08NK8X_H