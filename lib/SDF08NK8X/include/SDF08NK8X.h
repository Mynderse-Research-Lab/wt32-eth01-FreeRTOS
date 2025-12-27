#ifndef BERGERDA_SERVO_DRIVER_H
#define BERGERDA_SERVO_DRIVER_H

#include <chrono>     // For std::chrono::system_clock
#include <cstdint>    // For uint8_t, uint16_t, uint32_t, int32_t
#include <functional> // For std::function
#include <string>     // For std::string

namespace SDF08NK8X {
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

} // namespace SDF08NK8X

#endif // BERGERDA_SERVO_DRIVER_H