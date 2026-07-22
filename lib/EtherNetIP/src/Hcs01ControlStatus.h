// HCS01 / IndraDrive field-bus control and status words (P-0-4077 / P-0-4078).
//
// Bit maps from the Parameters Reference Book (Tab. 4-357 / 4-358) and
// docs/LOW_LEVEL_GANTRY_CONTROL.md. Used with profile type 0xFFFE
// (freely configurable) and the drive-controlled-positioning process-data map.

#ifndef ETHERNET_IP_HCS01_CONTROL_STATUS_H
#define ETHERNET_IP_HCS01_CONTROL_STATUS_H

#include <cstdint>

namespace eip {
namespace hcs01 {

// Ready-for-operation encoding in status word bits 15/14.
enum class ReadyForOperation : uint8_t {
  kNotReady = 0,       // 00
  kReadyBb = 1,        // 01 - ready for power on
  kReadyAb = 2,        // 10 - control + power section ready
  kInOperation = 3,    // 11 - in operation with torque (AF)
};

// Operating-mode acknowledgment in status word bits 1/0.
enum class OperatingModeAck : uint8_t {
  kParameterMode = 0,  // 00
  kNotRelevant = 1,    // 01
  kOperatingMode = 2,  // 10
};

// Positioning / jog mode in control word bits 7/6.
enum class PositioningJogMode : uint8_t {
  kPositioning = 0,       // 00 - start on bit0 toggle
  kJogPositive = 1,       // 01
  kJogNegative = 2,       // 10
  kPositioningHalt = 3,   // 11
};

struct Hcs01ControlWord {
  bool command_value_accept = false;   // bit0 - toggle to issue command
  bool operating_mode_select = false;  // bit1 - 0->1 operating, 1->0 parameter
  bool homing = false;                 // bit2
  bool relative_positioning = false;   // bit3 - with S-0-0282
  bool immediate_block_change = false; // bit4 - with S-0-0282
  bool clear_errors = false;           // bit5
  PositioningJogMode positioning_jog = PositioningJogMode::kPositioning;  // bits 7/6
  uint8_t command_operation_mode = 0;  // bits 9/8 (0=primary)
  bool iposync = false;                // bit12
  bool drive_halt = false;             // bit13 - 0->1 start, 1->0 halt
  bool drive_enable = false;           // bit14 - auto-set when comm active
  bool drive_on = false;             // bit15 - 0->1 controller enable

  uint16_t encode() const;
  static Hcs01ControlWord decode(uint16_t raw);

  // Enable sequence: Drive ON (bit15) + Drive Start via Drive Halt 0->1 (bit13).
  static Hcs01ControlWord makeDriveEnable();
  // Safe shutdown on bus failure: clear bits 13/14/15.
  static Hcs01ControlWord makeBusFailureSafe();
};

struct Hcs01StatusWord {
  OperatingModeAck operating_mode_ack = OperatingModeAck::kParameterMode;  // bits 1/0
  bool in_reference = false;              // bit2 - homed
  bool in_standstill = false;               // bit3
  bool command_value_reached = false;       // bit4 - in position (mode-dependent)
  bool command_change = false;              // bit5
  bool operating_mode_error = false;        // bit6
  bool not_following_command = false;       // bit7 - e.g. Drive Halt active
  uint8_t actual_operation_mode = 0;        // bits 9/8
  bool command_value_ack = false;           // bit10 - toggles on S-0-0282 accept
  bool class3_diagnostics = false;          // bit11
  bool class2_warning = false;              // bit12
  bool class1_error = false;                // bit13 - drive interlock
  ReadyForOperation ready = ReadyForOperation::kNotReady;  // bits 15/14

  static Hcs01StatusWord decode(uint16_t raw);
  uint16_t encode() const;

  bool isReadyForOperation() const {
    return ready == ReadyForOperation::kInOperation;
  }

  // Drive-endstop status: the HCS01 monitors its X31 digital inputs
  // internally. A limit-switch trigger causes a class-1 error (drive
  // interlock) and the drive stops following commands.

  /// True when the drive reports a class-1 error (bit13, drive interlock).
  /// Limit switch activation via X31.5/X31.6 is one possible cause.
  bool hasDriveError() const { return class1_error; }

  /// True when the drive is not following commands (bit7), e.g. due to
  /// Drive Halt, limit switch stop, or E-Stop.
  bool isMotionBlocked() const { return not_following_command; }

  /// True when the drive reports a class-2 warning (bit12).
  bool hasWarning() const { return class2_warning; }
};

}  // namespace hcs01
}  // namespace eip

#endif  // ETHERNET_IP_HCS01_CONTROL_STATUS_H
