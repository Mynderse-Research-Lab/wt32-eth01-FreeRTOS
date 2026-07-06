// Kinetix 5100 EtherNet/IP assembly (de)serialization.
//
// Byte maps are taken from the Kinetix 5100 user manual (2198-UM004),
// Tables 102-106, and reproduced in docs/EIP_MIGRATION.md. All scalars are
// little-endian. Position/speed/etc. use the drive's engineering units
// (0.1 RPM, PUU, 0.1% torque) - see docs for the unit table.
//
//   Output assembly, instance 104 (O->T): 40 bytes  (commands to drive)
//   Input  assembly, instance 154 (T->O): 52 bytes  (status from drive)

#ifndef ETHERNET_IP_KINETIX5100_ASSEMBLY_H
#define ETHERNET_IP_KINETIX5100_ASSEMBLY_H

#include <cstdint>

#include "EipByteBuffer.h"

namespace eip {
namespace k5100 {

// Assembly instance numbers (CIP Assembly object, class 0x04).
constexpr uint16_t kOutputInstance104 = 104;
constexpr uint16_t kOutputInstance106 = 106;
constexpr uint16_t kInputInstance154 = 154;

constexpr size_t kOutput104Size = 40;
constexpr size_t kInput154Size = 52;

// OperatingMode enumeration (output byte 0 / input byte 11).
enum class OperatingMode : int8_t {
  kNotSpecified = 0,
  kPosition = 1,
  kSpeed = 2,
  kHome = 3,
  kTorque = 4,
  kGear = 5,
  kIndex = 6,
  kEcam = 7,
};

// NonCyclicMoveType enumeration (output byte 24).
enum class NonCyclicMoveType : int8_t {
  kAbsolute = 0,
  kRelative = 1,
  kIncremental = 2,
  kHighSpeedCapture = 3,
};

// --- Output assembly, instance 104 (O->T) -----------------------------------
struct OutputAssembly104 {
  int8_t operating_mode = 0;  // OperatingMode

  // Control bits (byte 1). 0->1 transitions are edge-triggered by the drive.
  bool servo_on = false;       // bit0
  bool servo_off = false;      // bit1
  bool stop_motion = false;    // bit2
  bool fault_reset = false;    // bit3
  bool start_motion = false;   // bit4

  int8_t homing_method = 0;

  int32_t speed_reference = 0;     // 0.1 RPM
  int32_t accel_reference = 0;     // 0.1 RPM/s
  int32_t decel_reference = 0;     // 0.1 RPM/s
  int32_t position_reference = 0;  // PUU (user units)
  int32_t home_return_speed = 0;   // 0.1 RPM

  int8_t non_cyclic_move_type = 0;  // NonCyclicMoveType
  int8_t cyclic_move_type = 0;
  int8_t travel_mode = 0;  // 2=non-cyclic, 10=cyclic

  // Flag bits (byte 27).
  bool position_command_override = false;  // bit0
  bool position_command_overlap = false;   // bit1
  bool captured_position_select = false;   // bit2

  int32_t torque_reference = 0;  // 0.1% rated
  int32_t torque_ramp_time = 0;  // ms
  int8_t starting_index = 0;     // PR 0..99

  // Serialize to the fixed 40-byte instance-104 layout.
  Bytes serialize() const;
};

// --- Input assembly, instance 154 (T->O) ------------------------------------
struct InputAssembly154 {
  // Status byte 0.
  bool run_mode = false;           // bit0
  bool connection_faulted = false; // bit1
  bool diagnostic_active = false;  // bit2

  int8_t diagnostic_sequence_count = 0;  // byte 1

  // Status byte 8.
  bool fault = false;      // bit1
  bool uncertain = false;  // bit2

  // Status byte 9.
  bool warning_present = false;     // bit1
  bool active = false;              // bit2
  bool ready = false;               // bit3
  bool command_in_progress = false; // bit4 (toggles per accepted command)
  bool homed_status = false;        // bit5
  bool stopped = false;             // bit6
  bool at_reference = false;        // bit7

  int8_t operating_mode = 0;  // byte 11
  int8_t active_index = 0;    // byte 12
  int8_t motor_type = 0;      // byte 15 (0 none, 1 rotary, 2 linear)

  int32_t actual_speed = 0;     // byte 16, RPM
  uint16_t fault_code = 0;      // byte 20
  uint16_t warning_code = 0;    // byte 22
  int32_t actual_position = 0;  // byte 24, PUU
  int32_t actual_torque = 0;    // byte 28, % rated

  int32_t parameter_monitor[5] = {0, 0, 0, 0, 0};  // bytes 32..51

  // Parse from a 52-byte (or larger) instance-154 frame. Returns false if the
  // input is shorter than kInput154Size.
  bool deserialize(const Bytes& data);
  bool deserialize(const uint8_t* data, size_t len);

  // Drive-endstop status: the Kinetix 5100 monitors its TBIO digital inputs
  // internally. A limit-switch trigger causes the drive to fault and stop.
  // These helpers aggregate the status bits the firmware uses to react.

  /// True when the drive reports a fault (bit1) or uncertain (bit2) condition.
  /// Limit switch activation is one possible cause among others.
  bool hasDriveFault() const { return fault || uncertain; }

  /// True when the drive reports motion stopped (bit6).
  bool isMotionStopped() const { return stopped; }

  /// Heuristic: likely a limit-switch stop when both fault and stopped are
  /// asserted. The drive's KNX5100C DIO configuration must assign digital
  /// inputs as Forward/Reverse Limit for this to be meaningful.
  bool isLikelyLimitStop() const { return hasDriveFault() && isMotionStopped(); }

  /// True when the drive is ready to accept motion commands.
  bool isReady() const { return ready && !fault && !uncertain; }
};

}  // namespace k5100
}  // namespace eip

#endif  // ETHERNET_IP_KINETIX5100_ASSEMBLY_H
