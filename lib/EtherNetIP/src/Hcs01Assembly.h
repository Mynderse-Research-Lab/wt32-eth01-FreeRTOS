// HCS01 / IndraDrive EtherNet/IP assembly (de)serialization.
//
// Recommended drive-controlled-positioning map from docs/LOW_LEVEL_GANTRY_CONTROL.md
// and IndraDrive Functions manual.
// section 4.2 (Functions manual Tab. 4-21/4-22):
//
//   Output assembly, instance 101 (O->T): 10 bytes  (command to drive)
//   Input  assembly, instance 102 (T->O): 14 bytes  (actual from drive)
//
// All scalars are little-endian (Intel format). The i32 word-order caveat
// (docs section 7) must be confirmed at bench against the EDS before trusting
// this layout on hardware.

#ifndef ETHERNET_IP_HCS01_ASSEMBLY_H
#define ETHERNET_IP_HCS01_ASSEMBLY_H

#include <cstdint>

#include "EipByteBuffer.h"
#include "Hcs01ControlStatus.h"

namespace eip {
namespace hcs01 {

constexpr uint16_t kOutputInstance101 = 101;
constexpr uint16_t kInputInstance102 = 102;
constexpr size_t kOutput101Size = 18;
constexpr size_t kInput102Size = 14;

// Command assembly (instance 101): P-0-4077 + S-0-0282 + S-0-0259 + S-0-0260 + S-0-0359.
struct Hcs01PositioningCommand {
  Hcs01ControlWord control{};

  int32_t positioning_command_value = 0;  // S-0-0282 (PUU / user units)
  int32_t positioning_velocity = 0;         // S-0-0259
  int32_t positioning_acceleration = 0;     // S-0-0260
  int32_t positioning_deceleration = 0;     // S-0-0359

  Bytes serialize() const;
};

// Actual assembly (instance 102): P-0-4078 + S-0-0051 + S-0-0040 + S-0-0390.
struct Hcs01PositioningActual {
  Hcs01StatusWord status{};

  int32_t position_feedback = 0;       // S-0-0051
  int32_t velocity_feedback = 0;       // S-0-0040
  uint32_t diagnostic_message = 0;     // S-0-0390

  bool deserialize(const Bytes& data);
  bool deserialize(const uint8_t* data, size_t len);
};

}  // namespace hcs01
}  // namespace eip

#endif  // ETHERNET_IP_HCS01_ASSEMBLY_H
