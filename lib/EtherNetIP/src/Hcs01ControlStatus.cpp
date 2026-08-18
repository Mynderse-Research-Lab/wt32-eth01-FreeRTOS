#include "Hcs01ControlStatus.h"

namespace eip {
namespace hcs01 {

namespace {

constexpr uint16_t bit(bool set, unsigned pos) {
  return set ? static_cast<uint16_t>(1u << pos) : 0u;
}

}  // namespace

uint16_t Hcs01ControlWord::encode() const {
  uint16_t w = 0;
  w |= bit(command_value_accept, 0);
  w |= bit(operating_mode_select, 1);
  w |= bit(homing, 2);
  w |= bit(relative_positioning, 3);
  w |= bit(immediate_block_change, 4);
  w |= bit(clear_errors, 5);
  w |= static_cast<uint16_t>(static_cast<uint8_t>(positioning_jog) & 0x3) << 6;
  w |= static_cast<uint16_t>(command_operation_mode & 0x3) << 8;
  w |= bit(iposync, 12);
  w |= bit(drive_halt, 13);
  w |= bit(drive_enable, 14);
  w |= bit(drive_on, 15);
  return w;
}

Hcs01ControlWord Hcs01ControlWord::decode(uint16_t raw) {
  Hcs01ControlWord cw;
  cw.command_value_accept = (raw & 0x0001) != 0;
  cw.operating_mode_select = (raw & 0x0002) != 0;
  cw.homing = (raw & 0x0004) != 0;
  cw.relative_positioning = (raw & 0x0008) != 0;
  cw.immediate_block_change = (raw & 0x0010) != 0;
  cw.clear_errors = (raw & 0x0020) != 0;
  cw.positioning_jog =
      static_cast<PositioningJogMode>((raw >> 6) & 0x3);
  cw.command_operation_mode = static_cast<uint8_t>((raw >> 8) & 0x3);
  cw.iposync = (raw & 0x1000) != 0;
  cw.drive_halt = (raw & 0x2000) != 0;
  cw.drive_enable = (raw & 0x4000) != 0;
  cw.drive_on = (raw & 0x8000) != 0;
  return cw;
}

Hcs01ControlWord Hcs01ControlWord::makeDriveEnable() {
  Hcs01ControlWord cw;
  cw.operating_mode_select = true;
  cw.drive_on = true;
  cw.drive_enable = true;
  cw.drive_halt = true;  // Drive Start via 0->1 on bit13
  return cw;
}

Hcs01ControlWord Hcs01ControlWord::makeDriveHalt() {
  Hcs01ControlWord cw = makeDriveEnable();
  cw.drive_halt = false;  // Halt via 1->0 on bit13
  return cw;
}

Hcs01ControlWord Hcs01ControlWord::makeDriveOff() {
  Hcs01ControlWord cw;
  cw.operating_mode_select = true;
  cw.drive_enable = true;  // Class 1 active; Drive ON stays 0 (Ab)
  return cw;
}

Hcs01ControlWord Hcs01ControlWord::makeBusFailureSafe() {
  Hcs01ControlWord cw;
  // Bits 13/14/15 cleared - prevents auto-restart after F4009/E4005.
  return cw;
}

Hcs01StatusWord Hcs01StatusWord::decode(uint16_t raw) {
  Hcs01StatusWord sw;
  sw.operating_mode_ack =
      static_cast<OperatingModeAck>(raw & 0x3);
  sw.in_reference = (raw & 0x0004) != 0;
  sw.in_standstill = (raw & 0x0008) != 0;
  sw.command_value_reached = (raw & 0x0010) != 0;
  sw.command_change = (raw & 0x0020) != 0;
  sw.operating_mode_error = (raw & 0x0040) != 0;
  sw.not_following_command = (raw & 0x0080) != 0;
  sw.actual_operation_mode = static_cast<uint8_t>((raw >> 8) & 0x3);
  sw.command_value_ack = (raw & 0x0400) != 0;
  sw.class3_diagnostics = (raw & 0x0800) != 0;
  sw.class2_warning = (raw & 0x1000) != 0;
  sw.class1_error = (raw & 0x2000) != 0;
  sw.ready = static_cast<ReadyForOperation>((raw >> 14) & 0x3);
  return sw;
}

uint16_t Hcs01StatusWord::encode() const {
  uint16_t w = 0;
  w |= static_cast<uint16_t>(static_cast<uint8_t>(operating_mode_ack) & 0x3);
  w |= bit(in_reference, 2);
  w |= bit(in_standstill, 3);
  w |= bit(command_value_reached, 4);
  w |= bit(command_change, 5);
  w |= bit(operating_mode_error, 6);
  w |= bit(not_following_command, 7);
  w |= static_cast<uint16_t>(actual_operation_mode & 0x3) << 8;
  w |= bit(command_value_ack, 10);
  w |= bit(class3_diagnostics, 11);
  w |= bit(class2_warning, 12);
  w |= bit(class1_error, 13);
  w |= static_cast<uint16_t>(static_cast<uint8_t>(ready) & 0x3) << 14;
  return w;
}

}  // namespace hcs01
}  // namespace eip
