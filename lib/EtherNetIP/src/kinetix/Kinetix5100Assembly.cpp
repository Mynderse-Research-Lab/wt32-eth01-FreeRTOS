#include "Kinetix5100Assembly.h"

namespace eip {
namespace k5100 {

namespace {
constexpr uint8_t bit(bool set, unsigned pos) {
  return set ? static_cast<uint8_t>(1u << pos) : 0u;
}
}  // namespace

Bytes OutputAssembly104::serialize() const {
  Bytes out;
  out.reserve(kOutput104Size);
  ByteWriter w(out);

  w.i8(operating_mode);  // byte 0

  uint8_t control = 0;   // byte 1
  control |= bit(servo_on, 0);
  control |= bit(servo_off, 1);
  control |= bit(stop_motion, 2);
  control |= bit(fault_reset, 3);
  control |= bit(start_motion, 4);
  w.u8(control);

  w.pad(1);              // byte 2 reserved
  w.i8(homing_method);   // byte 3

  w.i32(speed_reference);     // bytes 4-7
  w.i32(accel_reference);     // bytes 8-11
  w.i32(decel_reference);     // bytes 12-15
  w.i32(position_reference);  // bytes 16-19
  w.i32(home_return_speed);   // bytes 20-23

  w.i8(non_cyclic_move_type);  // byte 24
  w.i8(cyclic_move_type);      // byte 25
  w.i8(travel_mode);           // byte 26

  uint8_t flags = 0;  // byte 27
  flags |= bit(position_command_override, 0);
  flags |= bit(position_command_overlap, 1);
  flags |= bit(captured_position_select, 2);
  w.u8(flags);

  w.i32(torque_reference);  // bytes 28-31
  w.i32(torque_ramp_time);  // bytes 32-35
  w.i8(starting_index);     // byte 36
  w.pad(3);                 // bytes 37-39

  return out;
}

bool InputAssembly154::deserialize(const Bytes& data) {
  return deserialize(data.data(), data.size());
}

bool InputAssembly154::deserialize(const uint8_t* data, size_t len) {
  if (len < kInput154Size) return false;
  ByteReader r(data, len);

  uint8_t b0 = 0;
  if (!r.u8(b0)) return false;
  run_mode = (b0 & 0x01) != 0;
  connection_faulted = (b0 & 0x02) != 0;
  diagnostic_active = (b0 & 0x04) != 0;

  if (!r.i8(diagnostic_sequence_count)) return false;  // byte 1
  if (!r.skip(6)) return false;                        // bytes 2-7

  uint8_t b8 = 0;
  if (!r.u8(b8)) return false;  // byte 8
  fault = (b8 & 0x02) != 0;
  uncertain = (b8 & 0x04) != 0;

  uint8_t b9 = 0;
  if (!r.u8(b9)) return false;  // byte 9
  warning_present = (b9 & 0x02) != 0;
  active = (b9 & 0x04) != 0;
  ready = (b9 & 0x08) != 0;
  command_in_progress = (b9 & 0x10) != 0;
  homed_status = (b9 & 0x20) != 0;
  stopped = (b9 & 0x40) != 0;
  at_reference = (b9 & 0x80) != 0;

  if (!r.skip(1)) return false;          // byte 10 reserved
  if (!r.i8(operating_mode)) return false;  // byte 11
  if (!r.i8(active_index)) return false;    // byte 12
  if (!r.skip(2)) return false;          // bytes 13-14 reserved
  if (!r.i8(motor_type)) return false;      // byte 15

  if (!r.i32(actual_speed)) return false;     // bytes 16-19
  if (!r.u16(fault_code)) return false;       // bytes 20-21
  if (!r.u16(warning_code)) return false;     // bytes 22-23
  if (!r.i32(actual_position)) return false;  // bytes 24-27
  if (!r.i32(actual_torque)) return false;    // bytes 28-31

  for (int i = 0; i < 5; ++i) {  // bytes 32-51
    if (!r.i32(parameter_monitor[i])) return false;
  }
  return true;
}

}  // namespace k5100
}  // namespace eip
