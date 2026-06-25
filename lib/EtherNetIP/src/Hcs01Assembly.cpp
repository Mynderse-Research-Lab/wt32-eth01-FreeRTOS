#include "Hcs01Assembly.h"

namespace eip {
namespace hcs01 {

Bytes Hcs01PositioningCommand::serialize() const {
  Bytes out;
  out.reserve(kOutput101Size);
  ByteWriter w(out);
  w.u16(control.encode());
  w.i32(positioning_command_value);
  w.i32(positioning_velocity);
  return out;
}

bool Hcs01PositioningActual::deserialize(const Bytes& data) {
  return deserialize(data.data(), data.size());
}

bool Hcs01PositioningActual::deserialize(const uint8_t* data, size_t len) {
  if (len < kInput102Size) return false;
  ByteReader r(data, len);

  uint16_t status_raw = 0;
  if (!r.u16(status_raw)) return false;
  status = Hcs01StatusWord::decode(status_raw);

  if (!r.i32(position_feedback)) return false;
  if (!r.i32(velocity_feedback)) return false;
  if (!r.u32(diagnostic_message)) return false;
  return true;
}

}  // namespace hcs01
}  // namespace eip
