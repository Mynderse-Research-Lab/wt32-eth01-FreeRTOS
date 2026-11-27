#include "CipMessageRouter.h"

namespace eip {

namespace {

// Logical segment, class type, 8-bit format  : 0x20 <id8>
// Logical segment, class type, 16-bit format : 0x21 0x00 <id16-lo> <id16-hi>
// Likewise 0x24/0x25 for instance, 0x30/0x31 for attribute, 0x2C/0x2D for
// connection point. The 16-bit form inserts a pad byte after the segment type.
void appendLogicalSegment(ByteWriter& w, uint8_t type8, uint8_t type16,
                          uint16_t value) {
  if (value <= 0xFF) {
    w.u8(type8);
    w.u8(static_cast<uint8_t>(value));
  } else {
    w.u8(type16);
    w.u8(0);  // pad byte
    w.u16(value);
  }
}

}  // namespace

Bytes buildEPath(uint16_t class_id, uint16_t instance_id, bool has_attribute,
                 uint16_t attribute_id) {
  Bytes path;
  ByteWriter w(path);
  appendLogicalSegment(w, 0x20, 0x21, class_id);
  appendLogicalSegment(w, 0x24, 0x25, instance_id);
  if (has_attribute) {
    appendLogicalSegment(w, 0x30, 0x31, attribute_id);
  }
  // Logical segments above always emit an even number of bytes for 8-bit ids
  // (2 bytes each) and 4 bytes for 16-bit ids, so the path is word-aligned.
  return path;
}

Bytes buildConnectionPointPath(uint16_t class_id, uint16_t connection_point) {
  Bytes path;
  ByteWriter w(path);
  appendLogicalSegment(w, 0x20, 0x21, class_id);
  appendLogicalSegment(w, 0x2C, 0x2D, connection_point);
  return path;
}

Bytes buildMessageRouterRequest(uint8_t service, const Bytes& epath,
                                const Bytes& request_data) {
  Bytes out;
  ByteWriter w(out);
  w.u8(service);
  // Path size is measured in 16-bit words; EPATHs are always word-aligned.
  w.u8(static_cast<uint8_t>(epath.size() / 2));
  w.bytes(epath);
  w.bytes(request_data);
  return out;
}

bool parseMessageRouterResponse(const Bytes& reply,
                                MessageRouterResponse& out) {
  ByteReader r(reply);
  uint8_t reserved = 0;
  uint8_t add_status_words = 0;
  if (!r.u8(out.reply_service)) return false;
  if (!r.u8(reserved)) return false;
  if (!r.u8(out.general_status)) return false;
  if (!r.u8(add_status_words)) return false;

  const size_t add_status_bytes = static_cast<size_t>(add_status_words) * 2;
  if (!r.bytes(out.additional_status, add_status_bytes)) return false;

  // Whatever remains is the service response data.
  return r.bytes(out.data, r.remaining());
}

}  // namespace eip
