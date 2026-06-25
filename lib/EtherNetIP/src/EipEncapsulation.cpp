#include "EipEncapsulation.h"

namespace eip {

Bytes encodeEncapsulation(EncapHeader header, const Bytes& data) {
  header.length = static_cast<uint16_t>(data.size());
  Bytes out;
  out.reserve(kEncapHeaderSize + data.size());
  ByteWriter w(out);
  w.u16(header.command);
  w.u16(header.length);
  w.u32(header.session_handle);
  w.u32(header.status);
  w.bytes(header.sender_context.data(), header.sender_context.size());
  w.u32(header.options);
  w.bytes(data);
  return out;
}

bool decodeEncapsulation(const Bytes& frame, EncapHeader& out_header,
                         size_t& out_data_offset, size_t& out_data_len) {
  if (frame.size() < kEncapHeaderSize) return false;
  ByteReader r(frame);
  EncapHeader h;
  if (!r.u16(h.command)) return false;
  if (!r.u16(h.length)) return false;
  if (!r.u32(h.session_handle)) return false;
  if (!r.u32(h.status)) return false;
  if (!r.bytes(h.sender_context.data(), h.sender_context.size())) return false;
  if (!r.u32(h.options)) return false;

  // The declared length must fit inside the remaining frame.
  if (h.length > r.remaining()) return false;

  out_header = h;
  out_data_offset = kEncapHeaderSize;
  out_data_len = h.length;
  return true;
}

Bytes buildRegisterSessionRequest(
    const std::array<uint8_t, 8>& sender_context) {
  Bytes data;
  ByteWriter w(data);
  w.u16(kEncapProtocolVersion);
  w.u16(0);  // options flags, must be 0

  EncapHeader h;
  h.setCommand(EncapCommand::kRegisterSession);
  h.sender_context = sender_context;
  return encodeEncapsulation(h, data);
}

bool parseRegisterSessionReply(const Bytes& frame,
                               uint32_t& out_session_handle) {
  EncapHeader h;
  size_t off = 0, len = 0;
  if (!decodeEncapsulation(frame, h, off, len)) return false;
  if (h.command != static_cast<uint16_t>(EncapCommand::kRegisterSession))
    return false;
  if (h.status != static_cast<uint32_t>(EncapStatus::kSuccess)) return false;
  // Reply data is protocol version (2) + options (2); we only need the handle.
  if (len < 4) return false;
  out_session_handle = h.session_handle;
  return true;
}

Bytes buildUnRegisterSessionRequest(uint32_t session_handle) {
  EncapHeader h;
  h.setCommand(EncapCommand::kUnRegisterSession);
  h.session_handle = session_handle;
  return encodeEncapsulation(h, Bytes{});
}

Bytes buildListIdentityRequest(
    const std::array<uint8_t, 8>& sender_context) {
  EncapHeader h;
  h.setCommand(EncapCommand::kListIdentity);
  h.sender_context = sender_context;
  return encodeEncapsulation(h, Bytes{});
}

}  // namespace eip
