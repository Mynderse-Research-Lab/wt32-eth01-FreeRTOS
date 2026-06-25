#include "EipConnectionManager.h"

#include "CipMessageRouter.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"

namespace eip {

bool parseListIdentityReply(const Bytes& frame, Identity& out) {
  EncapHeader h;
  size_t off = 0, len = 0;
  if (!decodeEncapsulation(frame, h, off, len)) return false;
  if (h.command != static_cast<uint16_t>(EncapCommand::kListIdentity))
    return false;
  if (h.status != static_cast<uint32_t>(EncapStatus::kSuccess)) return false;

  Bytes data(frame.begin() + off, frame.begin() + off + len);
  std::vector<CpfItem> items;
  if (!decodeCpf(data, items)) return false;

  const CpfItem* identity = nullptr;
  for (const CpfItem& item : items) {
    if (item.type_id ==
        static_cast<uint16_t>(CpfItemType::kListIdentityResponse)) {
      identity = &item;
      break;
    }
  }
  if (identity == nullptr) return false;

  ByteReader r(identity->data);
  Identity id;
  if (!r.u16(id.encap_protocol_version)) return false;
  if (!r.bytes(id.socket_address.data(), id.socket_address.size())) return false;
  if (!r.u16(id.vendor_id)) return false;
  if (!r.u16(id.device_type)) return false;
  if (!r.u16(id.product_code)) return false;
  if (!r.u8(id.revision_major)) return false;
  if (!r.u8(id.revision_minor)) return false;
  if (!r.u16(id.status)) return false;
  if (!r.u32(id.serial_number)) return false;

  uint8_t name_len = 0;
  if (!r.u8(name_len)) return false;
  Bytes name_bytes;
  if (!r.bytes(name_bytes, name_len)) return false;
  id.product_name.assign(name_bytes.begin(), name_bytes.end());

  if (!r.u8(id.state)) return false;

  out = id;
  return true;
}

uint16_t makeNetworkConnectionParams(uint16_t connection_size_bytes,
                                     ConnectionType type,
                                     ConnectionPriority priority,
                                     bool variable_size, bool redundant_owner) {
  uint16_t params = 0;
  params |= static_cast<uint16_t>(connection_size_bytes & 0x01FF);
  params |= static_cast<uint16_t>((variable_size ? 1u : 0u) << 9);
  params |= static_cast<uint16_t>((static_cast<uint16_t>(priority) & 0x3) << 10);
  params |= static_cast<uint16_t>((static_cast<uint16_t>(type) & 0x3) << 13);
  params |= static_cast<uint16_t>((redundant_owner ? 1u : 0u) << 15);
  return params;
}

uint8_t makeTransportClassTrigger(uint8_t transport_class, uint8_t trigger,
                                  bool server_direction) {
  uint8_t v = static_cast<uint8_t>(transport_class & 0x0F);
  v |= static_cast<uint8_t>((trigger & 0x07) << 4);
  v |= static_cast<uint8_t>((server_direction ? 1u : 0u) << 7);
  return v;
}

Bytes buildForwardOpenRequest(const ForwardOpenParams& p) {
  Bytes data;
  ByteWriter w(data);
  w.u8(p.priority_time_tick);
  w.u8(p.timeout_ticks);
  w.u32(p.ot_connection_id);
  w.u32(p.to_connection_id);
  w.u16(p.connection_serial);
  w.u16(p.originator_vendor_id);
  w.u32(p.originator_serial);
  w.u8(p.connection_timeout_multiplier);
  w.pad(3);  // reserved
  w.u32(p.ot_rpi_us);
  w.u16(p.ot_net_params);
  w.u32(p.to_rpi_us);
  w.u16(p.to_net_params);
  w.u8(p.transport_class_trigger);
  // Connection path size in 16-bit words, then the path.
  w.u8(static_cast<uint8_t>(p.connection_path.size() / 2));
  w.bytes(p.connection_path);

  Bytes epath =
      buildEPath(static_cast<uint16_t>(CipClass::kConnectionManager), 1);
  return buildMessageRouterRequest(static_cast<uint8_t>(CipService::kForwardOpen),
                                   epath, data);
}

bool parseForwardOpenReply(const Bytes& response_data, ForwardOpenReply& out) {
  ByteReader r(response_data);
  ForwardOpenReply reply;
  if (!r.u32(reply.ot_connection_id)) return false;
  if (!r.u32(reply.to_connection_id)) return false;
  if (!r.u16(reply.connection_serial)) return false;
  if (!r.u16(reply.originator_vendor_id)) return false;
  if (!r.u32(reply.originator_serial)) return false;
  if (!r.u32(reply.ot_api_us)) return false;
  if (!r.u32(reply.to_api_us)) return false;

  uint8_t app_reply_words = 0;
  uint8_t reserved = 0;
  if (!r.u8(app_reply_words)) return false;
  if (!r.u8(reserved)) return false;
  if (!r.bytes(reply.application_reply,
               static_cast<size_t>(app_reply_words) * 2)) {
    return false;
  }
  out = reply;
  return true;
}

Bytes buildForwardCloseRequest(uint8_t priority_time_tick, uint8_t timeout_ticks,
                               uint16_t connection_serial,
                               uint16_t originator_vendor_id,
                               uint32_t originator_serial,
                               const Bytes& connection_path) {
  Bytes data;
  ByteWriter w(data);
  w.u8(priority_time_tick);
  w.u8(timeout_ticks);
  w.u16(connection_serial);
  w.u16(originator_vendor_id);
  w.u32(originator_serial);
  // Connection path size (words) + pad + path.
  w.u8(static_cast<uint8_t>(connection_path.size() / 2));
  w.u8(0);  // reserved pad byte
  w.bytes(connection_path);

  Bytes epath =
      buildEPath(static_cast<uint16_t>(CipClass::kConnectionManager), 1);
  return buildMessageRouterRequest(
      static_cast<uint8_t>(CipService::kForwardClose), epath, data);
}

}  // namespace eip
