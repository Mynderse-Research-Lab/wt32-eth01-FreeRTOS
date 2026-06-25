#include "EipCpf.h"

namespace eip {

Bytes encodeCpf(const std::vector<CpfItem>& items) {
  Bytes out;
  ByteWriter w(out);
  w.u16(static_cast<uint16_t>(items.size()));
  for (const CpfItem& item : items) {
    w.u16(item.type_id);
    w.u16(static_cast<uint16_t>(item.data.size()));
    w.bytes(item.data);
  }
  return out;
}

bool decodeCpf(const uint8_t* data, size_t len, std::vector<CpfItem>& out) {
  ByteReader r(data, len);
  uint16_t count = 0;
  if (!r.u16(count)) return false;
  out.clear();
  out.reserve(count);
  for (uint16_t i = 0; i < count; ++i) {
    uint16_t type_id = 0;
    uint16_t item_len = 0;
    if (!r.u16(type_id)) return false;
    if (!r.u16(item_len)) return false;
    CpfItem item;
    item.type_id = type_id;
    if (!r.bytes(item.data, item_len)) return false;
    out.push_back(std::move(item));
  }
  return true;
}

bool decodeCpf(const Bytes& data, std::vector<CpfItem>& out) {
  return decodeCpf(data.data(), data.size(), out);
}

Bytes encodeSendRRDataPayload(const Bytes& cip_message, uint16_t timeout) {
  Bytes out;
  ByteWriter w(out);
  w.u32(0);        // interface handle: 0 = CIP
  w.u16(timeout);  // operation timeout in seconds (0 = rely on TCP)

  std::vector<CpfItem> items;
  items.emplace_back(CpfItemType::kNullAddress, Bytes{});
  items.emplace_back(CpfItemType::kUnconnectedData, cip_message);
  Bytes cpf = encodeCpf(items);
  w.bytes(cpf);
  return out;
}

bool decodeSendRRDataPayload(const Bytes& payload, Bytes& out_cip_response) {
  ByteReader r(payload);
  uint32_t interface_handle = 0;
  uint16_t timeout = 0;
  if (!r.u32(interface_handle)) return false;
  if (!r.u16(timeout)) return false;

  std::vector<CpfItem> items;
  if (!decodeCpf(r.cursor(), r.remaining(), items)) return false;

  for (const CpfItem& item : items) {
    if (item.type_id == static_cast<uint16_t>(CpfItemType::kUnconnectedData)) {
      out_cip_response = item.data;
      return true;
    }
  }
  return false;
}

}  // namespace eip
