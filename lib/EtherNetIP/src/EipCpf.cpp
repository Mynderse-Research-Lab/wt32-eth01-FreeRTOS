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

Bytes encodeSendUnitDataPayload(const Bytes& cpf_items) {
  Bytes out;
  ByteWriter w(out);
  w.u32(0);
  w.u16(0);
  w.bytes(cpf_items);
  return out;
}

bool decodeSendUnitDataAssembly(const Bytes& connected_data, bool skip_run_idle,
                                Bytes& out_assembly) {
  ByteReader r(connected_data);
  uint16_t cip_seq = 0;
  if (!r.u16(cip_seq)) return false;
  if (skip_run_idle) {
    if (!r.skip(4)) return false;
  }
  return r.bytes(out_assembly, r.remaining());
}

void buildClass1OutputCpfInto(Bytes& out, uint32_t connection_id,
                              uint32_t encap_sequence, uint16_t cip_sequence,
                              const Bytes& assembly,
                              bool include_run_idle_header) {
  // Flat CPF: item_count + sequenced address (8) + connected data
  // (cip_seq + optional Run/Idle + assembly). One reserve — no push_back growth.
  const size_t connected_len =
      2u + (include_run_idle_header ? 4u : 0u) + assembly.size();
  const size_t total = 2u + (2u + 2u + 8u) + (2u + 2u + connected_len);
  out.clear();
  out.reserve(total);
  ByteWriter w(out);
  w.u16(2);  // item count
  w.u16(static_cast<uint16_t>(CpfItemType::kSequencedAddress));
  w.u16(8);
  w.u32(connection_id);
  w.u32(encap_sequence);
  w.u16(static_cast<uint16_t>(CpfItemType::kConnectedData));
  w.u16(static_cast<uint16_t>(connected_len));
  w.u16(cip_sequence);
  if (include_run_idle_header) {
    // Run state (0x00000001). Confirmed for Kinetix 5100 via PC testing.
    w.u32(0x00000001);
  }
  w.bytes(assembly);
}

Bytes buildClass1OutputCpf(uint32_t connection_id, uint32_t encap_sequence,
                           uint16_t cip_sequence, const Bytes& assembly,
                           bool include_run_idle_header) {
  Bytes out;
  buildClass1OutputCpfInto(out, connection_id, encap_sequence, cip_sequence,
                           assembly, include_run_idle_header);
  return out;
}

bool parseClass1InputCpfView(const uint8_t* frame, size_t len,
                             uint32_t& out_connection_id,
                             const uint8_t*& out_assembly, size_t& out_len,
                             bool skip_run_idle) {
  out_assembly = nullptr;
  out_len = 0;
  if (frame == nullptr) return false;

  ByteReader r(frame, len);
  uint16_t count = 0;
  if (!r.u16(count)) return false;

  bool have_id = false;
  bool have_data = false;
  for (uint16_t i = 0; i < count; ++i) {
    uint16_t type_id = 0;
    uint16_t item_len = 0;
    if (!r.u16(type_id) || !r.u16(item_len)) return false;
    if (r.remaining() < item_len) return false;
    const uint8_t* item_data = r.cursor();
    if (!r.skip(item_len)) return false;

    if (type_id == static_cast<uint16_t>(CpfItemType::kSequencedAddress)) {
      if (item_len < 4) return false;
      ByteReader ar(item_data, item_len);
      if (!ar.u32(out_connection_id)) return false;
      have_id = true;
    } else if (type_id ==
               static_cast<uint16_t>(CpfItemType::kConnectedData)) {
      ByteReader dr(item_data, item_len);
      uint16_t cip_seq = 0;
      if (!dr.u16(cip_seq)) return false;
      if (skip_run_idle) {
        if (!dr.skip(4)) return false;
      }
      out_assembly = dr.cursor();
      out_len = dr.remaining();
      have_data = true;
    }
  }
  return have_id && have_data;
}

bool parseClass1InputCpf(const Bytes& frame, uint32_t& out_connection_id,
                         Bytes& out_assembly, bool skip_run_idle) {
  const uint8_t* assy = nullptr;
  size_t assy_len = 0;
  if (!parseClass1InputCpfView(frame.data(), frame.size(), out_connection_id,
                               assy, assy_len, skip_run_idle)) {
    return false;
  }
  out_assembly.assign(assy, assy + assy_len);
  return true;
}

}  // namespace eip
