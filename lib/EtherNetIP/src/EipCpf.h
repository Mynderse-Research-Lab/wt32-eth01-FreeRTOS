// Common Packet Format (CPF) - ODVA Vol 2, section 2-6.
//
// SendRRData / SendUnitData / ListIdentity replies all carry their payload as a
// CPF: a uint16 item count followed by that many {type_id, length, data} items.
// Address items locate the data (null address for unconnected, connected
// address item for Class 1/3); data items carry the actual CIP bytes.

#ifndef ETHERNET_IP_EIP_CPF_H
#define ETHERNET_IP_EIP_CPF_H

#include <cstdint>
#include <vector>

#include "EipByteBuffer.h"

namespace eip {

// CPF item type IDs (ODVA Vol 2, Table 2-6.1).
enum class CpfItemType : uint16_t {
  kNullAddress = 0x0000,
  kListIdentityResponse = 0x000C,
  kConnectedAddress = 0x00A1,   // connected address item (connection ID)
  kConnectedData = 0x00B1,      // connected data item (Class 1/3 payload)
  kUnconnectedData = 0x00B2,    // unconnected data item (explicit payload)
  kSequencedAddress = 0x8002,   // sequenced address item (Class 1 over UDP)
};

struct CpfItem {
  uint16_t type_id = 0;
  Bytes data;

  CpfItem() = default;
  CpfItem(CpfItemType t, Bytes d) : type_id(static_cast<uint16_t>(t)),
                                    data(std::move(d)) {}
};

// Encode a list of CPF items: uint16 count + each item.
Bytes encodeCpf(const std::vector<CpfItem>& items);

// Decode a CPF item list from a raw byte range.
bool decodeCpf(const uint8_t* data, size_t len, std::vector<CpfItem>& out);
bool decodeCpf(const Bytes& data, std::vector<CpfItem>& out);

// SendRRData (unconnected/explicit) envelope: a CIP message-router request is
// wrapped as interface handle (uint32, 0=CIP) + timeout (uint16) + CPF
// {null address item, unconnected data item}.
Bytes encodeSendRRDataPayload(const Bytes& cip_message, uint16_t timeout = 0);

// Extract the CIP response bytes from a SendRRData reply payload (skips the
// interface handle + timeout, then returns the unconnected data item).
bool decodeSendRRDataPayload(const Bytes& payload, Bytes& out_cip_response);

// SendUnitData (connected/implicit) envelope: interface handle (uint32, 0=CIP)
// + timeout (uint16) + CPF items (sequenced address + connected data).
Bytes encodeSendUnitDataPayload(const Bytes& cpf_items);

// Extract assembly bytes from a SendUnitData reply CPF connected-data item.
// Skips the 16-bit CIP sequence count and optional 32-bit Run/Idle header.
bool decodeSendUnitDataAssembly(const Bytes& connected_data, bool skip_run_idle,
                                Bytes& out_assembly);

// Build Class 1 O->T CPF: sequenced address (0x8002) + connected data (0x00B1).
Bytes buildClass1OutputCpf(uint32_t connection_id, uint32_t encap_sequence,
                           uint16_t cip_sequence, const Bytes& assembly,
                           bool include_run_idle_header);

// Parse Class 1 T->O CPF: extract sequenced-address connection ID + assembly.
// Used by multi-axis demux (match frame to axis by T->O connection ID).
bool parseClass1InputCpf(const Bytes& frame, uint32_t& out_connection_id,
                         Bytes& out_assembly, bool skip_run_idle = false);

}  // namespace eip

#endif  // ETHERNET_IP_EIP_CPF_H
