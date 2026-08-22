#include "EipIoConnection.h"

#include "EipCpf.h"
#include "EipEncapsulation.h"

namespace eip {

EipIoConnection::EipIoConnection(IUdpEndpoint& udp) : udp_(udp) {}

void EipIoConnection::setConfig(const IoConnectionConfig& config) {
  config_ = config;
}

void EipIoConnection::resetSequences() {
  encap_sequence_ = 0;
  cip_sequence_ = 0;
}

void EipIoConnection::buildOutputFrameInto(Bytes& out, const Bytes& assembly) {
  ++encap_sequence_;
  ++cip_sequence_;

  // Class 1 implicit I/O over UDP 2222 carries a CPF directly, with no
  // EtherNet/IP encapsulation header. The CPF contains a sequenced address
  // item (connection ID + encapsulation sequence) and a connected data item
  // (CIP sequence count + optional Run/Idle header + assembly data).
  buildClass1OutputCpfInto(out, config_.connection_id, encap_sequence_,
                           cip_sequence_, assembly,
                           config_.ot_include_run_idle_header);
}

Bytes EipIoConnection::buildOutputFrame(const Bytes& assembly) {
  Bytes out;
  buildOutputFrameInto(out, assembly);
  return out;
}

bool EipIoConnection::parseInputFrame(const Bytes& frame, Bytes& out_assembly) {
  auto parse_connected = [&](const Bytes& connected_data) {
    return decodeSendUnitDataAssembly(connected_data,
                                      config_.to_include_run_idle_header,
                                      out_assembly);
  };

  // Class 1 implicit T->O is a raw CPF UDP payload.
  std::vector<CpfItem> items;
  if (!decodeCpf(frame, items)) return false;

  for (const CpfItem& item : items) {
    if (item.type_id == static_cast<uint16_t>(CpfItemType::kConnectedData)) {
      return parse_connected(item.data);
    }
  }
  return false;
}

}  // namespace eip
