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

Bytes EipIoConnection::buildOutputFrame(const Bytes& assembly) {
  ++encap_sequence_;
  ++cip_sequence_;

  Bytes cpf = buildClass1OutputCpf(config_.connection_id, encap_sequence_,
                                   cip_sequence_, assembly,
                                   config_.include_run_idle_header);
  Bytes payload = encodeSendUnitDataPayload(cpf);

  EncapHeader header;
  header.setCommand(EncapCommand::kSendUnitData);
  header.session_handle = config_.session_handle;
  return encodeEncapsulation(header, payload);
}

bool EipIoConnection::parseInputFrame(const Bytes& frame, Bytes& out_assembly) {
  EncapHeader header;
  size_t off = 0;
  size_t len = 0;
  if (!decodeEncapsulation(frame, header, off, len)) return false;
  if (header.command != static_cast<uint16_t>(EncapCommand::kSendUnitData))
    return false;

  Bytes payload(frame.begin() + off, frame.begin() + off + len);
  ByteReader r(payload);
  uint32_t interface_handle = 0;
  uint16_t timeout = 0;
  if (!r.u32(interface_handle)) return false;
  if (!r.u16(timeout)) return false;

  std::vector<CpfItem> items;
  if (!decodeCpf(r.cursor(), r.remaining(), items)) return false;

  for (const CpfItem& item : items) {
    if (item.type_id == static_cast<uint16_t>(CpfItemType::kConnectedData)) {
      return decodeSendUnitDataAssembly(item.data,
                                        config_.include_run_idle_header,
                                        out_assembly);
    }
  }
  return false;
}

}  // namespace eip
