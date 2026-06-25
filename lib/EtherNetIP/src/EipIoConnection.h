// Class 1 implicit I/O connection framing over UDP 2222.
//
// Builds/parses SendUnitData packets with sequenced address (0x8002) and
// connected data (0x00B1). Run/Idle header presence is PROVISIONAL until
// confirmed against the drive EDS at bench.

#ifndef ETHERNET_IP_EIP_IO_CONNECTION_H
#define ETHERNET_IP_EIP_IO_CONNECTION_H

#include <cstdint>

#include "EipByteBuffer.h"
#include "EipTransport.h"

namespace eip {

struct IoConnectionConfig {
  uint32_t connection_id = 0;
  uint32_t session_handle = 0;
  bool include_run_idle_header = true;
};

class EipIoConnection {
 public:
  static constexpr uint16_t kDefaultUdpPort = 2222;

  explicit EipIoConnection(IUdpEndpoint& udp);

  void setConfig(const IoConnectionConfig& config);
  void resetSequences();

  // Build a full SendUnitData encapsulation frame for O->T assembly data.
  Bytes buildOutputFrame(const Bytes& assembly);

  // Parse a received T->O SendUnitData frame; returns assembly bytes.
  bool parseInputFrame(const Bytes& frame, Bytes& out_assembly);

  uint32_t encapSequence() const { return encap_sequence_; }
  uint16_t cipSequence() const { return cip_sequence_; }

 private:
  IUdpEndpoint& udp_;
  IoConnectionConfig config_;
  uint32_t encap_sequence_ = 0;
  uint16_t cip_sequence_ = 0;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_IO_CONNECTION_H
