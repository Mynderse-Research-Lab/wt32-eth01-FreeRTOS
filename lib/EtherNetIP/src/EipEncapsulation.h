// EtherNet/IP encapsulation layer (ODVA, Volume 2, chapter 2).
//
// Every EtherNet/IP message that travels over TCP/UDP 44818 (and the encap of
// UDP 2222 broadcasts like ListIdentity) begins with a fixed 24-byte
// encapsulation header followed by command-specific data.
//
// This header is pure framing: it knows nothing about CIP. The Connection
// Manager / Message Router layers build the command data that rides inside.

#ifndef ETHERNET_IP_EIP_ENCAPSULATION_H
#define ETHERNET_IP_EIP_ENCAPSULATION_H

#include <array>
#include <cstdint>

#include "EipByteBuffer.h"

namespace eip {

// Encapsulation header is exactly 24 bytes on the wire.
constexpr size_t kEncapHeaderSize = 24;

// Encapsulation commands (ODVA Vol 2, Table 2-3.2).
enum class EncapCommand : uint16_t {
  kNop = 0x0000,
  kListServices = 0x0004,
  kListIdentity = 0x0063,
  kListInterfaces = 0x0064,
  kRegisterSession = 0x0065,
  kUnRegisterSession = 0x0066,
  kSendRRData = 0x006F,    // unconnected (explicit) message
  kSendUnitData = 0x0070,  // connected (implicit/Class1, or Class3) message
};

// Encapsulation status codes (ODVA Vol 2, Table 2-3.3).
enum class EncapStatus : uint32_t {
  kSuccess = 0x0000,
  kInvalidCommand = 0x0001,
  kInsufficientMemory = 0x0002,
  kIncorrectData = 0x0003,
  kInvalidSessionHandle = 0x0064,
  kInvalidLength = 0x0065,
  kUnsupportedProtocolRevision = 0x0069,
};

struct EncapHeader {
  uint16_t command = 0;
  uint16_t length = 0;  // number of bytes of command data following the header
  uint32_t session_handle = 0;
  uint32_t status = 0;
  std::array<uint8_t, 8> sender_context{};
  uint32_t options = 0;

  void setCommand(EncapCommand c) { command = static_cast<uint16_t>(c); }
};

// Serialize a full encapsulation message: 24-byte header + data. `length` in
// the header is overwritten to match data.size().
Bytes encodeEncapsulation(EncapHeader header, const Bytes& data);

// Parse the 24-byte header out of `frame` and return the offset/length of the
// command data. Returns false if the frame is too short or the declared length
// overruns the buffer.
bool decodeEncapsulation(const Bytes& frame, EncapHeader& out_header,
                         size_t& out_data_offset, size_t& out_data_len);

// --- RegisterSession (0x0065) ------------------------------------------------
// Request/Reply data is 4 bytes: protocol version (=1) + options flags (=0).
constexpr uint16_t kEncapProtocolVersion = 1;

Bytes buildRegisterSessionRequest(
    const std::array<uint8_t, 8>& sender_context = {});

// Parse a RegisterSession reply; on success out_session_handle holds the
// handle the originator must echo on all subsequent commands.
bool parseRegisterSessionReply(const Bytes& frame, uint32_t& out_session_handle);

// --- UnRegisterSession (0x0066) ----------------------------------------------
// No command data; closes the session named by session_handle.
Bytes buildUnRegisterSessionRequest(uint32_t session_handle);

// --- ListIdentity (0x0063) ---------------------------------------------------
// Request has no command data. Reply carries a CPF with one Identity item
// (type 0x000C) describing the device.
Bytes buildListIdentityRequest(
    const std::array<uint8_t, 8>& sender_context = {});

}  // namespace eip

#endif  // ETHERNET_IP_EIP_ENCAPSULATION_H
