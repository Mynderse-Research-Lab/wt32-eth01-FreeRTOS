// Discovery + connection establishment for an EtherNet/IP originator.
//
//  - ListIdentity reply parsing (CIP Identity item, CPF type 0x000C).
//  - ForwardOpen / ForwardClose request building + reply parsing
//    (Connection Manager object, class 0x06 instance 1).
//
// These build the CIP Message Router request bytes (see CipMessageRouter.h);
// the caller wraps them in SendRRData + encapsulation (see EipCpf.h /
// EipEncapsulation.h) before putting them on the wire.

#ifndef ETHERNET_IP_EIP_CONNECTION_MANAGER_H
#define ETHERNET_IP_EIP_CONNECTION_MANAGER_H

#include <array>
#include <cstdint>
#include <string>

#include "EipByteBuffer.h"

namespace eip {

// --- ListIdentity ------------------------------------------------------------

struct Identity {
  uint16_t encap_protocol_version = 0;
  // Socket address (16 bytes) is big-endian per the spec; captured raw here.
  std::array<uint8_t, 16> socket_address{};
  uint16_t vendor_id = 0;
  uint16_t device_type = 0;
  uint16_t product_code = 0;
  uint8_t revision_major = 0;
  uint8_t revision_minor = 0;
  uint16_t status = 0;
  uint32_t serial_number = 0;
  std::string product_name;
  uint8_t state = 0;
};

// Parse a full ListIdentity reply frame (encapsulation + CPF with one Identity
// item). Returns false if the frame is malformed or carries no Identity item.
bool parseListIdentityReply(const Bytes& frame, Identity& out);

// --- ForwardOpen connection parameters ---------------------------------------

enum class ConnectionType : uint8_t {
  kNull = 0,
  kMulticast = 1,
  kPointToPoint = 2,
};

enum class ConnectionPriority : uint8_t {
  kLow = 0,
  kHigh = 1,
  kScheduled = 2,
  kUrgent = 3,
};

// Build the 16-bit "network connection parameters" word used in ForwardOpen.
uint16_t makeNetworkConnectionParams(uint16_t connection_size_bytes,
                                     ConnectionType type,
                                     ConnectionPriority priority,
                                     bool variable_size = false,
                                     bool redundant_owner = false,
                                     bool run_idle_header = false);

// Transport class/trigger byte: bit7 direction (1=server), bits6-4 production
// trigger (0=cyclic, 1=change-of-state, 2=application), bits3-0 transport
// class. For a Class 1 originator: client(0), cyclic(0), class 1 -> 0x01.
uint8_t makeTransportClassTrigger(uint8_t transport_class, uint8_t trigger,
                                  bool server_direction = false);

// Build the Class-1 connection path: Assembly class, config instance (logical
// instance segment 0x24/0x25), then O->T and T->O connection points (0x2C/0x2D).
// Kinetix 5100 EDS Connection1 path: "20 04 24 BF 2C 68 2C 9A".
Bytes buildAssemblyConnectionPath(uint16_t config_instance,
                                  uint16_t ot_connection_point,
                                  uint16_t to_connection_point);

// EtherNet/IP Class 1 T->O multicast address from a connection ID (239.192.x.y).
uint32_t multicastIpFromConnectionId(uint32_t connection_id);

struct ForwardOpenParams {
  uint8_t priority_time_tick = 0x0A;   // tick time exponent
  uint8_t timeout_ticks = 0xFA;        // -> connection timeout
  uint32_t ot_connection_id = 0;       // originator->target (originator chosen)
  uint32_t to_connection_id = 0;       // target->originator (0 -> target picks)
  uint16_t connection_serial = 0x0001;
  uint16_t originator_vendor_id = 0;
  uint32_t originator_serial = 0;
  uint8_t connection_timeout_multiplier = 0;
  uint32_t ot_rpi_us = 20000;          // O->T RPI in microseconds (20 ms)
  uint16_t ot_net_params = 0;
  uint32_t to_rpi_us = 20000;          // T->O RPI in microseconds
  uint16_t to_net_params = 0;
  uint8_t transport_class_trigger = 0x01;
  Bytes connection_path;               // EPATH (e.g. config + O->T + T->O)
};

// Build the ForwardOpen Message Router request (service 0x54 -> Connection
// Manager class 0x06, instance 1).
Bytes buildForwardOpenRequest(const ForwardOpenParams& p);

struct ForwardOpenReply {
  uint32_t ot_connection_id = 0;
  uint32_t to_connection_id = 0;
  uint16_t connection_serial = 0;
  uint16_t originator_vendor_id = 0;
  uint32_t originator_serial = 0;
  uint32_t ot_api_us = 0;  // actual packet interval granted, microseconds
  uint32_t to_api_us = 0;
  Bytes application_reply;
};

// Parse the success response data of a ForwardOpen (the Message Router response
// `data`, not the full reply). Returns false on truncation.
bool parseForwardOpenReply(const Bytes& response_data, ForwardOpenReply& out);

// Build a ForwardClose Message Router request (service 0x4E). The connection
// path is the same route used to open (typically the port/endpoint segment).
Bytes buildForwardCloseRequest(uint8_t priority_time_tick, uint8_t timeout_ticks,
                               uint16_t connection_serial,
                               uint16_t originator_vendor_id,
                               uint32_t originator_serial,
                               const Bytes& connection_path);

}  // namespace eip

#endif  // ETHERNET_IP_EIP_CONNECTION_MANAGER_H
