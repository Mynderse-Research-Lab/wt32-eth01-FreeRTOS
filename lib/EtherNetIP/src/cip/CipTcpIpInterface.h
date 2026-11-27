// CIP TCP/IP Interface object (class 0xF5): the Encapsulation Inactivity
// Timeout, attribute 13 (ODVA Vol 2, section 5-4.3.2.12).
//
// A target closes its TCP connection after that many seconds with no
// encapsulation activity. Class 1 I/O does not count (Vol 2, section 2-5.5.2),
// so the scanner sends a NOP often enough to keep the session open.

#ifndef ETHERNET_IP_CIP_TCP_IP_INTERFACE_H
#define ETHERNET_IP_CIP_TCP_IP_INTERFACE_H

#include <cstdint>

#include "EipByteBuffer.h"

namespace eip {

constexpr uint16_t kCipClassTcpIpInterface = 0x00F5;
constexpr uint16_t kTcpIpAttrEncapInactivityTimeout = 13;

// Spec default when the drive does not answer (Vol 2, attribute table).
constexpr uint16_t kEncapInactivityTimeoutDefaultS = 120;

// NOPs per timeout period, so up to three can be lost before the target closes.
constexpr uint32_t kEncapNopsPerTimeout = 4;
constexpr uint32_t kEncapNopMinIntervalMs = 250;

Bytes buildEncapInactivityTimeoutGetRequest();

// Attribute 13 is a UINT in seconds.
bool parseEncapInactivityTimeout(const Bytes& data, uint16_t& out_s);

// NOP period for a timeout of timeout_s (the default when !known). 0 means the
// target has the timeout disabled, so no NOP is needed.
uint32_t encapNopIntervalMs(bool known, uint16_t timeout_s);

}  // namespace eip

#endif  // ETHERNET_IP_CIP_TCP_IP_INTERFACE_H
