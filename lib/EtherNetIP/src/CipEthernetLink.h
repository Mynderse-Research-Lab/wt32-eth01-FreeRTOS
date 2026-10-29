// CIP Ethernet Link object (class 0xF6) request building and reply parsing.
// ODVA Vol 2, chapter 5-5. One instance per physical port (instance 1 = P1).
//
// Read-only diagnostics for the Class 1 dropout investigation: the X drive's
// P1 is the W5500's link partner, so its counters are the "switch port"
// counters for that cable.

#ifndef ETHERNET_IP_CIP_ETHERNET_LINK_H
#define ETHERNET_IP_CIP_ETHERNET_LINK_H

#include <cstddef>
#include <cstdint>

#include "EipByteBuffer.h"

namespace eip {

constexpr uint16_t kCipClassEthernetLink = 0x00F6;

enum class EthLinkAttr : uint8_t {
    kInterfaceSpeed = 1,     // UDINT, Mbit/s
    kInterfaceFlags = 2,     // DWORD
    kInterfaceCounters = 4,  // 11 x UDINT (optional)
    kMediaCounters = 5,      // 12 x UDINT (optional)
    kInterfaceControl = 6,   // WORD control bits + UINT forced speed
};

// Ethernet Link class-specific service: read counters, then zero them.
constexpr uint8_t kCipServiceGetAndClear = 0x4C;

constexpr size_t kEthInterfaceCounterCount = 11;
constexpr size_t kEthMediaCounterCount = 12;

struct EthLinkFlags {
    uint32_t raw = 0;
    bool link_up = false;
    bool full_duplex = false;
    uint8_t negotiation = 0;  // bits 2..4, see ethNegotiationName()
    bool reset_required = false;
    bool hardware_fault = false;
};

struct EthInterfaceControl {
    uint16_t control_bits = 0;  // bit0 auto-negotiate, bit1 forced full duplex
    uint16_t forced_speed = 0;  // Mbit/s when auto-negotiate is off
};

Bytes buildEthLinkGetRequest(uint16_t instance, EthLinkAttr attr);
Bytes buildEthLinkGetAndClearRequest(uint16_t instance, EthLinkAttr attr);

EthLinkFlags decodeEthLinkFlags(uint32_t raw);
const char* ethNegotiationName(uint8_t status);

bool parseEthUdint(const Bytes& data, uint32_t& out);
bool parseEthLinkFlags(const Bytes& data, EthLinkFlags& out);
bool parseEthInterfaceControl(const Bytes& data, EthInterfaceControl& out);
// Reads `count` little-endian UDINTs. False if the reply is too short.
bool parseEthCounters(const Bytes& data, uint32_t* out, size_t count);

const char* ethInterfaceCounterName(size_t index);
const char* ethMediaCounterName(size_t index);

}  // namespace eip

#endif  // ETHERNET_IP_CIP_ETHERNET_LINK_H
