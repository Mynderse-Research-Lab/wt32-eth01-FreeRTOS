// CIP Ethernet Link object (see CipEthernetLink.h).

#include "CipEthernetLink.h"

#include "CipMessageRouter.h"

namespace eip {

namespace {

Bytes buildEthLinkRequest(uint8_t service, uint16_t instance, EthLinkAttr attr) {
    const Bytes path = buildEPath(kCipClassEthernetLink, instance, true,
                                  static_cast<uint16_t>(attr));
    return buildMessageRouterRequest(service, path);
}

}  // namespace

Bytes buildEthLinkGetRequest(uint16_t instance, EthLinkAttr attr) {
    return buildEthLinkRequest(static_cast<uint8_t>(CipService::kGetAttributeSingle),
                               instance, attr);
}

Bytes buildEthLinkGetAndClearRequest(uint16_t instance, EthLinkAttr attr) {
    return buildEthLinkRequest(kCipServiceGetAndClear, instance, attr);
}

EthLinkFlags decodeEthLinkFlags(uint32_t raw) {
    EthLinkFlags f;
    f.raw = raw;
    f.link_up = (raw & 0x01u) != 0;
    f.full_duplex = (raw & 0x02u) != 0;
    f.negotiation = static_cast<uint8_t>((raw >> 2) & 0x07u);
    f.reset_required = (raw & 0x20u) != 0;
    f.hardware_fault = (raw & 0x40u) != 0;
    return f;
}

const char* ethNegotiationName(uint8_t status) {
    switch (status) {
        case 0: return "auto-negotiation in progress";
        case 1: return "auto-negotiation FAILED (default 10M half)";
        case 2: return "auto-negotiation FAILED, speed detected (duplex defaulted)";
        case 3: return "negotiated speed and duplex";
        case 4: return "forced (auto-negotiation not attempted)";
        default: return "reserved";
    }
}

bool parseEthUdint(const Bytes& data, uint32_t& out) {
    ByteReader r(data);
    return r.u32(out);
}

bool parseEthLinkFlags(const Bytes& data, EthLinkFlags& out) {
    uint32_t raw = 0;
    if (!parseEthUdint(data, raw)) return false;
    out = decodeEthLinkFlags(raw);
    return true;
}

bool parseEthInterfaceControl(const Bytes& data, EthInterfaceControl& out) {
    ByteReader r(data);
    return r.u16(out.control_bits) && r.u16(out.forced_speed);
}

bool parseEthCounters(const Bytes& data, uint32_t* out, size_t count) {
    if (out == nullptr) return false;
    ByteReader r(data);
    for (size_t i = 0; i < count; ++i) {
        if (!r.u32(out[i])) return false;
    }
    return true;
}

const char* ethInterfaceCounterName(size_t index) {
    static const char* const kNames[kEthInterfaceCounterCount] = {
        "in_octets", "in_ucast", "in_nucast", "in_discards", "in_errors",
        "in_unknown_protos", "out_octets", "out_ucast", "out_nucast",
        "out_discards", "out_errors",
    };
    return (index < kEthInterfaceCounterCount) ? kNames[index] : "?";
}

const char* ethMediaCounterName(size_t index) {
    static const char* const kNames[kEthMediaCounterCount] = {
        "alignment_errors", "fcs_errors", "single_collisions",
        "multiple_collisions", "sqe_test_errors", "deferred_tx",
        "late_collisions", "excessive_collisions", "mac_tx_errors",
        "carrier_sense_errors", "frame_too_long", "mac_rx_errors",
    };
    return (index < kEthMediaCounterCount) ? kNames[index] : "?";
}

}  // namespace eip
