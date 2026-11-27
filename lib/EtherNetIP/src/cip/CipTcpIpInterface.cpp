// CIP TCP/IP Interface object (see CipTcpIpInterface.h).

#include "CipTcpIpInterface.h"

#include "CipMessageRouter.h"

namespace eip {

Bytes buildEncapInactivityTimeoutGetRequest() {
    const Bytes path = buildEPath(kCipClassTcpIpInterface, 1, true,
                                  kTcpIpAttrEncapInactivityTimeout);
    return buildMessageRouterRequest(static_cast<uint8_t>(CipService::kGetAttributeSingle),
                                     path);
}

bool parseEncapInactivityTimeout(const Bytes& data, uint16_t& out_s) {
    ByteReader r(data);
    return r.u16(out_s);
}

uint32_t encapNopIntervalMs(bool known, uint16_t timeout_s) {
    const uint32_t s = known ? timeout_s : kEncapInactivityTimeoutDefaultS;
    if (s == 0) {
        return 0;
    }
    const uint32_t ms = (s * 1000u) / kEncapNopsPerTimeout;
    return (ms < kEncapNopMinIntervalMs) ? kEncapNopMinIntervalMs : ms;
}

}  // namespace eip
