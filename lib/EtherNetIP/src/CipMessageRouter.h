// CIP Message Router request/response framing + logical EPATH building.
// ODVA Vol 1, chapters 2-3 (messaging) and Appendix C (EPATH segments).
//
// A Message Router request is: service (1) + request_path_size (1, in 16-bit
// words) + request_path (EPATH) + request_data. The reply is: reply_service
// (= service | 0x80) + reserved (1) + general_status (1) +
// additional_status_size (1, words) + additional_status + response_data.

#ifndef ETHERNET_IP_CIP_MESSAGE_ROUTER_H
#define ETHERNET_IP_CIP_MESSAGE_ROUTER_H

#include <cstdint>

#include "EipByteBuffer.h"

namespace eip {

// Common CIP service codes (ODVA Vol 1, Table A-3.1).
enum class CipService : uint8_t {
  kGetAttributesAll = 0x01,
  kSetAttributesAll = 0x02,
  kGetAttributeList = 0x03,
  kReset = 0x05,
  kGetAttributeSingle = 0x0E,
  kSetAttributeSingle = 0x10,
  kForwardClose = 0x4E,
  kForwardOpen = 0x54,
};

constexpr uint8_t kCipReplyServiceMask = 0x80;

// Common CIP class IDs used by this project.
enum class CipClass : uint16_t {
  kIdentity = 0x01,
  kMessageRouter = 0x02,
  kAssembly = 0x04,
  kConnectionManager = 0x06,
};

// CIP general status codes (subset, ODVA Vol 1, Table B-1.1).
enum class CipStatus : uint8_t {
  kSuccess = 0x00,
  kConnectionFailure = 0x01,
  kResourceUnavailable = 0x02,
  kPathSegmentError = 0x04,
  kPathDestinationUnknown = 0x05,
  kServiceNotSupported = 0x08,
  kAttributeNotSupported = 0x14,
};

// Build a logical EPATH (class + instance, optionally attribute) using 8- or
// 16-bit logical segments per the magnitude of each value. Returns the raw
// path bytes; their length is always even (whole 16-bit words).
Bytes buildEPath(uint16_t class_id, uint16_t instance_id,
                 bool has_attribute = false, uint16_t attribute_id = 0);

// Build a connection-point EPATH (class + connection point), used by
// ForwardOpen to reference an assembly instance as a connection endpoint.
Bytes buildConnectionPointPath(uint16_t class_id, uint16_t connection_point);

// Assemble a Message Router request from an EPATH and (optional) request data.
Bytes buildMessageRouterRequest(uint8_t service, const Bytes& epath,
                                const Bytes& request_data = {});

struct MessageRouterResponse {
  uint8_t reply_service = 0;       // request service with 0x80 set
  uint8_t general_status = 0;      // 0 == success
  Bytes additional_status;         // raw additional status words (if any)
  Bytes data;                      // service response payload

  bool isSuccess() const {
    return general_status == static_cast<uint8_t>(CipStatus::kSuccess);
  }
};

// Parse a Message Router reply. Returns false only on structural truncation; a
// non-zero general_status is reported via out.general_status (still true).
bool parseMessageRouterResponse(const Bytes& reply,
                                MessageRouterResponse& out);

}  // namespace eip

#endif  // ETHERNET_IP_CIP_MESSAGE_ROUTER_H
