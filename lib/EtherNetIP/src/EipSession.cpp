#include "EipSession.h"

#include "EipCpf.h"
#include "EipEncapsulation.h"

#include <cstdio>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#endif

namespace eip {

namespace {

#ifdef ESP_PLATFORM
static const char* kTag = "EipSession";

void logFramePrefix(const char* label, const uint8_t* data, size_t len) {
  if (len >= 4) {
    uint16_t cmd = static_cast<uint16_t>(data[0] | (data[1] << 8));
    uint16_t cmd_len = static_cast<uint16_t>(data[2] | (data[3] << 8));
    ESP_LOGW(kTag, "%s %u bytes, cmd=0x%04X len=%u",
             label, static_cast<unsigned>(len),
             static_cast<unsigned>(cmd),
             static_cast<unsigned>(cmd_len));
  } else {
    ESP_LOGW(kTag, "%s %u bytes", label, static_cast<unsigned>(len));
  }
}
#endif

}  // namespace

EipSession::EipSession(ITcpClient& tcp) : tcp_(tcp) {}

bool EipSession::registerSession() {
#ifdef ESP_PLATFORM
  ESP_LOGW(kTag, "--- RegisterSession START ---");
#endif
  Bytes request = buildRegisterSessionRequest(sender_context_);
  Bytes response;
  if (!transact(request, response, 5000)) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "--- RegisterSession FAILED (timeout/error) ---");
#endif
    return false;
  }

  if (!parseRegisterSessionReply(response, session_handle_)) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "RegisterSession reply parse failed (%u bytes)",
             static_cast<unsigned>(response.size()));
#endif
    return false;
  }
#ifdef ESP_PLATFORM
  ESP_LOGI(kTag, "RegisterSession ok, handle=0x%08lX",
           static_cast<unsigned long>(session_handle_));
#endif
  return session_handle_ != 0;
}

bool EipSession::sendExplicit(const Bytes& cip_request, Bytes& out_cip_response,
                              uint16_t timeout_ms) {
  if (!isRegistered()) return false;

  Bytes payload = encodeSendRRDataPayload(cip_request);

  EncapHeader header;
  header.setCommand(EncapCommand::kSendRRData);
  header.session_handle = session_handle_;
  header.sender_context = sender_context_;
  Bytes request = encodeEncapsulation(header, payload);

  Bytes response;
  if (!transact(request, response, timeout_ms)) return false;

  EncapHeader reply_header;
  size_t off = 0;
  size_t len = 0;
  if (!decodeEncapsulation(response, reply_header, off, len)) return false;
  if (reply_header.status != static_cast<uint32_t>(EncapStatus::kSuccess))
    return false;

  Bytes reply_payload(response.begin() + off, response.begin() + off + len);
  return decodeSendRRDataPayload(reply_payload, out_cip_response);
}

bool EipSession::unregisterSession() {
  if (!isRegistered()) return true;

  Bytes request = buildUnRegisterSessionRequest(session_handle_);
  session_handle_ = 0;

  // UnRegisterSession (0x0066) has no command data and no reply (ODVA Vol 2).
  const ssize_t sent =
      tcp_.send(request.data(), request.size());
#ifdef ESP_PLATFORM
  if (sent == static_cast<ssize_t>(request.size())) {
    ESP_LOGI(kTag, "UnRegisterSession sent");
  } else {
    ESP_LOGW(kTag, "UnRegisterSession send failed (%d of %u bytes)",
             static_cast<int>(sent), static_cast<unsigned>(request.size()));
  }
#endif
  return sent == static_cast<ssize_t>(request.size());
}

bool EipSession::transact(const Bytes& request, Bytes& out_response,
                          uint16_t timeout_ms) {
#ifdef ESP_PLATFORM
  logFramePrefix("TX", request.data(), request.size());
#endif

  const ssize_t sent =
      tcp_.send(request.data(), request.size());
  if (sent != static_cast<ssize_t>(request.size())) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "TCP send failed (%d of %u bytes)",
             static_cast<int>(sent), static_cast<unsigned>(request.size()));
#endif
    return false;
  }

  uint8_t buf[kMaxFrameSize];
  const ssize_t n = tcp_.recv(buf, sizeof(buf), timeout_ms);
  if (n <= 0) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "TCP recv failed (%d bytes, timeout %u ms)",
             static_cast<int>(n), static_cast<unsigned>(timeout_ms));
#endif
    return false;
  }

#ifdef ESP_PLATFORM
  logFramePrefix("RX", buf, static_cast<size_t>(n));
#endif

  out_response.assign(buf, buf + n);
  return true;
}

}  // namespace eip
