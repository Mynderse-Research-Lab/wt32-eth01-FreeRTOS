#include "EipSession.h"

#include "EipCpf.h"
#include "EipEncapsulation.h"

namespace eip {

EipSession::EipSession(ITcpClient& tcp) : tcp_(tcp) {}

bool EipSession::registerSession() {
  Bytes request = buildRegisterSessionRequest(sender_context_);
  Bytes response;
  if (!transact(request, response, 5000)) return false;

  if (!parseRegisterSessionReply(response, session_handle_)) return false;
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
  Bytes response;
  const bool ok = transact(request, response, 2000);
  session_handle_ = 0;
  return ok;
}

bool EipSession::transact(const Bytes& request, Bytes& out_response,
                          uint16_t timeout_ms) {
  if (tcp_.send(request.data(), request.size()) !=
      static_cast<ssize_t>(request.size())) {
    return false;
  }

  uint8_t buf[kMaxFrameSize];
  const ssize_t n = tcp_.recv(buf, sizeof(buf), timeout_ms);
  if (n <= 0) return false;

  out_response.assign(buf, buf + n);
  return true;
}

}  // namespace eip
