#include "EipScanner.h"

#include "CipMessageRouter.h"
#include "Hcs01Assembly.h"
#include "Kinetix5100Assembly.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#endif

namespace eip {

namespace {

#ifdef ESP_PLATFORM
static const char* kTag = "EipScanner";

void logCipFailure(const char* stage, uint8_t general_status,
                   const Bytes& additional_status) {
  ESP_LOGW(kTag, "ForwardOpen %s: CIP status=0x%02X", stage, general_status);
  if (!additional_status.empty()) {
    uint16_t ext = 0;
    if (additional_status.size() >= 2) {
      ext = static_cast<uint16_t>(additional_status[0]) |
            (static_cast<uint16_t>(additional_status[1]) << 8);
    }
    ESP_LOGW(kTag, "  extended status=0x%04X", ext);
    char hex_buf[64] = {0};
    size_t offset = 0;
    for (size_t i = 0; i < additional_status.size() && offset < sizeof(hex_buf) - 3; ++i) {
      offset += snprintf(hex_buf + offset, sizeof(hex_buf) - offset, "%02X ", additional_status[i]);
    }
    ESP_LOGW(kTag, "  additional status (len=%d): %s", (int)additional_status.size(), hex_buf);
  }
}
#endif

}  // namespace

EipScanner::EipScanner(ITcpClient& tcp, IUdpEndpoint& udp,
                       const ScannerConfig& config)
    : tcp_(tcp),
      udp_(udp),
      config_(config),
      session_(tcp),
      io_(udp),
      idle_output_(buildIdleOutputAssembly()) {}

uint32_t EipScanner::recvTimeoutMs() const {
  if (open_reply_.to_api_us == 0) return 100;
  const uint32_t rpi_ms = (open_reply_.to_api_us + 999) / 1000;
  const uint32_t t = rpi_ms * 8u + 50u;
  return (t < 100u) ? 100u : t;
}

Bytes EipScanner::buildConnectionPath() const {
  return buildAssemblyConnectionPath(config_.config_assembly_instance,
                                     config_.ot_assembly_instance,
                                     config_.to_assembly_instance);
}

Bytes EipScanner::buildIdleOutputAssembly() const {
  if (config_.drive_family == ScannerConfig::DriveFamily::kHcs01) {
    hcs01::Hcs01PositioningCommand idle;
    return idle.serialize();
  }
  k5100::OutputAssembly104 idle;
  idle.servo_on = false;
  return idle.serialize();
}

bool EipScanner::connect() {
  disconnect();

  if (config_.target_ip == nullptr) return false;
  if (!tcp_.connect(config_.target_ip, EipSession::kDefaultPort)) return false;
  if (!session_.registerSession()) {
    tcp_.close();
    return false;
  }
  state_ = State::kRegistered;

  if (!forwardOpen()) {
    disconnect();
    return false;
  }

  uint32_t mcast_cid = open_reply_.to_connection_id;
  if (mcast_cid == 0) {
    mcast_cid = open_reply_.ot_connection_id;
  }
  const uint32_t bind_mcast_cid =
      (config_.to_connection_type == ConnectionType::kMulticast) ? mcast_cid : 0;
  if (!udp_.bind(EipIoConnection::kDefaultUdpPort, bind_mcast_cid)) {
    disconnect();
    return false;
  }

  configureIoFromOpenReply();
  io_.resetSequences();
  state_ = State::kConnected;
  return true;
}

bool EipScanner::forwardOpen() {
#ifdef ESP_PLATFORM
  ESP_LOGW(kTag, "--- ForwardOpen START ---");
#endif
  open_params_ = ForwardOpenParams{};
  open_params_.ot_connection_id = config_.ot_connection_id;
  open_params_.to_connection_id = config_.to_connection_id;
  open_params_.connection_serial = config_.connection_serial;
  open_params_.originator_vendor_id = config_.originator_vendor_id;
  open_params_.originator_serial = config_.originator_serial;
  open_params_.connection_timeout_multiplier =
      config_.connection_timeout_multiplier;
  open_params_.ot_rpi_us = config_.ot_rpi_us;
  open_params_.to_rpi_us = config_.to_rpi_us;
  open_params_.transport_class_trigger = makeTransportClassTrigger(1, 0);
  open_params_.connection_path = buildConnectionPath();
  // CIP Class 1 transport: connection size = 2 (sequence count) +
  //   RT header (4 bytes Run/Idle for O->T when include_run_idle_header is
  //   true, 0 for T->O) + assembly data.
  constexpr uint16_t kClass1SeqCountSize = 2;
  uint16_t ot_size = static_cast<uint16_t>(config_.ot_assembly_size) +
                     kClass1SeqCountSize;
  if (config_.include_run_idle_header) {
    ot_size += 4;
  }
  uint16_t to_size = static_cast<uint16_t>(config_.to_assembly_size) +
                     kClass1SeqCountSize;

  open_params_.ot_net_params = makeNetworkConnectionParams(
      ot_size,
      ConnectionType::kPointToPoint, ConnectionPriority::kScheduled,
      false, false, config_.include_run_idle_bit_in_net_params);
  open_params_.to_net_params = makeNetworkConnectionParams(
      to_size,
      config_.to_connection_type, ConnectionPriority::kScheduled,
      false, false, false);

  const Bytes cip_req = buildForwardOpenRequest(open_params_);
  Bytes cip_resp;
  if (!session_.sendExplicit(cip_req, cip_resp)) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "ForwardOpen SendRRData failed");
#endif
    return false;
  }

  MessageRouterResponse mr;
  if (!parseMessageRouterResponse(cip_resp, mr)) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "ForwardOpen reply parse failed (%u bytes)",
             static_cast<unsigned>(cip_resp.size()));
#endif
    return false;
  }
  if (!mr.isSuccess()) {
#ifdef ESP_PLATFORM
    logCipFailure("rejected", mr.general_status, mr.additional_status);
#endif
    return false;
  }
  if (!parseForwardOpenReply(mr.data, open_reply_)) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "ForwardOpen success data parse failed");
#endif
    return false;
  }
  if (open_reply_.ot_connection_id == 0) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "ForwardOpen reply has zero O->T connection ID");
#endif
    return false;
  }
#ifdef ESP_PLATFORM
  {
    const char* to_mode =
        (config_.to_connection_type == ConnectionType::kMulticast) ? "mcast"
                                                                 : "p2p";
    const uint32_t mcast_cid = (open_reply_.to_connection_id != 0)
                                   ? open_reply_.to_connection_id
                                   : open_reply_.ot_connection_id;
    if (config_.to_connection_type == ConnectionType::kMulticast) {
      ESP_LOGI(kTag,
               "ForwardOpen granted O->T=0x%08lx T->O=0x%08lx API=%lu us "
               "mcast=%u.%u.%u.%u",
               static_cast<unsigned long>(open_reply_.ot_connection_id),
               static_cast<unsigned long>(open_reply_.to_connection_id),
               static_cast<unsigned long>(open_reply_.to_api_us),
               (multicastIpFromConnectionId(mcast_cid) >> 24) & 0xFF,
               (multicastIpFromConnectionId(mcast_cid) >> 16) & 0xFF,
               (multicastIpFromConnectionId(mcast_cid) >> 8) & 0xFF,
               multicastIpFromConnectionId(mcast_cid) & 0xFF);
    } else {
      ESP_LOGI(kTag,
               "ForwardOpen granted O->T=0x%08lx T->O=0x%08lx API=%lu us T->O=%s",
               static_cast<unsigned long>(open_reply_.ot_connection_id),
               static_cast<unsigned long>(open_reply_.to_connection_id),
               static_cast<unsigned long>(open_reply_.to_api_us), to_mode);
    }
  }
#endif
  return true;
}

void EipScanner::configureIoFromOpenReply() {
  IoConnectionConfig io_cfg;
  io_cfg.connection_id = open_reply_.ot_connection_id;
  io_cfg.session_handle = session_.sessionHandle();
  io_cfg.ot_include_run_idle_header = config_.include_run_idle_header;
  io_cfg.to_include_run_idle_header = false; // Adapters do not send Run/Idle headers in T->O I/O
  io_.setConfig(io_cfg);
}

bool EipScanner::exchangeOnce(uint32_t recv_timeout_ms, Bytes& out_to_assembly) {
  if (state_ != State::kConnected) return false;

  Bytes output = idle_output_;
  if (process_image_ != nullptr) {
    Bytes cmd;
    if (process_image_->getCommand(cmd)) {
      output = std::move(cmd);
    }
  }


  const Bytes frame = io_.buildOutputFrame(output);
  const ssize_t sent =
      udp_.sendTo(frame.data(), frame.size(), parseIpv4Host(config_.target_ip),
                  EipIoConnection::kDefaultUdpPort);
  if (sent != static_cast<ssize_t>(frame.size())) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "O->T UDP send failed (%d of %u bytes)",
             static_cast<int>(sent), static_cast<unsigned>(frame.size()));
#endif
    return false;
  }
#ifdef ESP_PLATFORM
  static bool logged_ot_ok = false;
  if (!logged_ot_ok) {
    logged_ot_ok = true;
    ESP_LOGI(kTag, "O->T UDP sent %u bytes (conn_id=0x%08lX)",
             static_cast<unsigned>(frame.size()),
             static_cast<unsigned long>(open_reply_.ot_connection_id));
    char hex[256] = {0};
    int pos = 0;
    for (size_t i = 0; i < frame.size() && pos + 3 < (int)sizeof(hex); ++i) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", frame[i]);
    }
    ESP_LOGI(kTag, "O->T hex: %s", hex);
  }
#endif

  uint8_t buf[EipSession::kMaxFrameSize];
  const ssize_t n = udp_.recvFrom(buf, sizeof(buf), recv_timeout_ms);
  if (n <= 0) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "T->O UDP recv timeout/empty (%d bytes, wait %lu ms)",
             static_cast<int>(n), static_cast<unsigned long>(recv_timeout_ms));
#endif
    return false;
  }

  const Bytes rx(buf, buf + n);
  if (!io_.parseInputFrame(rx, out_to_assembly)) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "T->O parse failed (%d bytes, first=0x%02X%02X)",
             static_cast<int>(n), rx.size() > 1 ? rx[1] : 0, rx.size() > 0 ? rx[0] : 0);
#endif
    return false;
  }

#ifdef ESP_PLATFORM
  static bool logged_to_ok = false;
  if (!logged_to_ok) {
    logged_to_ok = true;
    ESP_LOGI(kTag, "T->O UDP received %u bytes (assembly=%u)",
             static_cast<unsigned>(n), static_cast<unsigned>(out_to_assembly.size()));
    char hex[256] = {0};
    int pos = 0;
    for (size_t i = 0; i < rx.size() && pos + 3 < (int)sizeof(hex); ++i) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx[i]);
    }
    ESP_LOGI(kTag, "T->O hex: %s", hex);
  }
#endif

  if (process_image_ != nullptr) {
    process_image_->setFeedback(out_to_assembly);
    process_image_->setOnline(true);
  }
  return true;
}

bool EipScanner::forwardClose() {
  if (!session_.isRegistered()) return true;

  const Bytes cip_req = buildForwardCloseRequest(
      open_params_.priority_time_tick, open_params_.timeout_ticks,
      open_reply_.connection_serial, open_reply_.originator_vendor_id,
      open_reply_.originator_serial, open_params_.connection_path);

  Bytes cip_resp;
  if (!session_.sendExplicit(cip_req, cip_resp)) return false;

  MessageRouterResponse mr;
  if (!parseMessageRouterResponse(cip_resp, mr)) return false;
  return mr.isSuccess();
}

void EipScanner::disconnect() {
  if (state_ == State::kConnected) {
    (void)forwardClose();
  }

  if (session_.isRegistered()) {
    (void)session_.unregisterSession();
  }

  if (process_image_ != nullptr) {
    process_image_->setOnline(false);
  }

  udp_.close();
  tcp_.close();
  state_ = State::kIdle;
  open_reply_ = ForwardOpenReply{};
}

}  // namespace eip
