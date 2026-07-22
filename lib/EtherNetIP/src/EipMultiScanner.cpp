#include "EipMultiScanner.h"

#include <chrono>
#include <cstring>
#include <memory>

#include "CipMessageRouter.h"
#include "EipCpf.h"
#include "Hcs01Assembly.h"
#include "Kinetix5100Assembly.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#endif

namespace eip {

namespace {

#ifdef ESP_PLATFORM
static const char* kTag = "EipMulti";
#endif

Bytes buildPath(const ScannerConfig& cfg) {
  return buildAssemblyConnectionPath(cfg.config_assembly_instance,
                                     cfg.ot_assembly_instance,
                                     cfg.to_assembly_instance);
}

}  // namespace

EipMultiScanner::EipMultiScanner(ITcpClient** tcp, size_t axis_count,
                                 IUdpEndpoint& udp, const MultiAxisSlot* slots)
    : udp_(udp) {
  if (axis_count > kMaxAxes) axis_count = kMaxAxes;
  axis_count_ = axis_count;
  for (size_t i = 0; i < axis_count_; ++i) {
    axes_[i].config = slots[i].config;
    axes_[i].image = slots[i].image;
    axes_[i].tcp = tcp[i];
    axes_[i].idle_output = buildIdleOutput(axes_[i].config);
  }
}

Bytes EipMultiScanner::buildIdleOutput(const ScannerConfig& cfg) const {
  if (cfg.drive_family == ScannerConfig::DriveFamily::kHcs01) {
    hcs01::Hcs01PositioningCommand idle;
    return idle.serialize();
  }
  k5100::OutputAssembly104 idle;
  idle.servo_on = false;
  return idle.serialize();
}

bool EipMultiScanner::axisConnected(size_t i) const {
  return i < axis_count_ && axes_[i].state == AxisState::kConnected;
}

const ForwardOpenReply& EipMultiScanner::openReply(size_t i) const {
  return axes_[i].open_reply;
}

uint32_t EipMultiScanner::recvTimeoutMs() const {
  uint32_t worst = 100;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    if (axes_[i].open_reply.to_api_us == 0) continue;
    const uint32_t rpi_ms = (axes_[i].open_reply.to_api_us + 999) / 1000;
    const uint32_t t = rpi_ms * 8u + 50u;
    if (t > worst) worst = t;
  }
  return worst;
}

void EipMultiScanner::configureIo(size_t i) {
  IoConnectionConfig io_cfg;
  io_cfg.connection_id = axes_[i].open_reply.ot_connection_id;
  io_cfg.session_handle = axes_[i].session->sessionHandle();
  io_cfg.ot_include_run_idle_header = axes_[i].config.include_run_idle_header;
  io_cfg.to_include_run_idle_header = false;
  axes_[i].io->setConfig(io_cfg);
  axes_[i].io->resetSequences();
}

bool EipMultiScanner::forwardOpenAxis(size_t i) {
  AxisRuntime& ax = axes_[i];
  ax.open_params = ForwardOpenParams{};
  ax.open_params.ot_connection_id = ax.config.ot_connection_id;
  ax.open_params.to_connection_id = ax.config.to_connection_id;
  ax.open_params.connection_serial = ax.config.connection_serial;
  ax.open_params.originator_vendor_id = ax.config.originator_vendor_id;
  ax.open_params.originator_serial = ax.config.originator_serial;
  ax.open_params.connection_timeout_multiplier =
      ax.config.connection_timeout_multiplier;
  ax.open_params.ot_rpi_us = ax.config.ot_rpi_us;
  ax.open_params.to_rpi_us = ax.config.to_rpi_us;
  ax.open_params.transport_class_trigger = makeTransportClassTrigger(1, 0);
  ax.open_params.connection_path = buildPath(ax.config);

  constexpr uint16_t kClass1SeqCountSize = 2;
  uint16_t ot_size =
      static_cast<uint16_t>(ax.config.ot_assembly_size) + kClass1SeqCountSize;
  if (ax.config.include_run_idle_header) ot_size += 4;
  uint16_t to_size =
      static_cast<uint16_t>(ax.config.to_assembly_size) + kClass1SeqCountSize;

  ax.open_params.ot_net_params = makeNetworkConnectionParams(
      ot_size, ConnectionType::kPointToPoint, ConnectionPriority::kScheduled,
      false, false, ax.config.include_run_idle_bit_in_net_params);
  ax.open_params.to_net_params = makeNetworkConnectionParams(
      to_size, ax.config.to_connection_type, ConnectionPriority::kScheduled,
      false, false, false);

  const Bytes cip_req = buildForwardOpenRequest(ax.open_params);
  Bytes cip_resp;
  if (!ax.session->sendExplicit(cip_req, cip_resp)) return false;

  MessageRouterResponse mr;
  if (!parseMessageRouterResponse(cip_resp, mr) || !mr.isSuccess()) {
#ifdef ESP_PLATFORM
    ESP_LOGW(kTag, "axis%u ForwardOpen rejected status=0x%02X",
             static_cast<unsigned>(i), mr.general_status);
#endif
    return false;
  }
  if (!parseForwardOpenReply(mr.data, ax.open_reply) ||
      ax.open_reply.ot_connection_id == 0) {
    return false;
  }

  // T->O demux key: prefer granted T->O CID, fall back to O->T.
  ax.to_connection_id = (ax.open_reply.to_connection_id != 0)
                            ? ax.open_reply.to_connection_id
                            : ax.open_reply.ot_connection_id;

#ifdef ESP_PLATFORM
  ESP_LOGI(kTag,
           "axis%u FO ok O->T=0x%08lX T->O=0x%08lX demux=0x%08lX API=%lu us",
           static_cast<unsigned>(i),
           static_cast<unsigned long>(ax.open_reply.ot_connection_id),
           static_cast<unsigned long>(ax.open_reply.to_connection_id),
           static_cast<unsigned long>(ax.to_connection_id),
           static_cast<unsigned long>(ax.open_reply.to_api_us));
#endif
  return true;
}

bool EipMultiScanner::connect() {
  disconnect();
  for (size_t i = 0; i < axis_count_; ++i) {
    if (i > 0) (void)sendKeepaliveAll();
    if (!openAxis(i)) {
      disconnect();
      return false;
    }
    if (!udp_bound_ && !bindSharedUdp()) {
      disconnect();
      return false;
    }
    (void)sendKeepaliveAll();
  }
  return true;
}

bool EipMultiScanner::bindSharedUdp() {
  if (udp_bound_) return true;
  if (!udp_.bind(EipIoConnection::kDefaultUdpPort, 0)) return false;
  udp_bound_ = true;
  return true;
}

bool EipMultiScanner::openAxis(size_t i) {
  if (i >= axis_count_) return false;
  AxisRuntime& ax = axes_[i];
  if (ax.tcp == nullptr || ax.config.target_ip == nullptr) return false;

  ax.session = std::make_unique<EipSession>(*ax.tcp);
  ax.io = std::make_unique<EipIoConnection>(udp_);

  if (!ax.tcp->connect(ax.config.target_ip, EipSession::kDefaultPort)) {
    return false;
  }
  if (!ax.session->registerSession()) {
    ax.tcp->close();
    return false;
  }
  ax.state = AxisState::kRegistered;

  if (!forwardOpenAxis(i)) {
    (void)ax.session->unregisterSession();
    ax.tcp->close();
    ax.state = AxisState::kIdle;
    return false;
  }

  configureIo(i);
  ax.state = AxisState::kConnected;
  return true;
}

bool EipMultiScanner::sendAxisOutput(size_t i) {
  AxisRuntime& ax = axes_[i];
  if (ax.state != AxisState::kConnected || ax.io == nullptr) return false;

  Bytes output = ax.idle_output;
  if (ax.image != nullptr) {
    Bytes cmd;
    if (ax.image->getCommand(cmd)) output = std::move(cmd);
  }

  const Bytes frame = ax.io->buildOutputFrame(output);
  const ssize_t sent =
      udp_.sendTo(frame.data(), frame.size(), ax.config.target_ip,
                  EipIoConnection::kDefaultUdpPort);
  return sent == static_cast<ssize_t>(frame.size());
}

bool EipMultiScanner::sendKeepaliveAll() {
  bool any = false;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state == AxisState::kConnected) {
      if (!sendAxisOutput(i)) return false;
      any = true;
    }
  }
  return any;
}

bool EipMultiScanner::applyFeedback(size_t i, const Bytes& assembly) {
  if (axes_[i].image != nullptr) {
    axes_[i].image->setFeedback(assembly);
    axes_[i].image->setOnline(true);
  }
  return true;
}

size_t EipMultiScanner::matchAxisByConnectionId(uint32_t connection_id) const {
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    if (axes_[i].to_connection_id == connection_id) return i;
    // Also accept O->T CID (some adapters echo it).
    if (axes_[i].open_reply.ot_connection_id == connection_id) return i;
  }
  return kMaxAxes;
}

ExchangeStatus EipMultiScanner::exchangeOnce(uint32_t recv_timeout_ms) {
  size_t connected = 0;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state == AxisState::kConnected) ++connected;
  }
  if (connected == 0) return ExchangeStatus::kOutputSendFailed;

  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    if (!sendAxisOutput(i)) return ExchangeStatus::kOutputSendFailed;
  }

  uint32_t api_ms = 0;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    if (axes_[i].open_reply.to_api_us == 0) continue;
    const uint32_t ms = (axes_[i].open_reply.to_api_us + 999) / 1000;
    if (ms > 0 && (api_ms == 0 || ms < api_ms)) api_ms = ms;
  }
  if (api_ms == 0) api_ms = 5;

  bool got[kMaxAxes] = {};
  size_t remaining = connected;
  const auto cycle_start = std::chrono::steady_clock::now();
  const auto deadline =
      cycle_start +
      std::chrono::milliseconds(recv_timeout_ms > 0 ? recv_timeout_ms : 1);
  auto last_ot = cycle_start;

  while (remaining > 0) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    const auto since_ot_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ot)
            .count();
    if (since_ot_ms >= static_cast<std::chrono::milliseconds::rep>(api_ms)) {
      if (!sendKeepaliveAll()) return ExchangeStatus::kOutputSendFailed;
      last_ot = now;
    }

    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                          deadline - now)
                          .count();
    const auto until_ot =
        static_cast<std::chrono::milliseconds::rep>(api_ms) - since_ot_ms;
    uint32_t slice_ms = left > 0 ? static_cast<uint32_t>(left) : 1u;
    if (until_ot > 0 &&
        static_cast<uint32_t>(until_ot) < slice_ms) {
      slice_ms = static_cast<uint32_t>(until_ot);
      if (slice_ms == 0) slice_ms = 1;
    }

    uint8_t buf[EipSession::kMaxFrameSize];
    const ssize_t n = udp_.recvFrom(buf, sizeof(buf), slice_ms);
    if (n <= 0) continue;

    const Bytes rx(buf, buf + n);
    uint32_t cid = 0;
    Bytes assembly;
    if (!parseClass1InputCpf(rx, cid, assembly, false)) continue;

    const size_t idx = matchAxisByConnectionId(cid);
    if (idx >= kMaxAxes) continue;
    if (got[idx]) continue;
    got[idx] = true;
    --remaining;
    (void)applyFeedback(idx, assembly);
  }

  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state == AxisState::kConnected && !got[i]) {
      return ExchangeStatus::kInputMiss;
    }
  }
  return connected > 0 ? ExchangeStatus::kOk : ExchangeStatus::kInputMiss;
}

bool EipMultiScanner::forwardCloseAxis(size_t i) {
  AxisRuntime& ax = axes_[i];
  if (ax.session == nullptr || !ax.session->isRegistered()) return true;

  const Bytes cip_req = buildForwardCloseRequest(
      ax.open_params.priority_time_tick, ax.open_params.timeout_ticks,
      ax.open_reply.connection_serial, ax.open_reply.originator_vendor_id,
      ax.open_reply.originator_serial, ax.open_params.connection_path);

  Bytes cip_resp;
  if (!ax.session->sendExplicit(cip_req, cip_resp)) return false;
  MessageRouterResponse mr;
  if (!parseMessageRouterResponse(cip_resp, mr)) return false;
  return mr.isSuccess();
}

void EipMultiScanner::disconnect() {
  for (size_t i = 0; i < axis_count_; ++i) {
    AxisRuntime& ax = axes_[i];
    if (ax.state == AxisState::kConnected) {
      (void)forwardCloseAxis(i);
    }
    if (ax.session && ax.session->isRegistered()) {
      (void)ax.session->unregisterSession();
    }
    if (ax.image) ax.image->setOnline(false);
    if (ax.tcp) ax.tcp->close();
    ax.session.reset();
    ax.io.reset();
    ax.state = AxisState::kIdle;
    ax.open_reply = ForwardOpenReply{};
    ax.to_connection_id = 0;
  }
  if (udp_bound_) {
    udp_.close();
    udp_bound_ = false;
  }
}

}  // namespace eip
