#include "EipMultiScanner.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <climits>

#include "CipMessageRouter.h"
#include "EipClass1TimingStats.h"
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
    if (!mr.additional_status.empty()) {
      uint16_t ext = 0;
      if (mr.additional_status.size() >= 2) {
        ext = static_cast<uint16_t>(mr.additional_status[0]) |
              (static_cast<uint16_t>(mr.additional_status[1]) << 8);
      }
      ESP_LOGW(kTag,
               "axis%u FO extended=0x%04X (0x0106=ownership, 0x0111/112=RPI, "
               "0x0127/128=size)",
               static_cast<unsigned>(i), ext);
      // 0x0112 often appends acceptable O->T / T->O RPI (µs) after type bytes.
      if (ext == 0x0112 && mr.additional_status.size() >= 12) {
        const uint8_t* p = mr.additional_status.data();
        const uint32_t ot =
            static_cast<uint32_t>(p[4]) | (static_cast<uint32_t>(p[5]) << 8) |
            (static_cast<uint32_t>(p[6]) << 16) |
            (static_cast<uint32_t>(p[7]) << 24);
        const uint32_t to =
            static_cast<uint32_t>(p[8]) | (static_cast<uint32_t>(p[9]) << 8) |
            (static_cast<uint32_t>(p[10]) << 16) |
            (static_cast<uint32_t>(p[11]) << 24);
        ESP_LOGW(kTag,
                 "axis%u FO 0x0112 hint OT_RPI~%lu us TO_RPI~%lu us "
                 "(raise CONFIG_EIP_X_RPI_US)",
                 static_cast<unsigned>(i), static_cast<unsigned long>(ot),
                 static_cast<unsigned long>(to));
      }
    }
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

  // Cache IPv4 once at FO — Class 1 sendTo must never re-parse strings.
  ax.target_ip_host = ax.config.target_ip_host;
  if (ax.target_ip_host == 0) {
    ax.target_ip_host = parseIpv4Host(ax.config.target_ip);
  }
  if (ax.target_ip_host == 0) return false;

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
  // Do not stamp last_feedback_us here — FO→peer-open gap is not T->O age.
  return true;
}

void EipMultiScanner::beginCyclicExchange() {
  const int64_t now = class1NowUs();
  cyclic_started_us_ = now;
  ot_sent_since_cyclic_ = false;
  stale_gate_latched_ = false;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    axes_[i].last_feedback_us = now;
    axes_[i].received_to_since_cyclic_ = false;
  }
}

bool EipMultiScanner::sendAxisOutput(size_t i) {
  AxisRuntime& ax = axes_[i];
  if (ax.state != AxisState::kConnected || ax.io == nullptr) return false;

  // Reuse per-axis scratch to avoid std::vector churn every RPI.
  Bytes& output = ax.ot_cmd_scratch;
  if (ax.image == nullptr || !ax.image->getCommand(output)) {
    output = ax.idle_output;
  }

  ax.io->buildOutputFrameInto(ax.ot_frame_scratch, output);
  const Bytes& frame = ax.ot_frame_scratch;
  const ssize_t sent =
      udp_.sendTo(frame.data(), frame.size(), ax.target_ip_host,
                  EipIoConnection::kDefaultUdpPort);
  if (sent == static_cast<ssize_t>(frame.size())) {
    class1TimingStats().noteOtAssemblySent(output.data(), output.size(),
                                           class1NowUs());
    return true;
  }
  return false;
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

bool EipMultiScanner::applyFeedback(size_t i, const uint8_t* assembly,
                                    size_t len) {
  if (axes_[i].image != nullptr) {
    axes_[i].image->setFeedback(assembly, len);
    axes_[i].image->setOnline(true);
  }
  axes_[i].last_feedback_us = class1NowUs();
  axes_[i].received_to_since_cyclic_ = true;
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

ExchangeStatus EipMultiScanner::exchangeOnce(uint32_t /*recv_timeout_ms*/) {
  size_t connected = 0;
  for (size_t i = 0; i < axis_count_; ++i) {
    last_got_[i] = false;
    if (axes_[i].state == AxisState::kConnected) ++connected;
  }
  if (connected == 0) return ExchangeStatus::kOutputSendFailed;

  // Seed clocks when cyclic pumping begins (not at FO).
  if (cyclic_started_us_ <= 0) {
    beginCyclicExchange();
  }

  const int64_t t_ot0 = class1NowUs();
  for (size_t n = 0; n < axis_count_; ++n) {
    const size_t i = (ot_rotate_ + n) % axis_count_;
    if (axes_[i].state != AxisState::kConnected) continue;
    if (!sendAxisOutput(i)) return ExchangeStatus::kOutputSendFailed;
  }
  ot_sent_since_cyclic_ = true;
  if (axis_count_ > 0) {
    ot_rotate_ = (ot_rotate_ + 1) % axis_count_;
  }
  {
    const int64_t dt = class1NowUs() - t_ot0;
    if (dt >= 0 && dt <= static_cast<int64_t>(UINT32_MAX)) {
      class1TimingStats().recordOtSendUs(static_cast<uint32_t>(dt));
    }
  }

  // Drain every complete datagram already buffered — no wait for drive phase.
  uint8_t buf[EipSession::kMaxFrameSize];
  for (size_t drained = 0; drained < kMaxDrainPerCycle; ++drained) {
    const ssize_t n = udp_.recvFrom(buf, sizeof(buf), 0);
    if (n <= 0) break;

    uint32_t cid = 0;
    const uint8_t* assy = nullptr;
    size_t assy_len = 0;
    if (!parseClass1InputCpfView(buf, static_cast<size_t>(n), cid, assy,
                                 assy_len, false)) {
      continue;
    }

    const size_t idx = matchAxisByConnectionId(cid);
    if (idx >= kMaxAxes) continue;
    if (last_got_[idx]) continue;
    last_got_[idx] = true;
    (void)applyFeedback(idx, assy, assy_len);
  }

  uint32_t rpi_us = 0;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    const uint32_t api = axes_[i].open_reply.to_api_us;
    if (api > 0 && (rpi_us == 0 || api < rpi_us)) rpi_us = api;
  }
  if (rpi_us == 0) rpi_us = 2000;

  const int64_t now = class1NowUs();
  const uint32_t since_cyclic_us =
      (cyclic_started_us_ > 0 && now >= cyclic_started_us_)
          ? static_cast<uint32_t>(now - cyclic_started_us_)
          : 0u;
  // First T->O may lag several RPIs after the first O->T; do not tear down
  // during the initial grace window (N * RPI after cyclic start).
  if (!stale_gate_latched_) {
    if (!class1StaleGateArmed(since_cyclic_us, rpi_us, ot_sent_since_cyclic_)) {
      return ExchangeStatus::kOk;
    }
    // Grace just ended: reseed axes that never got T->O so the stale window
    // starts *after* grace (otherwise age-from-cyclic-start == grace and
    // soft-miss fires on the same cycle the gate arms).
    stale_gate_latched_ = true;
    for (size_t i = 0; i < axis_count_; ++i) {
      if (axes_[i].state != AxisState::kConnected) continue;
      if (!axes_[i].received_to_since_cyclic_) {
        axes_[i].last_feedback_us = now;
      }
    }
    return ExchangeStatus::kOk;
  }

  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    const int64_t last = axes_[i].last_feedback_us;
    const uint32_t age =
        (last > 0 && now >= last)
            ? static_cast<uint32_t>(now - last)
            : UINT32_MAX;
    if (shouldTeardownAfterStaleUs(age, rpi_us)) {
      return ExchangeStatus::kInputMiss;
    }
  }
  return ExchangeStatus::kOk;
}

bool EipMultiScanner::axisReceivedLastCycle(size_t i) const {
  if (i >= axis_count_) return false;
  return last_got_[i];
}

uint32_t EipMultiScanner::axisFeedbackAgeUs(size_t i) const {
  if (i >= axis_count_ || axes_[i].state != AxisState::kConnected) {
    return UINT32_MAX;
  }
  const int64_t last = axes_[i].last_feedback_us;
  if (last <= 0) return UINT32_MAX;
  const int64_t now = class1NowUs();
  if (now < last) return 0;
  const int64_t age = now - last;
  return (age > static_cast<int64_t>(UINT32_MAX))
             ? UINT32_MAX
             : static_cast<uint32_t>(age);
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
    ax.last_feedback_us = 0;
    ax.target_ip_host = 0;
    ax.received_to_since_cyclic_ = false;
    ax.ot_cmd_scratch.clear();
    ax.ot_frame_scratch.clear();
  }
  cyclic_started_us_ = 0;
  ot_sent_since_cyclic_ = false;
  stale_gate_latched_ = false;
  if (udp_bound_) {
    udp_.close();
    udp_bound_ = false;
  }
}

}  // namespace eip
