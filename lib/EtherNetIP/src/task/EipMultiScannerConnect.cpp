/**
 * @file EipMultiScannerConnect.cpp
 * @brief Forward open, forward close and reopen for the multi-axis scanner.
 */

#include "EipMultiScanner.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <climits>

#include "CipMessageRouter.h"
#include "CipTcpIpInterface.h"
#include "EipClass1TimingStats.h"
#include "EipCpf.h"
#include "EipScannerPolicy.h"
#include "Hcs01Assembly.h"
#include "Kinetix5100Assembly.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#endif

#include "EipMultiScannerInternal.h"

namespace eip {
namespace {


Bytes buildPath(const ScannerConfig& cfg) {
  return buildAssemblyConnectionPath(cfg.config_assembly_instance,
                                     cfg.ot_assembly_instance,
                                     cfg.to_assembly_instance);
}

}  // namespace

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

#ifdef ESP_PLATFORM
  ESP_LOGI(kTag,
           "axis%u FO sizes O->T=%u T->O=%u asm=%u/%u rpi=%lu inst %u/%u/%u T->O=%s",
           static_cast<unsigned>(i), ot_size, to_size,
           static_cast<unsigned>(ax.config.ot_assembly_size),
           static_cast<unsigned>(ax.config.to_assembly_size),
           static_cast<unsigned long>(ax.config.ot_rpi_us),
           ax.config.config_assembly_instance, ax.config.ot_assembly_instance,
           ax.config.to_assembly_instance,
           (ax.config.to_connection_type == ConnectionType::kMulticast)
               ? "mcast"
               : "p2p");
#endif

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

  if (ax.session == nullptr) {
    ax.session = std::make_unique<EipSession>(*ax.tcp);
  }
  if (ax.io == nullptr) {
    ax.io = std::make_unique<EipIoConnection>(udp_);
  }

  if (!ax.tcp->connect(ax.config.target_ip, EipSession::kDefaultPort)) {
    return false;
  }
  if (!ax.session->registerSession()) {
    ax.tcp->close();
    return false;
  }
  ax.state = AxisState::kRegistered;
  if (ax.config.read_encap_inactivity_timeout) {
      readEncapInactivityTimeout(i);
  } else {
      ax.nop_interval_ms = eip::encapNopIntervalMs(false, 0);
  }

  if (!forwardOpenAxis(i)) {
    (void)ax.session->unregisterSession();
    ax.tcp->close();
    ax.state = AxisState::kIdle;
    return false;
  }

  configureIo(i);
  ax.next_nop_us = class1NowUs() + static_cast<int64_t>(ax.nop_interval_ms) * 1000;
  ax.nop_failed_logged = false;
  ax.state = AxisState::kConnected;
  ax.first_to_logged = false;
  // Do not stamp last_feedback_us here — FO→peer-open gap is not T->O age.
  return true;
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

// A Kinetix ends its explicit session while Class 1 carries on over UDP, so that
// session may be gone by the time a ForwardClose needs it.
bool EipMultiScanner::reconnectExplicitSession(size_t i) {
  AxisRuntime& ax = axes_[i];
  if (ax.tcp == nullptr || ax.session == nullptr) return false;
  ax.tcp->close();
  if (!ax.tcp->connect(ax.config.target_ip, EipSession::kDefaultPort)) return false;
  return ax.session->registerSession();
}

// Read on the axis's own session before Forward Open. A drive that does not
// answer gets the spec default, so it is still kept alive.
void EipMultiScanner::readEncapInactivityTimeout(size_t i) {
    AxisRuntime& ax = axes_[i];
    uint16_t timeout_s = 0;
    bool known = false;
    Bytes reply;
    MessageRouterResponse mr;
    if (ax.session->sendExplicit(buildEncapInactivityTimeoutGetRequest(), reply) &&
        parseMessageRouterResponse(reply, mr)) {
        known = mr.isSuccess() && parseEncapInactivityTimeout(mr.data, timeout_s);
    }
    ax.nop_interval_ms = eip::encapNopIntervalMs(known, timeout_s);
#ifdef ESP_PLATFORM
    if (known) {
        ESP_LOGI(kTag, "axis%u encapsulation inactivity timeout %u s, NOP every %lu ms",
                 static_cast<unsigned>(i), static_cast<unsigned>(timeout_s),
                 static_cast<unsigned long>(ax.nop_interval_ms));
    } else {
        ESP_LOGW(kTag, "axis%u encapsulation inactivity timeout not read (CIP status 0x%02X),"
                 " assuming %u s, NOP every %lu ms",
                 static_cast<unsigned>(i), mr.general_status,
                 static_cast<unsigned>(kEncapInactivityTimeoutDefaultS),
                 static_cast<unsigned long>(ax.nop_interval_ms));
    }
#endif
}

void EipMultiScanner::serviceEncapKeepalive(int64_t now_us) {
    for (size_t i = 0; i < axis_count_; ++i) {
        AxisRuntime& ax = axes_[i];
        if (ax.state != AxisState::kConnected || ax.nop_interval_ms == 0 ||
            ax.session == nullptr || now_us < ax.next_nop_us) {
            continue;
        }
        ax.next_nop_us = now_us + static_cast<int64_t>(ax.nop_interval_ms) * 1000;
        if (!ax.session->sendNop() && !ax.nop_failed_logged) {
            ax.nop_failed_logged = true;
            deferred_log_.push(Class1LogEvent::kEncapNopFailed, static_cast<uint8_t>(i),
                               ax.nop_interval_ms);
        }
        return;
    }
}

uint32_t EipMultiScanner::axisEncapNopIntervalMs(size_t i) const {
    return (i < axis_count_) ? axes_[i].nop_interval_ms : 0;
}

bool EipMultiScanner::gracefulCloseAxis(size_t i) {
  if (i >= axis_count_ || axes_[i].state != AxisState::kConnected) return false;
  AxisRuntime& ax = axes_[i];
  // Still kConnected here: the keepalive helper keeps this axis's O->T running
  // through a retry, so the connection cannot time out while it is rebuilt.
  bool closed = ax.tcp != nullptr && ax.tcp->isConnected() && forwardCloseAxis(i);
  if (!closed) {
    closed = reconnectExplicitSession(i) && forwardCloseAxis(i);
  }
  closeAxis(i, false);
  return closed;
}

void EipMultiScanner::closeAxis(size_t i, bool forward_close) {
  AxisRuntime& ax = axes_[i];
  const bool had_connection = (ax.state == AxisState::kConnected) ||
                              (ax.open_reply.ot_connection_id != 0);
  // kIdle first so any concurrent keepalive/drain stops touching this axis.
  ax.state = AxisState::kIdle;
  if (had_connection && forward_close) {
    (void)forwardCloseAxis(i);
  }
  if (ax.session && ax.session->isRegistered()) {
    (void)ax.session->unregisterSession();
  }
  if (ax.image) ax.image->setOnline(false);
  if (ax.tcp) ax.tcp->close();
  ax.open_reply = ForwardOpenReply{};
  ax.to_connection_id = 0;
  ax.last_feedback_us = 0;
  ax.target_ip_host = 0;
  ax.received_to_since_cyclic_ = false;
  ax.first_to_logged = false;
  ax.ot_cmd_scratch.clear();
  ax.ot_frame_scratch.clear();
}

void EipMultiScanner::prepareAxisReopen(size_t i) {
  if (i >= axis_count_) return;
  // ForwardClose still needs open_reply, so only the state flips here.
  axes_[i].state = AxisState::kIdle;
  if (axes_[i].image != nullptr) {
    axes_[i].image->setOnline(false);
  }
}

bool EipMultiScanner::reopenAxis(size_t i) {
  if (i >= axis_count_) return false;
  AxisRuntime& ax = axes_[i];
  prepareAxisReopen(i);
  closeAxis(i);

  const bool ok = openAxis(i);
  const int64_t now = class1NowUs();
  if (ok) {
    // Fresh FO: start the stale window now, as beginCyclicExchange would.
    ax.last_feedback_us = now;
    ax.received_to_since_cyclic_ = false;
    ax.reopen_pending = false;
    ax.reopen_backoff_ms = kAxisReopenInitialBackoffMs;
    deferred_log_.push(Class1LogEvent::kAxisReopenOk, static_cast<uint8_t>(i));
    return true;
  }
  const uint32_t wait_ms = (ax.reopen_backoff_ms > 0) ? ax.reopen_backoff_ms
                                                      : kAxisReopenInitialBackoffMs;
  ax.reopen_pending = true;
  ax.reopen_due_us = now + static_cast<int64_t>(wait_ms) * 1000;
  ax.reopen_backoff_ms = axisReopenNextBackoffMs(wait_ms);
  deferred_log_.push(Class1LogEvent::kAxisReopenFailed, static_cast<uint8_t>(i),
                     wait_ms);
  return false;
}

size_t EipMultiScanner::nextAxisToReopen() const {
  const int64_t now = class1NowUs();
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].reopen_pending && now >= axes_[i].reopen_due_us) {
      return i;
    }
  }
  return kMaxAxes;
}

bool EipMultiScanner::axisReopenPending(size_t i) const {
  return i < axis_count_ && axes_[i].reopen_pending;
}

void EipMultiScanner::deferAxisReopen(size_t i) {
  if (i >= axis_count_ || !axes_[i].reopen_pending) return;
  AxisRuntime& ax = axes_[i];
  const uint32_t wait_ms = (ax.reopen_backoff_ms > 0) ? ax.reopen_backoff_ms
                                                      : kAxisReopenInitialBackoffMs;
  ax.reopen_due_us = class1NowUs() + static_cast<int64_t>(wait_ms) * 1000;
  ax.reopen_backoff_ms = axisReopenNextBackoffMs(wait_ms);
}

void EipMultiScanner::requestAxisReopen(size_t i) {
  if (i >= axis_count_ || axes_[i].reopen_pending) return;
  AxisRuntime& ax = axes_[i];
  ax.reopen_pending = true;
  ax.reopen_due_us =
      class1NowUs() + static_cast<int64_t>(kAxisReopenInitialBackoffMs) * 1000;
  ax.reopen_backoff_ms = axisReopenNextBackoffMs(kAxisReopenInitialBackoffMs);
}

void EipMultiScanner::disconnect() {
  for (size_t i = 0; i < axis_count_; ++i) {
    closeAxis(i);
    axes_[i].reopen_pending = false;
    axes_[i].reopen_due_us = 0;
    axes_[i].reopen_backoff_ms = 0;
  }
  cyclic_started_us_ = 0;
  ot_sent_since_cyclic_ = false;
  stale_gate_latched_ = false;
  hcs01_stale_warn_ms_ = 0;
  hcs01_ot_fail_warn_ms_ = 0;
  if (udp_bound_) {
    udp_.close();
    udp_bound_ = false;
  }
}

}  // namespace eip
