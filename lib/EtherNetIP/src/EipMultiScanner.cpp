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
  deferred_log_.push(Class1LogEvent::kOtSendFailed, static_cast<uint8_t>(i),
                     static_cast<uint32_t>(static_cast<int32_t>(sent)),
                     static_cast<uint32_t>(frame.size()), ax.target_ip_host);
  return false;
}

bool EipMultiScanner::sendKeepaliveAll() {
  bool any = false;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    if (!sendAxisOutput(i)) {
      if (axes_[i].config.drive_family == ScannerConfig::DriveFamily::kHcs01) {
        deferred_log_.push(Class1LogEvent::kHcs01KeepaliveFailed,
                           static_cast<uint8_t>(i));
        continue;
      }
      return false;
    }
    any = true;
  }
  return any;
}

size_t EipMultiScanner::drainBufferedInputs(size_t max_n) {
  if (max_n > kMaxDrainPerCycle) max_n = kMaxDrainPerCycle;
  // One burst holds every axis' Class 1 CPF (tens of bytes each). Do not put
  // EipSession::kMaxFrameSize (4 KiB) on the HoldKA stack (4 KiB) — that
  // overflowed TLSF after theta FO.
  constexpr size_t kClass1UdpBatchMax = 512;
  uint8_t buf[kClass1UdpBatchMax];
  UdpDatagramView views[kMaxDrainPerCycle];
  size_t applied = 0;
  size_t need = 0;
  for (size_t i = 0; i < axis_count_; ++i) {
    last_got_[i] = false;
    if (axes_[i].state == AxisState::kConnected) ++need;
  }
  size_t drained = 0;
  bool done = false;
  while (!done && drained < max_n) {
    const size_t got =
        udp_.recvBatch(buf, sizeof(buf), views, max_n - drained);
    if (got == 0) break;

    for (size_t v = 0; v < got && !done; ++v) {
      ++drained;
      const size_t n = views[v].len;

      uint32_t cid = 0;
      const uint8_t* assy = nullptr;
      size_t assy_len = 0;
      if (!parseClass1InputCpfView(views[v].data, n, cid, assy, assy_len,
                                   false)) {
        continue;
      }

      const size_t idx = matchAxisByConnectionId(cid);
      if (idx >= kMaxAxes) {
        static uint32_t last_unmatched_cid = 0;
        if (cid != last_unmatched_cid) {
          last_unmatched_cid = cid;
          deferred_log_.push(Class1LogEvent::kUnmatchedCid, 0, cid,
                             static_cast<uint32_t>(n));
        }
        continue;
      }
      if (last_got_[idx]) continue;
      last_got_[idx] = true;
      if (!axes_[idx].first_to_logged) {
        axes_[idx].first_to_logged = true;
        deferred_log_.push(Class1LogEvent::kFirstToReceived,
                           static_cast<uint8_t>(idx),
                           static_cast<uint32_t>(assy_len),
                           static_cast<uint32_t>(axes_[idx].config.to_assembly_size),
                           cid);
      }
      (void)applyFeedback(idx, assy, assy_len);
      ++applied;
      // One fresh T->O per connected axis is enough this cycle.
      if (need > 0 && applied >= need) done = true;
    }
  }
  return applied;
}

bool EipMultiScanner::applyFeedback(size_t i, const uint8_t* assembly,
                                    size_t len) {
  const size_t expect = axes_[i].config.to_assembly_size;
  if (expect > 0 && len > expect) {
    len = expect;
  }
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
    if (!sendAxisOutput(i)) {
      if (axes_[i].config.drive_family == ScannerConfig::DriveFamily::kHcs01) {
        const uint32_t now_ms = static_cast<uint32_t>(class1NowUs() / 1000);
        if (shouldWarnSoftMiss(hcs01_ot_fail_warn_ms_, now_ms)) {
          deferred_log_.push(Class1LogEvent::kHcs01OtFailed,
                             static_cast<uint8_t>(i));
        }
        continue;
      }
      return ExchangeStatus::kOutputSendFailed;
    }
  }
  ot_sent_since_cyclic_ = true;
  if (axis_count_ > 0) {
    ot_rotate_ = (ot_rotate_ + 1) % axis_count_;
  }
  {
    const int64_t dt = class1NowUs() - t_ot0;
    if (dt >= 0 && dt <= static_cast<int64_t>(UINT32_MAX)) {
      class1TimingStats().recordOtSendUs(static_cast<uint32_t>(dt));
      last_ot_us_ = static_cast<uint32_t>(dt);
    }
  }

  {
    const int64_t t_drain0 = class1NowUs();
    (void)drainBufferedInputs(kMaxDrainPerCycle);
    const int64_t dt = class1NowUs() - t_drain0;
    if (dt >= 0 && dt <= static_cast<int64_t>(UINT32_MAX)) {
      class1TimingStats().recordToDrainUs(static_cast<uint32_t>(dt));
      last_drain_us_ = static_cast<uint32_t>(dt);
    }
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

  bool any_kinetix = false;
  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    if (axes_[i].config.drive_family != ScannerConfig::DriveFamily::kHcs01) {
      any_kinetix = true;
      break;
    }
  }

  for (size_t i = 0; i < axis_count_; ++i) {
    if (axes_[i].state != AxisState::kConnected) continue;
    const int64_t last = axes_[i].last_feedback_us;
    const uint32_t age =
        (last > 0 && now >= last)
            ? static_cast<uint32_t>(now - last)
            : UINT32_MAX;
    if (!shouldTeardownAfterStaleUs(age, rpi_us)) continue;
    if (axes_[i].config.drive_family == ScannerConfig::DriveFamily::kHcs01 &&
        any_kinetix) {
      const uint32_t now_ms = static_cast<uint32_t>(now / 1000);
      if (shouldWarnSoftMiss(hcs01_stale_warn_ms_, now_ms, 1000)) {
        deferred_log_.push(Class1LogEvent::kHcs01Stale, static_cast<uint8_t>(i),
                           age);
      }
      // The drive dropped its side long ago (16x RPI); a new ForwardOpen is
      // the only way back. The task performs it outside the exchange.
      if (!axes_[i].reopen_pending && age >= kHcs01ReopenAfterStaleUs) {
        axes_[i].reopen_pending = true;
        axes_[i].reopen_due_us = now;
        axes_[i].reopen_backoff_ms = kAxisReopenInitialBackoffMs;
        deferred_log_.push(Class1LogEvent::kAxisReopenRequested,
                           static_cast<uint8_t>(i), age);
      }
      if (axes_[i].image != nullptr) {
        axes_[i].image->setOnline(false);
      }
      continue;
    }
    deferred_log_.push(Class1LogEvent::kStaleTeardown, static_cast<uint8_t>(i),
                       age, rpi_us);
    return ExchangeStatus::kInputMiss;
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

size_t EipMultiScanner::flushDeferredLogs() {
  size_t flushed = 0;
  Class1LogEntry e;
  while (deferred_log_.pop(e)) {
    ++flushed;
#ifdef ESP_PLATFORM
    const unsigned ax = static_cast<unsigned>(e.axis);
    switch (e.event) {
      case Class1LogEvent::kFirstToReceived:
        ESP_LOGI(kTag, "axis%u first T->O assy=%lu expect=%lu cid=0x%08lX", ax,
                 static_cast<unsigned long>(e.a), static_cast<unsigned long>(e.b),
                 static_cast<unsigned long>(e.c));
        break;
      case Class1LogEvent::kUnmatchedCid:
        ESP_LOGW(kTag, "T->O unmatched cid=0x%08lX n=%lu",
                 static_cast<unsigned long>(e.a), static_cast<unsigned long>(e.b));
        break;
      case Class1LogEvent::kOtSendFailed:
        ESP_LOGW(kTag, "axis%u O->T send failed dest=%u.%u.%u.%u (%d of %lu)", ax,
                 static_cast<unsigned>((e.c >> 24) & 0xFFu),
                 static_cast<unsigned>((e.c >> 16) & 0xFFu),
                 static_cast<unsigned>((e.c >> 8) & 0xFFu),
                 static_cast<unsigned>(e.c & 0xFFu),
                 static_cast<int>(static_cast<int32_t>(e.a)),
                 static_cast<unsigned long>(e.b));
        break;
      case Class1LogEvent::kHcs01OtFailed:
        ESP_LOGW(kTag, "axis%u HCS01 O->T failed — X/Z stay up", ax);
        break;
      case Class1LogEvent::kHcs01KeepaliveFailed:
        ESP_LOGW(kTag, "axis%u HCS01 keepalive O->T failed — X/Z keep pumping", ax);
        break;
      case Class1LogEvent::kHcs01Stale:
        ESP_LOGW(kTag, "axis%u HCS01 T->O stale — X/Z stay up (age=%lu us)", ax,
                 static_cast<unsigned long>(e.a));
        break;
      case Class1LogEvent::kStaleTeardown:
        ESP_LOGW(kTag, "axis%u T->O stale (age=%lu us rpi=%lu) — teardown", ax,
                 static_cast<unsigned long>(e.a), static_cast<unsigned long>(e.b));
        break;
      case Class1LogEvent::kAxisReopenRequested:
        ESP_LOGW(kTag, "axis%u silent %lu us — per-axis reopen (X/Z stay up)", ax,
                 static_cast<unsigned long>(e.a));
        break;
      case Class1LogEvent::kAxisReopenOk:
        ESP_LOGI(kTag, "axis%u reopen OK", ax);
        break;
      case Class1LogEvent::kAxisReopenFailed:
        ESP_LOGW(kTag, "axis%u reopen failed — retry in %lu ms", ax,
                 static_cast<unsigned long>(e.a));
        break;
      case Class1LogEvent::kEncapNopFailed:
        ESP_LOGW(kTag, "axis%u explicit TCP session lost: NOP failed (every %lu ms)."
                 " Class 1 carries on; the close rebuilds the session", ax,
                 static_cast<unsigned long>(e.a));
        break;
      default:
        ESP_LOGW(kTag, "axis%u unknown Class 1 event %d", ax,
                 static_cast<int>(e.event));
        break;
    }
#endif
  }
  const uint32_t dropped = deferred_log_.takeDropped();
#ifdef ESP_PLATFORM
  if (dropped > 0) {
    ESP_LOGW(kTag, "Class 1 log queue dropped %lu event(s)",
             static_cast<unsigned long>(dropped));
  }
#else
  (void)dropped;
#endif
  return flushed;
}

}  // namespace eip
