// Multi-axis Class 1 scanner: T->O on shared UDP 2222 (demux by CID);
// O->T is one W5500 UDP socket per dest (see EipSocketW5500Udp).
//
// Mirrors the PC dual-hold pattern (tools/eip_test.py): one UDP listener,
// one ForwardOpen per axis, O->T keepalive while opening the peer so the
// first connection does not time out. Host-testable (no FreeRTOS).

#ifndef ETHERNET_IP_EIP_MULTI_SCANNER_H
#define ETHERNET_IP_EIP_MULTI_SCANNER_H

#include <cstddef>
#include <cstdint>
#include <memory>

#include "EipConnectionManager.h"
#include "EipClass1Timing.h"
#include "EipIoConnection.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipSession.h"
#include "EipTransport.h"

namespace eip {

struct MultiAxisSlot {
  ScannerConfig config;
  EipProcessImage* image = nullptr;
};

class EipMultiScanner {
 public:
  static constexpr size_t kMaxAxes = 3;
  // Hard cap so a flood cannot stall Class 1. Cyclic drain also stops once
  // every connected axis has one T->O this cycle.
  static constexpr size_t kMaxDrainPerCycle = 16;

  EipMultiScanner(ITcpClient** tcp, size_t axis_count, IUdpEndpoint& udp,
                  const MultiAxisSlot* slots);

  // Open one axis (RegisterSession + ForwardOpen). Call bindSharedUdp() after
  // the first successful open. For dual-axis: keep pumping sendKeepaliveAll()
  // (from another task) while openAxis(1) runs.
  bool openAxis(size_t i);

  // Bind shared Class 1 UDP 2222 (P2P). Call once after the first openAxis.
  bool bindSharedUdp();

  // Convenience: open all axes in order, binding UDP after the first.
  // Does NOT interleave keepalive during a blocking ForwardOpen — firmware
  // should call openAxis / bindSharedUdp / keepalive-task / openAxis instead.
  bool connect();

  // Send O->T for every connected axis (no recv). Used as FO keepalive.
  bool sendKeepaliveAll();

  // Drain already-buffered T->O (non-blocking). Call during HoldKA / after FO
  // so the shared UDP RX cannot overflow before the cyclic loop starts.
  size_t drainBufferedInputs(size_t max_n = kMaxDrainPerCycle);

  // Seed feedback clocks for all connected axes. Call when the Class 1 cyclic
  // loop actually starts — not at ForwardOpen (FO→first-exchange gap is not
  // starvation). Also invoked automatically on the first exchangeOnce.
  void beginCyclicExchange();

  // Send O->T to all axes, then drain whatever T->O is already buffered
  // (non-blocking). Does not wait for drive phase. Returns kInputMiss when
  // any axis feedback is older than 3x granted RPI *after* cyclic start
  // (FO time is never treated as fresh feedback).
  ExchangeStatus exchangeOnce(uint32_t recv_timeout_ms = 0);

  /// True if axis i received T->O in the last exchangeOnce drain.
  bool axisReceivedLastCycle(size_t i) const;

  /// Microseconds since axis i last applied fresh T->O (UINT32_MAX if never).
  uint32_t axisFeedbackAgeUs(size_t i) const;

  void disconnect();

  size_t axisCount() const { return axis_count_; }
  bool axisConnected(size_t i) const;
  const ForwardOpenReply& openReply(size_t i) const;
  uint32_t recvTimeoutMs() const;
  bool udpBound() const { return udp_bound_; }

 private:
  enum class AxisState { kIdle, kRegistered, kConnected };

  struct AxisRuntime {
    ScannerConfig config;
    EipProcessImage* image = nullptr;
    ITcpClient* tcp = nullptr;
    std::unique_ptr<EipSession> session;
    std::unique_ptr<EipIoConnection> io;
    ForwardOpenParams open_params{};
    ForwardOpenReply open_reply{};
    Bytes idle_output;
    Bytes ot_cmd_scratch;    // reused each cycle (avoid vector alloc)
    Bytes ot_frame_scratch;  // reused CPF frame buffer
    AxisState state = AxisState::kIdle;
    uint32_t to_connection_id = 0;
    uint32_t target_ip_host = 0;  // cached at openAxis; Class 1 send path only
    // 0 = never seeded / never received. Set by beginCyclicExchange or
    // applyFeedback — never by ForwardOpen success.
    int64_t last_feedback_us = 0;
    bool received_to_since_cyclic_ = false;
  };

  bool forwardOpenAxis(size_t i);
  bool forwardCloseAxis(size_t i);
  bool sendAxisOutput(size_t i);
  bool applyFeedback(size_t i, const uint8_t* assembly, size_t len);
  size_t matchAxisByConnectionId(uint32_t connection_id) const;
  Bytes buildIdleOutput(const ScannerConfig& cfg) const;
  void configureIo(size_t i);

  IUdpEndpoint& udp_;
  size_t axis_count_ = 0;
  AxisRuntime axes_[kMaxAxes];
  bool udp_bound_ = false;
  size_t ot_rotate_ = 0;
  bool last_got_[kMaxAxes] = {};
  int64_t cyclic_started_us_ = 0;
  bool ot_sent_since_cyclic_ = false;
  // True after grace ends and feedback clocks were reseeded for the stale window.
  bool stale_gate_latched_ = false;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_MULTI_SCANNER_H
