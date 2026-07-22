// Multi-axis Class 1 scanner: shared UDP 2222, demux T->O by connection ID.
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
  static constexpr size_t kMaxAxes = 2;

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

  // Send O->T to all axes, then drain T->O until each axis has a fresh
  // feedback or recv_timeout_ms elapses. Re-sends O->T on the granted API
  // while waiting so Class 1 does not starve behind T->O. Demux by CID.
  ExchangeStatus exchangeOnce(uint32_t recv_timeout_ms);

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
    AxisState state = AxisState::kIdle;
    uint32_t to_connection_id = 0;
  };

  bool forwardOpenAxis(size_t i);
  bool forwardCloseAxis(size_t i);
  bool sendAxisOutput(size_t i);
  bool applyFeedback(size_t i, const Bytes& assembly);
  size_t matchAxisByConnectionId(uint32_t connection_id) const;
  Bytes buildIdleOutput(const ScannerConfig& cfg) const;
  void configureIo(size_t i);

  IUdpEndpoint& udp_;
  size_t axis_count_ = 0;
  AxisRuntime axes_[kMaxAxes];
  bool udp_bound_ = false;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_MULTI_SCANNER_H
