// EtherNet/IP Class 1 scanner lifecycle orchestrator (pure, host-testable).
//
// RegisterSession -> ForwardOpen -> cyclic O->T/T->O exchange -> ForwardClose.
// No FreeRTOS or socket blocking policy; the caller supplies RPI timing and
// reconnect logic (see EipScannerTask.cpp).

#ifndef ETHERNET_IP_EIP_SCANNER_H
#define ETHERNET_IP_EIP_SCANNER_H

#include <cstddef>
#include <cstdint>

#include "EipConnectionManager.h"
#include "EipIoConnection.h"
#include "EipProcessImage.h"
#include "EipSession.h"
#include "EipTransport.h"

namespace eip {

struct ScannerConfig {
  enum class DriveFamily { kKinetix5100, kHcs01 };

  const char* target_ip = nullptr;
  /// Cached host-order IPv4 for Class 1 sendTo (0 = parse target_ip once at FO).
  uint32_t target_ip_host = 0;
  DriveFamily drive_family = DriveFamily::kKinetix5100;
  // Kinetix 5100 EDS: config assembly instance 0xBF (191) is a dummy placeholder.
  // Confirmed working via PC-side iterative testing.
  uint16_t config_assembly_instance = 191;
  uint16_t ot_assembly_instance = 104;
  uint16_t to_assembly_instance = 154;
  size_t ot_assembly_size = 40;
  size_t to_assembly_size = 52;
  uint32_t ot_rpi_us = 20000;
  uint32_t to_rpi_us = 20000;
  uint32_t ot_connection_id = 0x10000001;
  uint32_t to_connection_id = 0;
  uint16_t connection_serial = 0x0001;
  uint16_t originator_vendor_id = 0;
  uint32_t originator_serial = 0xCAFEB00D;
  uint8_t connection_timeout_multiplier = 4;
  bool include_run_idle_header = true;
  /// When true, set bit 8 (Run/Idle header format) in O->T network params.
  /// When false, omit bit 8 even when include_run_idle_header is true
  /// (required by Kinetix 5100 which wants the 4-byte header on the wire
  /// but does not conform to the standard bit-8 signalling convention).
  bool include_run_idle_bit_in_net_params = false;
  ConnectionType to_connection_type = ConnectionType::kMulticast;
};

class EipScanner {
 public:
  enum class State { kIdle, kRegistered, kConnected };

  EipScanner(ITcpClient& tcp, IUdpEndpoint& udp, const ScannerConfig& config);

  // TCP connect, RegisterSession, ForwardOpen, UDP bind. Returns false on any
  // failure (caller should disconnect and retry).
  bool connect();

  // Send idle O->T assembly and receive one T->O frame within recv_timeout_ms.
  bool exchangeOnce(uint32_t recv_timeout_ms, Bytes& out_to_assembly);

  // ForwardClose (best effort), UnRegisterSession, close sockets.
  void disconnect();

  State state() const { return state_; }
  const ForwardOpenReply& openReply() const { return open_reply_; }

  // Recommended T->O receive timeout: granted T->O API * multiplier.
  uint32_t recvTimeoutMs() const;

  // Optional shared process image for axis command/feedback (default nullptr).
  void setProcessImage(EipProcessImage* image) { process_image_ = image; }

 private:
  bool forwardOpen();
  bool forwardClose();
  Bytes buildConnectionPath() const;
  Bytes buildIdleOutputAssembly() const;
  void configureIoFromOpenReply();

  ITcpClient& tcp_;
  IUdpEndpoint& udp_;
  ScannerConfig config_;
  EipSession session_;
  EipIoConnection io_;
  ForwardOpenParams open_params_{};
  ForwardOpenReply open_reply_{};
  Bytes idle_output_;
  Bytes ot_frame_scratch_;
  State state_ = State::kIdle;
  EipProcessImage* process_image_ = nullptr;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_SCANNER_H
