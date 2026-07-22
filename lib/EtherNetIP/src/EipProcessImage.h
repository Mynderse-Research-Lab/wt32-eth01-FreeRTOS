// Thread-safe O->T command / T->O feedback bridge between EipScanner and axis
// adapters. Mutex-guarded; safe across FreeRTOS tasks (scanner vs GantryUpdate).

#ifndef ETHERNET_IP_EIP_PROCESS_IMAGE_H
#define ETHERNET_IP_EIP_PROCESS_IMAGE_H

#include <mutex>

#include "EipByteBuffer.h"

namespace eip {

class EipProcessImage {
 public:
  void setCommand(const Bytes& command);
  bool getCommand(Bytes& out_command) const;
  bool hasCommand() const;

  void setFeedback(const Bytes& feedback);
  bool getFeedback(Bytes& out_feedback) const;

  // True after setFeedback until consumeFeedbackFresh() or next setFeedback.
  bool feedbackFresh() const;
  void consumeFeedbackFresh();

  void setOnline(bool online);
  bool isOnline() const;

 private:
  mutable std::mutex mutex_;
  Bytes command_;
  Bytes feedback_;
  bool command_valid_ = false;
  bool feedback_fresh_ = false;
  bool online_ = false;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_PROCESS_IMAGE_H
