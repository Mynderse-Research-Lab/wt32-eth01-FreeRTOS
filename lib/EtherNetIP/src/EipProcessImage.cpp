#include "EipProcessImage.h"

namespace eip {

void EipProcessImage::setCommand(const Bytes& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  command_ = command;
  command_valid_ = !command_.empty();
}

bool EipProcessImage::getCommand(Bytes& out_command) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_valid_) return false;
  out_command = command_;
  return true;
}

bool EipProcessImage::hasCommand() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_valid_ && !command_.empty();
}

void EipProcessImage::setFeedback(const Bytes& feedback) {
  std::lock_guard<std::mutex> lock(mutex_);
  feedback_ = feedback;
  feedback_fresh_ = true;
}

bool EipProcessImage::getFeedback(Bytes& out_feedback) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (feedback_.empty()) return false;
  out_feedback = feedback_;
  return true;
}

bool EipProcessImage::feedbackFresh() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return feedback_fresh_;
}

void EipProcessImage::consumeFeedbackFresh() {
  std::lock_guard<std::mutex> lock(mutex_);
  feedback_fresh_ = false;
}

void EipProcessImage::setOnline(bool online) {
  std::lock_guard<std::mutex> lock(mutex_);
  online_ = online;
}

bool EipProcessImage::isOnline() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return online_;
}

}  // namespace eip
