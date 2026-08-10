#include "EipClass1TimingStats.h"

#include <algorithm>
#include <chrono>

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#endif

namespace eip {

namespace {

void sortCopy(const uint32_t* src, size_t n, uint32_t* dst) {
  for (size_t i = 0; i < n; ++i) dst[i] = src[i];
  std::sort(dst, dst + n);
}

uint32_t percentileSorted(const uint32_t* sorted, size_t n, unsigned pct) {
  if (n == 0) return 0;
  if (pct >= 100) return sorted[n - 1];
  const size_t idx = (static_cast<size_t>(pct) * (n - 1)) / 100;
  return sorted[idx];
}

}  // namespace

void Class1TimingStats::Ring::push(uint32_t us) {
  samples[next] = us;
  next = (next + 1) % kClass1TimingRingSize;
  if (filled < kClass1TimingRingSize) ++filled;
  ++total;
}

Class1TimingSnapshot Class1TimingStats::Ring::snapshot() const {
  Class1TimingSnapshot out;
  out.count = static_cast<uint32_t>(filled);
  if (filled == 0) return out;

  uint32_t sorted[kClass1TimingRingSize];
  // Reconstruct chronological buffer: oldest at next when full.
  if (filled < kClass1TimingRingSize) {
    sortCopy(samples, filled, sorted);
  } else {
    size_t j = 0;
    for (size_t i = next; i < kClass1TimingRingSize; ++i) sorted[j++] = samples[i];
    for (size_t i = 0; i < next; ++i) sorted[j++] = samples[i];
    std::sort(sorted, sorted + filled);
  }

  out.min_us = sorted[0];
  out.max_us = sorted[filled - 1];
  out.p50_us = percentileSorted(sorted, filled, 50);
  out.p99_us = percentileSorted(sorted, filled, 99);
  return out;
}

void Class1TimingStats::reset() {
  exchange_ = Ring{};
  ot_send_ = Ring{};
  cycle_ = Ring{};
  cmd_to_start_ = Ring{};
  exchange_count_ = 0;
  start_pending_ = false;
  start_published_us_ = 0;
}

void Class1TimingStats::recordExchangeUs(uint32_t us) {
  exchange_.push(us);
  ++exchange_count_;
}

void Class1TimingStats::recordOtSendUs(uint32_t us) { ot_send_.push(us); }

void Class1TimingStats::recordCycleUs(uint32_t us) { cycle_.push(us); }

void Class1TimingStats::recordCmdToStartUs(uint32_t us) {
  cmd_to_start_.push(us);
}

void Class1TimingStats::noteAbsoluteStartMotionPublished(int64_t now_us) {
  start_pending_ = true;
  start_published_us_ = now_us;
}

void Class1TimingStats::noteOtAssemblySent(const uint8_t* assembly, size_t len,
                                           int64_t now_us) {
  if (!start_pending_ || assembly == nullptr || len < 2) return;
  // Kinetix 104 control byte1 bit4 = StartMotion.
  if ((assembly[1] & 0x10) == 0) return;
  const int64_t dt = now_us - start_published_us_;
  start_pending_ = false;
  if (dt < 0) return;
  const uint32_t us =
      (dt > static_cast<int64_t>(UINT32_MAX)) ? UINT32_MAX
                                              : static_cast<uint32_t>(dt);
  recordCmdToStartUs(us);
}

Class1TimingSnapshot Class1TimingStats::exchange() const {
  return exchange_.snapshot();
}
Class1TimingSnapshot Class1TimingStats::otSend() const {
  return ot_send_.snapshot();
}
Class1TimingSnapshot Class1TimingStats::cycle() const {
  return cycle_.snapshot();
}
Class1TimingSnapshot Class1TimingStats::cmdToStart() const {
  return cmd_to_start_.snapshot();
}

Class1TimingStats& class1TimingStats() {
  static Class1TimingStats s;
  return s;
}

int64_t class1NowUs() {
#if defined(ESP_PLATFORM)
  return esp_timer_get_time();
#else
  using clock = std::chrono::steady_clock;
  return std::chrono::duration_cast<std::chrono::microseconds>(
             clock::now().time_since_epoch())
      .count();
#endif
}

}  // namespace eip
