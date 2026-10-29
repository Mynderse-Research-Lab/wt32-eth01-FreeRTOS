/**
 * @file EipClass1DeferredLog.h
 * @brief Fixed-size event queue so the Class 1 exchange never logs inline.
 *
 * exchangeOnce() runs with the W5500 bus held and the SPI3 Class 1 critical
 * flag set. ESP_LOG* there blocks on the UART and the LAN console TCP send,
 * stretching the critical section past the drive timeout. Events are queued
 * here and printed by the scanner task after the critical section exits.
 * Freestanding C++17 (host-testable, no ESP-IDF).
 */
#ifndef ETHERNET_IP_EIP_CLASS1_DEFERRED_LOG_H
#define ETHERNET_IP_EIP_CLASS1_DEFERRED_LOG_H

#include <cstddef>
#include <cstdint>
#include <mutex>

namespace eip {

enum class Class1LogEvent : uint8_t {
    kFirstToReceived = 0,  // a = assembly bytes, b = expected bytes, c = CID
    kUnmatchedCid,         // a = CID, b = datagram bytes
    kOtSendFailed,         // a = sent (int32), b = frame bytes, c = dest IPv4
    kHcs01OtFailed,        // no payload
    kHcs01KeepaliveFailed, // no payload
    kHcs01Stale,           // a = age us
    kStaleTeardown,        // a = age us, b = rpi us
    kAxisReopenRequested,  // a = age us
    kAxisReopenOk,         // no payload
    kAxisReopenFailed,     // a = next retry ms
};

struct Class1LogEntry {
    Class1LogEvent event = Class1LogEvent::kFirstToReceived;
    uint8_t axis = 0;
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t c = 0;
};

class Class1DeferredLog {
   public:
    static constexpr size_t kCapacity = 16;

    /// Queue an event. Never blocks on I/O; drops (and counts) when full.
    bool push(Class1LogEvent event, uint8_t axis, uint32_t a = 0, uint32_t b = 0,
              uint32_t c = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ >= kCapacity) {
            ++dropped_;
            return false;
        }
        Class1LogEntry& e = entries_[(head_ + count_) % kCapacity];
        e.event = event;
        e.axis = axis;
        e.a = a;
        e.b = b;
        e.c = c;
        ++count_;
        return true;
    }

    /// Pop the oldest event. Returns false when empty.
    bool pop(Class1LogEntry& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == 0) {
            return false;
        }
        out = entries_[head_];
        head_ = (head_ + 1) % kCapacity;
        --count_;
        return true;
    }

    /// Return and reset the number of events dropped since the last call.
    uint32_t takeDropped() {
        std::lock_guard<std::mutex> lock(mutex_);
        const uint32_t d = dropped_;
        dropped_ = 0;
        return d;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = 0;
        count_ = 0;
        dropped_ = 0;
    }

   private:
    mutable std::mutex mutex_;
    Class1LogEntry entries_[kCapacity];
    size_t head_ = 0;
    size_t count_ = 0;
    uint32_t dropped_ = 0;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_CLASS1_DEFERRED_LOG_H
