/**
 * @file Spi3BusCritical.h
 * @brief Class 1 SPI critical-section counter (host-testable, no FreeRTOS).
 */
#pragma once

#include <atomic>

namespace spi3 {

class Class1Critical {
public:
    /// Returns true on the idle -> active transition (first enter).
    bool enter() {
        return count_.fetch_add(1, std::memory_order_acq_rel) == 0;
    }

    /// Returns true on the active -> idle transition (last exit). An unmatched
    /// exit clamps to 0 and also reports idle so waiters are never stranded.
    bool exit() {
        const int v = count_.fetch_sub(1, std::memory_order_acq_rel);
        if (v <= 0) {
            count_.store(0, std::memory_order_release);
            return true;
        }
        return v == 1;
    }

    bool active() const {
        return count_.load(std::memory_order_acquire) > 0;
    }

    int load() const { return count_.load(std::memory_order_acquire); }

    void reset() { count_.store(0, std::memory_order_release); }

private:
    std::atomic<int> count_{0};
};

}  // namespace spi3
