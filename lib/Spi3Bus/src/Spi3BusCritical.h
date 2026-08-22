/**
 * @file Spi3BusCritical.h
 * @brief Class 1 SPI critical-section counter (host-testable, no FreeRTOS).
 */
#pragma once

#include <atomic>

namespace spi3 {

class Class1Critical {
public:
    void enter() { count_.fetch_add(1, std::memory_order_acq_rel); }

    void exit() {
        const int v = count_.fetch_sub(1, std::memory_order_acq_rel);
        if (v <= 0) {
            count_.store(0, std::memory_order_release);
        }
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
