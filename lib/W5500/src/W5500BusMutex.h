// Shared SPI bus mutex for W5500 chip + socket register access.
#ifndef W5500_BUS_MUTEX_H
#define W5500_BUS_MUTEX_H

#include <cstdint>
#include <mutex>

namespace w5500 {

// Longest hold and longest wait on the bus mutex since the last take, with
// the FreeRTOS task names (dropout diagnostics: who blocks Class 1).
struct BusHoldStats {
    uint32_t max_hold_us = 0;
    char holder[16] = "";
    uint32_t max_wait_us = 0;
    char waiter[16] = "";
};

/// std::mutex that times each hold and wait (BasicLockable for lock_guard).
class BusMutex {
public:
    void lock();
    void unlock();

    /// Copy the stats and start a new window. Never takes the bus mutex, so a
    /// low-priority caller cannot hold off Class 1 (G-B6).
    BusHoldStats take();

private:
    std::mutex m_;
    int64_t acquired_us_ = 0;
    BusHoldStats stats_;
};

/// Process-wide SPI mutex (chip recover + socket I/O).
BusMutex& spiBusMutex();

}  // namespace w5500

#endif  // W5500_BUS_MUTEX_H
