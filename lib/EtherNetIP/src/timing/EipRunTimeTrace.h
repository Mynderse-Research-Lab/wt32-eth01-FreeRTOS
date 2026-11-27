// Per-task CPU time around a Class 1 loop gap (G-B6).
//
// The EipLog task snapshots every task's FreeRTOS run-time counter every few
// tens of milliseconds into a short history. When the scanner reports a long
// gap, the snapshot taken just before the gap is diffed against a fresh one,
// which names the task that used the CPU while the scanner was not running.
// No ISR or tick-hook code is involved.

#ifndef ETHERNET_IP_EIP_RUN_TIME_TRACE_H
#define ETHERNET_IP_EIP_RUN_TIME_TRACE_H

#include <cstddef>
#include <cstdint>

namespace eip {

constexpr size_t kRunTimeMaxTasks = 40;
constexpr size_t kRunTimeHistoryDepth = 8;

struct TaskRunTime {
    uint32_t task_number = 0;  // FreeRTOS xTaskNumber, unique per task
    uint32_t run_us = 0;       // ulRunTimeCounter, wraps at 2^32
    int8_t core = -1;          // pinned core, or -1 for no affinity
};

struct RunTimeSnapshot {
    uint32_t t_us = 0;
    uint8_t count = 0;
    TaskRunTime tasks[kRunTimeMaxTasks];
};

class RunTimeHistory {
public:
    void push(const RunTimeSnapshot& s);

    /// Newest snapshot taken at or before t_us, or nullptr if none is that old.
    const RunTimeSnapshot* atOrBefore(uint32_t t_us) const;

    size_t size() const { return (count_ < kRunTimeHistoryDepth) ? count_ : kRunTimeHistoryDepth; }

private:
    RunTimeSnapshot buf_[kRunTimeHistoryDepth];
    uint32_t count_ = 0;
};

struct TaskRunDelta {
    uint32_t task_number = 0;
    uint32_t delta_us = 0;
    int8_t core = -1;
};

/// CPU time each task gained from `before` to `after`, largest first. Tasks
/// absent from `before` (created since) count from zero.
size_t diffRunTimes(const RunTimeSnapshot& before, const RunTimeSnapshot& after,
                    TaskRunDelta* out, size_t max_out);

}  // namespace eip

#endif  // ETHERNET_IP_EIP_RUN_TIME_TRACE_H
