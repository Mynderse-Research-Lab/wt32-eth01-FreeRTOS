// Per-task CPU time around a Class 1 loop gap (see EipRunTimeTrace.h).

#include "EipRunTimeTrace.h"

namespace eip {

void RunTimeHistory::push(const RunTimeSnapshot& s) {
    buf_[count_ % kRunTimeHistoryDepth] = s;
    ++count_;
}

const RunTimeSnapshot* RunTimeHistory::atOrBefore(uint32_t t_us) const {
    const size_t n = size();
    // Newest first; the signed difference survives the 32-bit wrap.
    for (size_t k = 0; k < n; ++k) {
        const RunTimeSnapshot& s = buf_[(count_ - 1 - k) % kRunTimeHistoryDepth];
        if (static_cast<int32_t>(t_us - s.t_us) >= 0) {
            return &s;
        }
    }
    return nullptr;
}

size_t diffRunTimes(const RunTimeSnapshot& before, const RunTimeSnapshot& after,
                    TaskRunDelta* out, size_t max_out) {
    if (out == nullptr || max_out == 0) {
        return 0;
    }
    size_t n = 0;
    for (size_t i = 0; i < after.count; ++i) {
        const TaskRunTime& a = after.tasks[i];
        uint32_t base = 0;
        for (size_t j = 0; j < before.count; ++j) {
            if (before.tasks[j].task_number == a.task_number) {
                base = before.tasks[j].run_us;
                break;
            }
        }
        const uint32_t delta = a.run_us - base;
        if (delta == 0) {
            continue;
        }
        // Insertion into the sorted output; drop the smallest when full.
        size_t pos = n;
        while (pos > 0 && out[pos - 1].delta_us < delta) {
            --pos;
        }
        if (pos >= max_out) {
            continue;
        }
        const size_t last = (n < max_out) ? n : max_out - 1;
        for (size_t k = last; k > pos; --k) {
            out[k] = out[k - 1];
        }
        out[pos].task_number = a.task_number;
        out[pos].delta_us = delta;
        out[pos].core = a.core;
        if (n < max_out) {
            ++n;
        }
    }
    return n;
}

}  // namespace eip
