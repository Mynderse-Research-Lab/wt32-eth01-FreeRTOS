/**
 * @file CellNetOtaSenderReport.cpp
 * @brief Progress reports and the result record for the CellNet OTA sender.
 */

#if defined(__linux__)

#include "CellNetOtaSenderInternal.h"

#include "CellNetOtaReceiver.h"

namespace cellnet_ota_sender {

void OtaSender::say(const std::string& line) const
{
    if (log_) {
        log_(line);
    }
}

int OtaSender::fail(int rc, const std::string& line)
{
    say(line);
    msg_ = line;
    return rc;
}

void OtaSender::report(const char* phase, bool force)
{
    if (!progress_) {
        return;
    }
    const int64_t now = nowUs();
    if (!force && t_progress_ >= 0 && now - t_progress_ < kProgressIntervalUs) {
        return;
    }
    t_progress_ = now;
    Ota::SenderProgress p{};
    p.phase = phase;
    p.blocks_done = acked_;
    p.blocks_total = blocks_;
    const uint64_t bytes = static_cast<uint64_t>(acked_) * Ota::kBlockSize;
    p.bytes_done = bytes < image_size_ ? static_cast<uint32_t>(bytes) : image_size_;
    p.bytes_total = image_size_;
    if (t_ready_ >= 0 && t_last_received_ > t_ready_) {
        p.kbps = static_cast<double>(p.bytes_done) / 1024.0 /
                 (toMs(t_last_received_ - t_ready_) / 1000.0);
    }
    progress_(p);
}

void OtaSender::fill(Ota::SenderResult& out) const
{
    out.have_result = have_result_;
    out.result = result_;
    out.old_slot = old_slot_;
    out.new_slot = new_slot_;
    if (node_known_) {
        std::memcpy(out.node_mac, node_mac_.b, 6);
    }
    if (t_reboot_ >= 0 && t_heartbeat_ >= 0) {
        out.reboot_to_heartbeat_ms = toMs(t_heartbeat_ - t_reboot_);
    }
    if (t_begin_ >= 0 && t_ready_ >= 0) {
        out.begin_to_ready_ms = toMs(t_ready_ - t_begin_);
    }
    if (t_ready_ >= 0 && t_last_received_ >= 0) {
        out.ready_to_last_ms = toMs(t_last_received_ - t_ready_);
    }
    if (t_end_ >= 0 && t_result_ >= 0) {
        out.end_to_result_ms = toMs(t_result_ - t_end_);
    }
    const int64_t t_first = t_reboot_ >= 0 ? t_reboot_ : t_begin_;
    if (t_result_ >= 0 && t_first >= 0) {
        out.total_ms = toMs(t_result_ - t_first);
    }
    out.frames_sent = stats_.frames_sent;
    out.data_frames = stats_.data_frames;
    out.resends = stats_.resends;
    out.probes = stats_.probes;
    out.missing_received = stats_.missing_rx;
    out.timeouts = stats_.timeouts;
}

}  // namespace cellnet_ota_sender

#endif  // __linux__
