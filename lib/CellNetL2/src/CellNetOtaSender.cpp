/**
 * @file CellNetOtaSender.cpp
 * @brief Linux sender for a WT32 firmware update over CellNet L2 (N-2).
 *
 * The logic moved here from tools/cellnet_ota_send.cpp, where it was proven
 * on hardware; see CellNetOtaSender.h for the steps. Linux only: off Linux
 * this file compiles to nothing. Not listed in the ESP component.
 */

#if defined(__linux__)

#include "CellNetOtaSender.h"
#include "CellNetOtaSenderInternal.h"

#include "CellNetL2Framing.h"
#include "CellNetOtaReceiver.h"
#include "cell_net_l2_protocol.h"

#include <arpa/inet.h>
#include <errno.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace cellnet_ota_sender {
constexpr uint8_t code(Ota::Status s)
{
    return static_cast<uint8_t>(s);
}

int64_t nowUs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

double toMs(int64_t us)
{
    return static_cast<double>(us) / 1000.0;
}


std::string strf(const char* fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return std::string();
    }
    if (static_cast<size_t>(n) < sizeof(buf)) {
        return std::string(buf, static_cast<size_t>(n));
    }
    std::string s(static_cast<size_t>(n) + 1, '\0');
    va_start(ap, fmt);
    std::vsnprintf(&s[0], s.size(), fmt, ap);
    va_end(ap);
    s.resize(static_cast<size_t>(n));
    return s;
}

const char* statusName(uint8_t s)
{
    switch (static_cast<Ota::Status>(s)) {
    case Ota::Status::READY: return "READY";
    case Ota::Status::RECEIVED: return "RECEIVED";
    case Ota::Status::MISSING: return "MISSING";
    case Ota::Status::DONE: return "DONE";
    case Ota::Status::ERR_SIZE: return "ERR_SIZE";
    case Ota::Status::ERR_SESSION: return "ERR_SESSION";
    case Ota::Status::ERR_BUSY: return "ERR_BUSY";
    case Ota::Status::ERR_FLASH: return "ERR_FLASH";
    case Ota::Status::ERR_VERIFY: return "ERR_VERIFY";
    case Ota::Status::ERR_TIMEOUT: return "ERR_TIMEOUT";
    }
    return "UNKNOWN";
}


std::string macStr(const Mac& m)
{
    char buf[18];
    std::snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", m.b[0], m.b[1], m.b[2],
                  m.b[3], m.b[4], m.b[5]);
    return std::string(buf);
}

bool fromMac(const ParsedFrame& pf, const Mac& m)
{
    return std::memcmp(pf.eth.src_mac, m.b, 6) == 0;
}


void OtaSender::setNodeMac(const uint8_t mac[6])
{
    std::memcpy(node_mac_.b, mac, 6);
    ota_mac_ = node_mac_;
    ota_mac_.b[0] = static_cast<uint8_t>(ota_mac_.b[0] | 0x02);
    node_known_ = true;
}
bool OtaSender::sendFrame(const std::vector<uint8_t>& frame)
{
    if (!link_.send(frame)) {
        fail(kExitFail, "FAIL: " + link_.error());
        return false;
    }
    ++stats_.frames_sent;
    return true;
}

bool OtaSender::sendReboot()
{
    L2CellCommandPayload cmd{};
    cmd.command_id = kCmdRebootToOta;
    cmd.param_u8 = 0;
    cmd.param_u16 = session_;
    cmd.param_float = 0.0f;
    return sendFrame(CellNetL2Framing::buildCellCommandFrame(
        link_.mac().b, CellNodeId::SUPERVISOR, ctrl_seq_++, ts(), cmd, node_mac_.b));
}

bool OtaSender::sendBegin()
{
    L2OtaBeginPayload p{};
    p.session = session_;
    p.block_size = Ota::kBlockSize;
    p.frag_data = frag_;
    p.reserved = 0;
    p.image_size = image_size_;
    return sendFrame(CellNetL2Framing::buildOtaBeginFrame(
        link_.mac().b, CellNodeId::SUPERVISOR, ctrl_seq_++, ts(), p, ota_mac_.b));
}

bool OtaSender::sendEnd()
{
    L2OtaEndPayload p{};
    p.session = session_;
    p.reserved = 0;
    p.image_size = image_size_;
    return sendFrame(CellNetL2Framing::buildOtaEndFrame(
        link_.mac().b, CellNodeId::SUPERVISOR, ctrl_seq_++, ts(), p, ota_mac_.b));
}

bool OtaSender::sendData(uint16_t block, uint8_t frag, bool ack, Kind kind)
{
    L2OtaDataHeader h{};
    h.session = session_;
    h.block = block;
    h.frag = frag;
    h.flags = ack ? Ota::kFlagAckReq : static_cast<uint8_t>(0);
    h.data_len = fragLen(block, frag);
    const uint8_t* data = image_.data() + static_cast<size_t>(block) * Ota::kBlockSize +
                          static_cast<size_t>(frag) * frag_;
    const uint8_t seq = next_seq_;
    if (!sendFrame(CellNetL2Framing::buildOtaDataFrame(link_.mac().b, CellNodeId::SUPERVISOR,
                                                       seq, ts(), h, data, ota_mac_.b))) {
        return false;
    }
    next_seq_ = static_cast<uint8_t>(next_seq_ + 1);
    last_seq_ = seq;
    last_block_ = block;
    last_frag_ = frag;
    ++stats_.data_frames;
    switch (kind) {
    case Kind::FIRST: ++stats_.first_sends; break;
    case Kind::RESEND: ++stats_.resends; break;
    case Kind::PROBE: ++stats_.probes; break;
    }
    if (outstanding() > stats_.max_outstanding) {
        stats_.max_outstanding = outstanding();
    }
    if (ack) {
        status_deadline_ = nowUs() + kStatusTimeoutUs;
    }
    return true;
}

/**
 * Send pending fragments of block acked_ while the window has room. ACK_REQ
 * goes on the last frame of the burst, and also on the frame that fills the
 * window, so the last frame sent always asks for a status (section 5.1).
 */
bool OtaSender::flushPending()
{
    while (!pending_.empty()) {
        if (outstanding() >= kWindow) {
            ++stats_.window_stalls;
            return true;
        }
        const uint8_t f = pending_.front();
        pending_.erase(pending_.begin());
        const bool ack = pending_.empty() || outstanding() + 1 >= kWindow;
        const Kind kind = sent_once_[f] ? Kind::RESEND : Kind::FIRST;
        sent_once_[f] = true;
        if (!sendData(acked_, f, ack, kind)) {
            return false;
        }
    }
    return true;
}

void OtaSender::queueBlock(uint16_t b)
{
    const uint16_t n = fragCount(b);
    pending_.clear();
    for (uint16_t f = 0; f < n; ++f) {
        pending_.push_back(static_cast<uint8_t>(f));
    }
    sent_once_.assign(n, false);
    timeouts_ = 0;
    missing_rounds_ = 0;
}

/** Replace pending with the fragments of block acked_ the bitmap marks missing. */
size_t OtaSender::queueMissing(const L2OtaStatusPayload& st)
{
    const uint16_t n = fragCount(acked_);
    pending_.clear();
    for (uint16_t f = 0; f < n; ++f) {
        if ((st.missing[f / 8] & (1u << (f % 8))) != 0) {
            pending_.push_back(static_cast<uint8_t>(f));
        }
    }
    return pending_.size();
}

/**
 * Take the node's processed_seq. It names a frame we sent and the node takes
 * frames in order, so a valid value lies in (processed, last sent]; anything
 * else is a reordered or stale status and is ignored. Returns true on advance.
 */
bool OtaSender::updateProcessed(uint8_t p)
{
    const unsigned span = static_cast<uint8_t>(last_seq_ - processed_);
    const unsigned adv = static_cast<uint8_t>(p - processed_);
    if (adv == 0) {
        return false;
    }
    if (adv > span) {
        ++stats_.bad_processed;
        return false;
    }
    processed_ = p;
    return true;
}

/**
 * OTA_STATUS from the OTA MAC: 1 = ours (copied to st), 0 = ignore, -1 = fatal.
 * ERR_SESSION carries the node's session, not ours, so it is fatal whatever
 * its session field says.
 */
int OtaSender::acceptStatus(const ParsedFrame& pf, L2OtaStatusPayload& st)
{
    if (static_cast<CellMsgType>(pf.cell.msg_type) != CellMsgType::OTA_STATUS ||
        !fromMac(pf, ota_mac_)) {
        if (static_cast<CellMsgType>(pf.cell.msg_type) == CellMsgType::HEARTBEAT &&
            fromMac(pf, ota_mac_) && (pf.payload.heartbeat.status_flags & kHbErasing) != 0) {
            saw_erasing_ = true;
        }
        return 0;
    }
    st = pf.payload.ota_status;
    if (st.status == code(Ota::Status::ERR_SESSION)) {
        fail(kExitFail, strf("FAIL: node replied ERR_SESSION (node session %u, ours %u)",
                             static_cast<unsigned>(st.session),
                             static_cast<unsigned>(session_)));
        return -1;
    }
    if (st.session != session_) {
        ++stats_.stale_status;
        return 0;
    }
    if (st.status >= code(Ota::Status::ERR_SIZE)) {
        fail(kExitFail, strf("FAIL: node replied %s (code %u) for block %u",
                             statusName(st.status), static_cast<unsigned>(st.status),
                             static_cast<unsigned>(st.block)));
        return -1;
    }
    return 1;
}
void OtaSender::run(Ota::SenderResult& out)
{
    int rc = discover();
    if (rc == kExitSetup) {
        fill(out);
        out.code = kExitSetup;
        out.message = msg_;
        return;
    }

    const uint16_t last = static_cast<uint16_t>(blocks_ - 1);
    say(strf("interface MAC %s, node %s, OTA address %s", macStr(link_.mac()).c_str(),
             macStr(node_mac_).c_str(), macStr(ota_mac_).c_str()));
    say(strf("image %u bytes, %u blocks (last %u bytes), frag %u -> %u per block, "
             "frame %u bytes, session %u",
             static_cast<unsigned>(image_size_), static_cast<unsigned>(blocks_),
             static_cast<unsigned>(blockLen(last)), static_cast<unsigned>(frag_),
             static_cast<unsigned>(fragCount(0)),
             static_cast<unsigned>(sizeof(L2OtaDataFrameHeader) + frag_),
             static_cast<unsigned>(session_)));

    rc = waitOtaMode();
    if (rc == kExitOk) {
        rc = begin();
    }
    if (rc == kExitOk) {
        queueBlock(0);
        rc = runBlocks();
    }
    if (rc == kExitOk) {
        rc = finish();
    }

    int exit_code = rc;  // kExitFail, kExitOk with a result, or kExitUnknown
    std::string why = msg_;
    if (exit_code != kExitFail && have_result_) {
        if (result_.status != code(Ota::Status::DONE)) {
            exit_code = fail(kExitFail, strf("FAIL: OTA_RESULT status is %s, not DONE",
                                             statusName(result_.status)));
            why = msg_;
        } else if (result_.emac_missed != 0) {
            exit_code = fail(kExitEmacMissed,
                             strf("FAIL: emac_missed = %u: frames were dropped for want of a "
                                  "receive buffer (section 5.1 requires 0)",
                                  static_cast<unsigned>(result_.emac_missed)));
            why = msg_;
        }
    }
    if (exit_code != kExitFail) {
        const int rc6 = waitNewImage();
        if (rc6 == kExitFail || (exit_code == kExitOk && rc6 != kExitOk)) {
            exit_code = rc6;
            why = msg_;
        }
    }
    say(strf("sender detail: first sends %u; window stalls %u; max outstanding %u of %u; "
             "stale statuses %u; out-of-range processed_seq %u",
             stats_.first_sends, stats_.window_stalls, stats_.max_outstanding, kWindow,
             stats_.stale_status, stats_.bad_processed));

    fill(out);
    out.code = exit_code;
    out.ok = exit_code == kExitOk;
    if (exit_code == kExitOk) {
        out.message = strf("OK: %u bytes to %s, active_slot %d -> %d, %.1f ms, emac_missed 0",
                           static_cast<unsigned>(image_size_), macStr(node_mac_).c_str(),
                           old_slot_, new_slot_, out.total_ms);
    } else {
        out.message = why.empty() ? std::string("FAIL: aborted") : why;
    }
}

}  // namespace

namespace CellNet {
namespace Ota {

SenderResult runOtaSender(const SenderOptions& opt, const std::vector<uint8_t>& image,
                          const ProgressFn& progress, const LogFn& log)
{
    using namespace cellnet_ota_sender;
    SenderResult out;
    const uint32_t frags_per_block =
        opt.frag_data == 0
            ? 0
            : (kBlockSize + opt.frag_data - 1u) / static_cast<uint32_t>(opt.frag_data);
    if (opt.frag_data == 0 || opt.frag_data > kMaxFragData || frags_per_block > kMaxFrags) {
        out.code = kExitSetup;
        out.message = strf("frag_data must be %u..%u (at most %u fragments per block)",
                           static_cast<unsigned>((kBlockSize + kMaxFrags - 1) / kMaxFrags),
                           static_cast<unsigned>(kMaxFragData),
                           static_cast<unsigned>(kMaxFrags));
        return out;
    }
    if (image.empty()) {
        out.code = kExitSetup;
        out.message = "image is empty";
        return out;
    }
    if (image.size() > static_cast<size_t>(0xFFFFu) * kBlockSize) {
        out.code = kExitSetup;
        out.message = strf("image too large (%zu bytes)", image.size());
        return out;
    }
    if (opt.node_mac_known) {
        std::memcpy(out.node_mac, opt.node_mac, 6);
    }

    Link link;
    if (!link.open(opt.ifname)) {
        out.code = kExitSetup;
        out.message = link.error();
        return out;
    }
    OtaSender sender(link, opt, image, progress, log);
    sender.run(out);
    return out;
}

}  // namespace Ota
}  // namespace CellNet

#endif  // __linux__
