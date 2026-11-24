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

namespace {

using CellNet::CellNetL2Framing;
using CellNet::ParsedFrame;
namespace Ota = CellNet::Ota;

constexpr uint8_t kCmdRebootToOta = 7;
constexpr uint16_t kHbOtaMode = 0x8000;            // HEARTBEAT status_flags bit 15
constexpr uint16_t kHbErasing = 0x4000;            // bit 14
constexpr unsigned kWindow = 18;                   // section 5.1, W
constexpr int64_t kDiscoverWaitUs = 3000000;       // one GANTRY_STATUS before the reboot
constexpr int64_t kRebootResendUs = 1000000;
constexpr int kRebootMaxSends = 5;
constexpr int64_t kHeartbeatWaitUs = 30000000;
constexpr int64_t kBeginResendUs = 1000000;
constexpr int64_t kBeginBaseWaitUs = 2000000;      // + 3.5 us per image byte
constexpr int64_t kStatusTimeoutUs = 100000;
constexpr int kMaxTimeouts = 20;                   // consecutive, same block
constexpr int kMaxMissingRounds = 64;              // resend rounds, same block
constexpr int64_t kEndResendUs = 2000000;
constexpr int64_t kEndWaitUs = 10000000;
constexpr int64_t kResultGraceUs = 1000000;        // OTA_STATUS DONE seen, no result yet
constexpr int kMaxEndRounds = 5;
constexpr int64_t kNewImageWaitUs = 30000000;
constexpr unsigned kProgressEvery = 64;            // blocks between progress log lines
constexpr int64_t kProgressIntervalUs = 100000;    // progress callbacks in the block phase

enum ExitCode : int {
    kExitOk = 0,
    kExitSetup = 1,
    kExitFail = 2,
    kExitUnknown = 3,
    kExitEmacMissed = 4,
    kExitNotConfirmed = 5,
};

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

std::string strf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

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

struct Mac {
    uint8_t b[6] = {};
};

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

/** AF_PACKET raw socket on one interface, CellNet EtherType only. */
class Link {
public:
    ~Link()
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    bool open(const std::string& ifname)
    {
        const size_t len = ifname.size();
        if (len == 0 || len >= IFNAMSIZ) {
            error_ = strf("bad interface name '%s'", ifname.c_str());
            return false;
        }
        fd_ = socket(AF_PACKET, SOCK_RAW, htons(CELL_NET_L2_ETHERTYPE));
        if (fd_ < 0) {
            setErrno("socket(AF_PACKET) (needs CAP_NET_RAW)");
            return false;
        }
        ifreq ifr{};
        std::memcpy(ifr.ifr_name, ifname.c_str(), len + 1);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            setErrno("SIOCGIFINDEX");
            return false;
        }
        ifindex_ = ifr.ifr_ifindex;
        if (ioctl(fd_, SIOCGIFHWADDR, &ifr) < 0) {
            setErrno("SIOCGIFHWADDR");
            return false;
        }
        std::memcpy(mac_.b, ifr.ifr_hwaddr.sa_data, 6);

        sockaddr_ll sll{};
        sll.sll_family = AF_PACKET;
        sll.sll_protocol = htons(CELL_NET_L2_ETHERTYPE);
        sll.sll_ifindex = ifindex_;
        if (bind(fd_, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
            setErrno("bind");
            return false;
        }
#ifdef PACKET_IGNORE_OUTGOING
        // Our own line-rate OTA_DATA would otherwise be looped back into the
        // receive queue. recvFrame also skips PACKET_OUTGOING for older kernels.
        int one = 1;
        setsockopt(fd_, SOL_PACKET, PACKET_IGNORE_OUTGOING, &one, sizeof(one));
#endif
        int rcvbuf = 1 << 20;
        setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        return true;
    }

    const Mac& mac() const { return mac_; }
    /** The last socket error, set when open, send or recvFrame fails. */
    const std::string& error() const { return error_; }

    /** Send one frame, padded to the 60-byte Ethernet minimum. */
    bool send(std::vector<uint8_t> frame)
    {
        if (frame.size() < CellNetL2Framing::kMinEthFrameLen) {
            frame.resize(CellNetL2Framing::kMinEthFrameLen, 0);
        }
        sockaddr_ll to{};
        to.sll_family = AF_PACKET;
        to.sll_protocol = htons(CELL_NET_L2_ETHERTYPE);
        to.sll_ifindex = ifindex_;
        to.sll_halen = 6;
        std::memcpy(to.sll_addr, frame.data(), 6);
        for (int attempt = 0; attempt < 2000; ++attempt) {
            const ssize_t n = sendto(fd_, frame.data(), frame.size(), 0,
                                     reinterpret_cast<const sockaddr*>(&to), sizeof(to));
            if (n == static_cast<ssize_t>(frame.size())) {
                return true;
            }
            if (n < 0 && (errno == ENOBUFS || errno == EAGAIN || errno == EINTR)) {
                // Transmit queue full: give it 50 us and retry the same bytes.
                timespec ts{};
                ts.tv_nsec = 50000;
                nanosleep(&ts, nullptr);
                continue;
            }
            if (n < 0) {
                setErrno("sendto");
            } else {
                error_ = strf("sendto: short write (%zd of %zu bytes)", n, frame.size());
            }
            return false;
        }
        error_ = "sendto: transmit queue stayed full";
        return false;
    }

    /**
     * Wait until the absolute deadline (CLOCK_MONOTONIC, us) for the next valid
     * CellNet frame from any source. Returns 1 with a frame, 0 at the deadline,
     * -1 on a socket error (see error()).
     */
    int recvFrame(int64_t deadline_us, ParsedFrame& out)
    {
        for (;;) {
            sockaddr_ll from{};
            socklen_t from_len = sizeof(from);
            const ssize_t n = recvfrom(fd_, rx_, sizeof(rx_), MSG_DONTWAIT,
                                       reinterpret_cast<sockaddr*>(&from), &from_len);
            if (n > 0) {
                if (from.sll_pkttype == PACKET_OUTGOING) {
                    continue;
                }
                if (CellNetL2Framing::parseFrame(rx_, static_cast<size_t>(n), out)) {
                    return 1;
                }
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                setErrno("recvfrom");
                return -1;
            }
            const int64_t now = nowUs();
            if (now >= deadline_us) {
                return 0;
            }
            const int64_t rem_us = deadline_us - now;
            const int timeout_ms = static_cast<int>((rem_us + 999) / 1000);
            pollfd p{};
            p.fd = fd_;
            p.events = POLLIN;
            if (poll(&p, 1, timeout_ms) < 0 && errno != EINTR) {
                setErrno("poll");
                return -1;
            }
        }
    }

private:
    void setErrno(const char* what)
    {
        const int e = errno;
        error_ = strf("%s: %s", what, std::strerror(e));
    }

    int fd_ = -1;
    int ifindex_ = 0;
    Mac mac_{};
    std::string error_;
    uint8_t rx_[2048] = {};
};

struct Stats {
    unsigned frames_sent = 0;      // every frame this sender sent
    unsigned data_frames = 0;      // OTA_DATA: first sends + resends + probes
    unsigned first_sends = 0;
    unsigned resends = 0;
    unsigned probes = 0;
    unsigned missing_rx = 0;       // OTA_STATUS MISSING received (all, stale included)
    unsigned timeouts = 0;         // 100 ms waits without a useful status
    unsigned window_stalls = 0;    // a burst stopped because W was reached
    unsigned stale_status = 0;     // other session, old block, or old MISSING
    unsigned bad_processed = 0;    // processed_seq outside (processed, last sent]
    unsigned max_outstanding = 0;
};

class OtaSender {
public:
    OtaSender(Link& link, const Ota::SenderOptions& opt, const std::vector<uint8_t>& image,
              const Ota::ProgressFn& progress, const Ota::LogFn& log)
        : link_(link), session_(opt.session), skip_reboot_(opt.skip_reboot), image_(image),
          image_size_(static_cast<uint32_t>(image.size())), frag_(opt.frag_data),
          progress_(progress), log_(log)
    {
        if (opt.node_mac_known) {
            setNodeMac(opt.node_mac);
        }
        blocks_ = static_cast<uint16_t>((image_size_ + Ota::kBlockSize - 1) / Ota::kBlockSize);
        t_start_ = nowUs();
    }

    void run(Ota::SenderResult& out);

private:
    enum class Kind { FIRST, RESEND, PROBE };

    uint32_t blockLen(uint16_t b) const
    {
        const uint32_t rest = image_size_ - static_cast<uint32_t>(b) * Ota::kBlockSize;
        return rest < Ota::kBlockSize ? rest : Ota::kBlockSize;
    }
    uint16_t fragCount(uint16_t b) const
    {
        return static_cast<uint16_t>((blockLen(b) + frag_ - 1) / frag_);
    }
    uint16_t fragLen(uint16_t b, uint8_t f) const
    {
        const uint32_t rest = blockLen(b) - static_cast<uint32_t>(f) * frag_;
        return static_cast<uint16_t>(rest < frag_ ? rest : frag_);
    }
    uint32_t ts() const { return static_cast<uint32_t>(nowUs() - t_start_); }
    /** OTA_DATA frames sent that the node has not reported as processed. */
    unsigned outstanding() const { return static_cast<uint8_t>(last_seq_ - processed_); }

    void setNodeMac(const uint8_t mac[6]);
    void say(const std::string& line) const;
    int fail(int rc, const std::string& line);
    void report(const char* phase, bool force);
    void fill(Ota::SenderResult& out) const;

    bool sendFrame(const std::vector<uint8_t>& frame);
    bool sendReboot();
    bool sendBegin();
    bool sendEnd();
    bool sendData(uint16_t block, uint8_t frag, bool ack, Kind kind);
    bool flushPending();
    void queueBlock(uint16_t b);
    size_t queueMissing(const L2OtaStatusPayload& st);
    bool updateProcessed(uint8_t p);
    int acceptStatus(const ParsedFrame& pf, L2OtaStatusPayload& st);

    int discover();
    int waitOtaMode();
    int begin();
    int runBlocks();
    int sendBlocks();
    int finish();
    int waitNewImage();

    Link& link_;
    uint16_t session_;
    bool skip_reboot_;
    const std::vector<uint8_t>& image_;
    uint32_t image_size_;
    uint16_t frag_;
    const Ota::ProgressFn& progress_;
    const Ota::LogFn& log_;
    Mac node_mac_{};
    Mac ota_mac_{};
    bool node_known_ = false;
    uint16_t blocks_ = 0;

    uint8_t ctrl_seq_ = 0;     // sequence for every frame other than OTA_DATA
    uint8_t next_seq_ = 1;     // first OTA_DATA carries 1, so "nothing processed" is 0,
    uint8_t last_seq_ = 0;     // which is also what the node reports before any data
    uint8_t processed_ = 0;
    uint16_t last_block_ = 0;  // last OTA_DATA sent, for the probe
    uint8_t last_frag_ = 0;

    uint16_t acked_ = 0;       // blocks below this are RECEIVED; the in-flight block
    std::vector<uint8_t> pending_;   // fragments of block acked_ still to send, ascending
    std::vector<bool> sent_once_;    // per fragment of block acked_
    int timeouts_ = 0;
    int missing_rounds_ = 0;
    int64_t status_deadline_ = 0;

    int old_slot_ = -1;        // active_slot seen before the reboot, if any
    int new_slot_ = -1;        // active_slot of the first GANTRY_STATUS after the result
    bool saw_erasing_ = false;
    bool got_done_ = false;
    bool have_result_ = false;
    L2OtaResultPayload result_{};
    std::string msg_;          // the line behind the latest failure or outcome

    int64_t t_start_ = 0;
    int64_t t_reboot_ = -1;
    int64_t t_heartbeat_ = -1;
    int64_t t_begin_ = -1;
    int64_t t_ready_ = -1;
    int64_t t_last_received_ = -1;
    int64_t t_end_ = -1;
    int64_t t_result_ = -1;
    int64_t t_progress_ = -1;

    Stats stats_;
};

void OtaSender::setNodeMac(const uint8_t mac[6])
{
    std::memcpy(node_mac_.b, mac, 6);
    ota_mac_ = node_mac_;
    ota_mac_.b[0] = static_cast<uint8_t>(ota_mac_.b[0] | 0x02);
    node_known_ = true;
}

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

/**
 * Step 0: one GANTRY_STATUS from the node, to learn its MAC when unknown and
 * the pre-update slot. Without a known MAC the first gantry heard wins.
 */
int OtaSender::discover()
{
    const bool known = node_known_;
    if (known && skip_reboot_) {
        return kExitOk;  // node already in OTA mode: it sends no GANTRY_STATUS
    }
    report("discover", true);
    const int64_t deadline = nowUs() + kDiscoverWaitUs;
    if (known) {
        say(strf("waiting up to %.0f s for GANTRY_STATUS from %s", toMs(kDiscoverWaitUs) / 1000.0,
                 macStr(node_mac_).c_str()));
    } else {
        say(strf("waiting up to %.0f s for GANTRY_STATUS from any gantry (sender 0x%02x) "
                 "to learn the node MAC",
                 toMs(kDiscoverWaitUs) / 1000.0, static_cast<unsigned>(CellNodeId::GANTRY)));
    }
    for (;;) {
        ParsedFrame pf;
        const int rx = link_.recvFrame(deadline, pf);
        if (rx < 0) {
            msg_ = link_.error();
            return kExitSetup;
        }
        if (rx == 0) {
            break;
        }
        if (static_cast<CellMsgType>(pf.cell.msg_type) != CellMsgType::GANTRY_STATUS ||
            pf.cell.sender_id != static_cast<uint8_t>(CellNodeId::GANTRY)) {
            continue;
        }
        if (known && !fromMac(pf, node_mac_)) {
            continue;
        }
        if (!known) {
            setNodeMac(pf.eth.src_mac);
        }
        old_slot_ = pf.payload.gantry_status.active_slot;
        say(strf("pre-update GANTRY_STATUS from %s%s: active_slot %d",
                 macStr(node_mac_).c_str(), known ? "" : " (MAC learned)", old_slot_));
        return kExitOk;
    }
    if (!known) {
        msg_ = strf("no GANTRY_STATUS (sender 0x%02x) within %.0f s: node MAC not learned",
                    static_cast<unsigned>(CellNodeId::GANTRY), toMs(kDiscoverWaitUs) / 1000.0);
        return kExitSetup;
    }
    say(strf("no GANTRY_STATUS from %s within %.0f s; pre-update slot not known yet",
             macStr(node_mac_).c_str(), toMs(kDiscoverWaitUs) / 1000.0));
    return kExitOk;
}

/** Steps 1 and 2: REBOOT_TO_OTA, then the first OTA_MODE heartbeat. */
int OtaSender::waitOtaMode()
{
    report("reboot", true);
    const int64_t t0 = nowUs();
    const int64_t deadline = t0 + kHeartbeatWaitUs;
    int64_t next_send = t0;
    int sends = 0;
    say(strf("waiting up to %.0f s for an OTA_MODE heartbeat from %s",
             toMs(kHeartbeatWaitUs) / 1000.0, macStr(ota_mac_).c_str()));
    for (;;) {
        const int64_t now = nowUs();
        if (!skip_reboot_ && sends < kRebootMaxSends && now >= next_send) {
            if (!sendReboot()) {
                return kExitFail;
            }
            if (sends == 0) {
                t_reboot_ = now;
            }
            ++sends;
            next_send += kRebootResendUs;
            say(strf("REBOOT_TO_OTA session %u sent to %s (%d/%d)",
                     static_cast<unsigned>(session_), macStr(node_mac_).c_str(), sends,
                     kRebootMaxSends));
        }
        if (now >= deadline) {
            return fail(kExitFail, strf("FAIL: no OTA_MODE heartbeat from %s within %.0f s",
                                        macStr(ota_mac_).c_str(),
                                        toMs(kHeartbeatWaitUs) / 1000.0));
        }
        int64_t wake = deadline;
        if (!skip_reboot_ && sends < kRebootMaxSends && next_send < wake) {
            wake = next_send;
        }
        ParsedFrame pf;
        const int rx = link_.recvFrame(wake, pf);
        if (rx < 0) {
            return fail(kExitFail, "FAIL: " + link_.error());
        }
        if (rx == 0) {
            continue;
        }
        const auto type = static_cast<CellMsgType>(pf.cell.msg_type);
        if (type == CellMsgType::HEARTBEAT && fromMac(pf, ota_mac_)) {
            const uint16_t flags = pf.payload.heartbeat.status_flags;
            if ((flags & kHbOtaMode) != 0) {
                t_heartbeat_ = nowUs();
                std::string line =
                    strf("OTA_MODE heartbeat: node uptime %u ms, flags 0x%04x",
                         static_cast<unsigned>(pf.payload.heartbeat.uptime_ms),
                         static_cast<unsigned>(flags));
                if (t_reboot_ >= 0) {
                    line += strf(", %.1f ms after the first REBOOT_TO_OTA",
                                 toMs(t_heartbeat_ - t_reboot_));
                }
                say(line);
                return kExitOk;
            }
        } else if (type == CellMsgType::GANTRY_STATUS && fromMac(pf, node_mac_) &&
                   old_slot_ < 0) {
            old_slot_ = pf.payload.gantry_status.active_slot;
            say(strf("pre-update GANTRY_STATUS: active_slot %d", old_slot_));
        }
    }
}

/** Step 3: OTA_BEGIN until READY. */
int OtaSender::begin()
{
    report("begin", true);
    const int64_t t0 = nowUs();
    const int64_t deadline =
        t0 + kBeginBaseWaitUs + static_cast<int64_t>(image_size_) * 7 / 2;
    int64_t next_send = t0;
    int sends = 0;
    t_begin_ = t0;
    for (;;) {
        const int64_t now = nowUs();
        if (now >= next_send) {
            if (!sendBegin()) {
                return kExitFail;
            }
            ++sends;
            next_send += kBeginResendUs;
        }
        if (now >= deadline) {
            return fail(kExitFail, strf("FAIL: no READY within %.1f s (%d OTA_BEGIN sent)",
                                        toMs(deadline - t0) / 1000.0, sends));
        }
        ParsedFrame pf;
        const int rx = link_.recvFrame(next_send < deadline ? next_send : deadline, pf);
        if (rx < 0) {
            return fail(kExitFail, "FAIL: " + link_.error());
        }
        if (rx == 0) {
            continue;
        }
        L2OtaStatusPayload st{};
        const int r = acceptStatus(pf, st);
        if (r < 0) {
            return kExitFail;
        }
        if (r == 0) {
            continue;
        }
        updateProcessed(st.processed_seq);
        if (st.status == code(Ota::Status::READY)) {
            t_ready_ = nowUs();
            say(strf("READY after %.1f ms (%d OTA_BEGIN sent, erasing seen in heartbeat: %s)",
                     toMs(t_ready_ - t_begin_), sends, saw_erasing_ ? "yes" : "no"));
            return kExitOk;
        }
    }
}

/** Step 4, with one progress report on entry and one on the way out. */
int OtaSender::runBlocks()
{
    report("blocks", true);
    const int rc = sendBlocks();
    report("blocks", true);
    return rc;
}

/**
 * Send block acked_ until RECEIVED, then the next one, so block n+1 waits in
 * the node's receive buffers while it writes block n.
 */
int OtaSender::sendBlocks()
{
    if (!flushPending()) {
        return kExitFail;
    }
    while (acked_ < blocks_) {
        ParsedFrame pf;
        const int rx = link_.recvFrame(status_deadline_, pf);
        if (rx < 0) {
            return fail(kExitFail, "FAIL: " + link_.error());
        }
        if (rx == 0) {
            // 100 ms without a useful status.
            ++stats_.timeouts;
            if (++timeouts_ >= kMaxTimeouts) {
                return fail(kExitFail,
                            strf("FAIL: %d consecutive %.0f ms timeouts on block %u "
                                 "(outstanding %u)",
                                 timeouts_, toMs(kStatusTimeoutUs),
                                 static_cast<unsigned>(acked_), outstanding()));
            }
            if (outstanding() < kWindow) {
                if (!pending_.empty()) {
                    if (!flushPending()) {
                        return kExitFail;
                    }
                } else if (!sendData(last_block_, last_frag_, true, Kind::PROBE)) {
                    return kExitFail;
                }
            }
            // Window full: send nothing and wait again (section 5.1).
            status_deadline_ = nowUs() + kStatusTimeoutUs;
            continue;
        }
        L2OtaStatusPayload st{};
        const int r = acceptStatus(pf, st);
        if (r < 0) {
            return kExitFail;
        }
        if (r == 0) {
            continue;
        }
        bool useful = updateProcessed(st.processed_seq);
        if (st.status == code(Ota::Status::RECEIVED)) {
            if (st.block == acked_) {
                t_last_received_ = nowUs();
                ++acked_;
                if (acked_ % kProgressEvery == 0 || acked_ == blocks_) {
                    say(strf("block %u/%u received, %.1f ms since READY",
                             static_cast<unsigned>(acked_), static_cast<unsigned>(blocks_),
                             toMs(t_last_received_ - t_ready_)));
                }
                if (acked_ < blocks_) {
                    queueBlock(acked_);
                } else {
                    pending_.clear();
                }
                report("blocks", false);
                useful = true;
            } else {
                ++stats_.stale_status;  // a block already acknowledged
            }
        } else if (st.status == code(Ota::Status::MISSING)) {
            ++stats_.missing_rx;
            // Only the answer to the last frame sent (always an ACK_REQ frame)
            // reflects everything sent; an older MISSING is stale.
            if (st.block == acked_ && st.processed_seq == last_seq_) {
                if (++missing_rounds_ > kMaxMissingRounds) {
                    return fail(kExitFail, strf("FAIL: %d MISSING rounds on block %u",
                                                missing_rounds_,
                                                static_cast<unsigned>(acked_)));
                }
                queueMissing(st);
                useful = true;
            } else {
                ++stats_.stale_status;
            }
        }
        // READY repeats and DONE: only processed_seq counts here.
        if (useful) {
            timeouts_ = 0;
            status_deadline_ = nowUs() + kStatusTimeoutUs;
        }
        if (!flushPending()) {
            return kExitFail;
        }
    }
    return kExitOk;
}

/** Step 5: OTA_END until OTA_RESULT; MISSING sends the block again. */
int OtaSender::finish()
{
    for (int round = 1;; ++round) {
        if (round > kMaxEndRounds) {
            return fail(kExitFail,
                        strf("FAIL: OTA_END answered with MISSING %d times", kMaxEndRounds));
        }
        report("end", true);
        const int64_t t0 = nowUs();
        int64_t deadline = t0 + kEndWaitUs;
        int64_t next_send = t0;
        bool go_back = false;
        t_end_ = t0;
        while (!go_back) {
            const int64_t now = nowUs();
            if (now >= next_send) {
                if (!sendEnd()) {
                    return kExitFail;
                }
                next_send += kEndResendUs;
            }
            if (now >= deadline) {
                if (got_done_) {
                    return fail(kExitUnknown,
                                "OTA_STATUS DONE but no OTA_RESULT: emac_missed unknown");
                }
                return fail(kExitUnknown,
                            strf("no OTA_RESULT within %.0f s: outcome unknown, not failed",
                                 toMs(kEndWaitUs) / 1000.0));
            }
            ParsedFrame pf;
            const int rx = link_.recvFrame(next_send < deadline ? next_send : deadline, pf);
            if (rx < 0) {
                return fail(kExitFail, "FAIL: " + link_.error());
            }
            if (rx == 0) {
                continue;
            }
            if (static_cast<CellMsgType>(pf.cell.msg_type) == CellMsgType::OTA_RESULT &&
                fromMac(pf, ota_mac_)) {
                const L2OtaResultPayload res = pf.payload.ota_result;
                if (res.session != session_) {
                    ++stats_.stale_status;
                    continue;
                }
                result_ = res;
                have_result_ = true;
                t_result_ = nowUs();
                return kExitOk;
            }
            L2OtaStatusPayload st{};
            const int r = acceptStatus(pf, st);
            if (r < 0) {
                return kExitFail;
            }
            if (r == 0) {
                continue;
            }
            updateProcessed(st.processed_seq);
            if (st.status == code(Ota::Status::DONE)) {
                if (!got_done_) {
                    got_done_ = true;
                    t_result_ = nowUs();
                    say(strf("OTA_STATUS DONE after %.1f ms; waiting for OTA_RESULT",
                             toMs(t_result_ - t_end_)));
                    if (now + kResultGraceUs < deadline) {
                        deadline = now + kResultGraceUs;
                    }
                }
            } else if (st.status == code(Ota::Status::MISSING) && st.block < blocks_) {
                ++stats_.missing_rx;
                say(strf("OTA_END: node still needs block %u; sending from there",
                         static_cast<unsigned>(st.block)));
                acked_ = st.block;
                const uint16_t n = fragCount(acked_);
                sent_once_.assign(n, true);
                timeouts_ = 0;
                missing_rounds_ = 0;
                if (queueMissing(st) == 0) {
                    for (uint16_t f = 0; f < n; ++f) {
                        pending_.push_back(static_cast<uint8_t>(f));
                    }
                }
                go_back = true;
            }
            // RECEIVED for the last block and READY: nothing to do.
        }
        const int rc = runBlocks();
        if (rc != kExitOk) {
            return rc;
        }
    }
}

/** Step 6: the first GANTRY_STATUS from the normal MAC shows the new image. */
int OtaSender::waitNewImage()
{
    report("confirm", true);
    const int64_t t0 = nowUs();
    const int64_t ref = t_result_ >= 0 ? t_result_ : t0;
    const int64_t deadline = t0 + kNewImageWaitUs;
    for (;;) {
        ParsedFrame pf;
        const int rx = link_.recvFrame(deadline, pf);
        if (rx < 0) {
            return fail(kExitFail, "FAIL: " + link_.error());
        }
        if (rx == 0) {
            break;
        }
        if (static_cast<CellMsgType>(pf.cell.msg_type) != CellMsgType::GANTRY_STATUS ||
            !fromMac(pf, node_mac_)) {
            continue;
        }
        const int slot = pf.payload.gantry_status.active_slot;
        new_slot_ = slot;
        say(strf("new image: GANTRY_STATUS from %s, active_slot %d, %.1f ms after %s",
                 macStr(node_mac_).c_str(), slot, toMs(nowUs() - ref),
                 t_result_ >= 0 ? "OTA_RESULT" : "the end of step 5"));
        if (old_slot_ >= 0 && slot == old_slot_) {
            return fail(kExitNotConfirmed,
                        strf("FAIL: active_slot unchanged (%d): not booted into the new image",
                             slot));
        }
        if (old_slot_ < 0) {
            say("(pre-update slot not seen, so the slot change is not checked)");
        }
        return kExitOk;
    }
    return fail(kExitNotConfirmed,
                strf("FAIL: no GANTRY_STATUS from %s within %.0f s: reboot not confirmed",
                     macStr(node_mac_).c_str(), toMs(kNewImageWaitUs) / 1000.0));
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
