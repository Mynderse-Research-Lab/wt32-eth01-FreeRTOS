/**
 * @file cellnet_ota_send.cpp
 * @brief Test sender for a WT32 firmware update over CellNet L2 (N-2).
 *
 * Test tool for N-2, used before the supervisor integration: it drives one
 * complete CellNet OTA update from a Linux host (the Raspberry Pi 5) so the
 * node side can be proven end to end. The protocol is
 * docs/CELLNET_OTA_PROTOCOL.md (sections 3 to 7, variant B, window of
 * section 5.1). The supervisor's /api/ota sender replaces this tool later.
 *
 * Build on the Pi, from the repository root:
 *   g++ -O2 -std=c++17 -Wall -Wextra -I lib/CellNetL2/src -I include \
 *       tools/cellnet_ota_send.cpp lib/CellNetL2/src/CellNetL2Framing.cpp \
 *       -o cellnet_ota_send
 *
 * Usage (raw socket, needs CAP_NET_RAW: run as root, or
 * `sudo setcap cap_net_raw+ep ./cellnet_ota_send`):
 *   cellnet_ota_send <ifname> <node-mac> <image.bin> <session> [--frag 456] [--skip-reboot]
 *
 *   ifname        interface on the cell switch, e.g. eth0
 *   node-mac      the node's normal MAC, aa:bb:cc:dd:ee:ff
 *   image.bin     the application image (idf/build/<app>.bin)
 *   session       OTA session, 0..65535 (decimal or 0x hex); use a counter
 *   --frag N      data bytes per OTA_DATA fragment (default 456: 486-byte frames)
 *   --skip-reboot the node is already in OTA mode; do not send REBOOT_TO_OTA
 *
 * Steps:
 *   1. CELL_COMMAND 7 REBOOT_TO_OTA (param_u16 = session) to the node MAC,
 *      every 1 s, at most 5 times.
 *   2. Wait up to 30 s for a HEARTBEAT with OTA_MODE (bit 15) from the OTA MAC
 *      (node MAC with the first octet | 0x02). Everything after goes there.
 *   3. OTA_BEGIN, every 1 s, until READY (at most 2 s + 3.5 us per image byte).
 *   4. Blocks, one block ahead of the node's write, W = 18 unprocessed frames.
 *   5. OTA_END, every 2 s, until OTA_RESULT (at most 10 s).
 *   6. Wait up to 30 s for GANTRY_STATUS from the normal MAC: the new image.
 *
 * Exit codes: 0 DONE with emac_missed == 0 and the new image seen;
 *   1 usage or setup error; 2 protocol failure or abort; 3 no OTA_RESULT
 *   (outcome unknown); 4 emac_missed != 0; 5 reboot into the new image not
 *   confirmed (no GANTRY_STATUS, or the slot did not change).
 */

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

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
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
constexpr uint16_t kDefaultFrag = 456;             // variant B, 486-byte frames
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
constexpr unsigned kProgressEvery = 64;            // blocks between progress lines

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

int hexVal(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

bool parseMac(const char* s, Mac& out)
{
    for (int i = 0; i < 6; ++i) {
        const int hi = hexVal(s[0]);
        if (hi < 0) {
            return false;
        }
        const int lo = hexVal(s[1]);
        if (lo < 0) {
            return false;
        }
        out.b[i] = static_cast<uint8_t>((hi << 4) | lo);
        s += 2;
        if (i < 5) {
            if (*s != ':' && *s != '-') {
                return false;
            }
            ++s;
        }
    }
    return *s == '\0';
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

bool loadImage(const char* path, std::vector<uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        return false;
    }
    const std::streamoff size = f.tellg();
    if (size <= 0) {
        return false;
    }
    out.resize(static_cast<size_t>(size));
    f.seekg(0);
    return static_cast<bool>(f.read(reinterpret_cast<char*>(out.data()), size));
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

    bool open(const char* ifname)
    {
        const size_t len = std::strlen(ifname);
        if (len == 0 || len >= IFNAMSIZ) {
            std::fprintf(stderr, "bad interface name '%s'\n", ifname);
            return false;
        }
        fd_ = socket(AF_PACKET, SOCK_RAW, htons(CELL_NET_L2_ETHERTYPE));
        if (fd_ < 0) {
            std::perror("socket(AF_PACKET) (needs CAP_NET_RAW)");
            return false;
        }
        ifreq ifr{};
        std::memcpy(ifr.ifr_name, ifname, len + 1);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            std::perror("SIOCGIFINDEX");
            return false;
        }
        ifindex_ = ifr.ifr_ifindex;
        if (ioctl(fd_, SIOCGIFHWADDR, &ifr) < 0) {
            std::perror("SIOCGIFHWADDR");
            return false;
        }
        std::memcpy(mac_.b, ifr.ifr_hwaddr.sa_data, 6);

        sockaddr_ll sll{};
        sll.sll_family = AF_PACKET;
        sll.sll_protocol = htons(CELL_NET_L2_ETHERTYPE);
        sll.sll_ifindex = ifindex_;
        if (bind(fd_, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
            std::perror("bind");
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
            std::perror("sendto");
            return false;
        }
        std::fprintf(stderr, "sendto: transmit queue stayed full\n");
        return false;
    }

    /**
     * Wait until the absolute deadline (CLOCK_MONOTONIC, us) for the next valid
     * CellNet frame from any source. Returns false only at the deadline.
     */
    bool recvFrame(int64_t deadline_us, ParsedFrame& out)
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
                    return true;
                }
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                std::perror("recvfrom");
                std::exit(kExitFail);
            }
            const int64_t now = nowUs();
            if (now >= deadline_us) {
                return false;
            }
            const int64_t rem_us = deadline_us - now;
            const int timeout_ms = static_cast<int>((rem_us + 999) / 1000);
            pollfd p{};
            p.fd = fd_;
            p.events = POLLIN;
            if (poll(&p, 1, timeout_ms) < 0 && errno != EINTR) {
                std::perror("poll");
                std::exit(kExitFail);
            }
        }
    }

private:
    int fd_ = -1;
    int ifindex_ = 0;
    Mac mac_{};
    uint8_t rx_[2048] = {};
};

struct Stats {
    unsigned frames_sent = 0;      // every frame this tool sent
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
    OtaSender(Link& link, const Mac& node_mac, uint16_t session,
              const std::vector<uint8_t>& image, uint16_t frag)
        : link_(link), node_mac_(node_mac), ota_mac_(node_mac), session_(session),
          image_(image), image_size_(static_cast<uint32_t>(image.size())), frag_(frag)
    {
        ota_mac_.b[0] = static_cast<uint8_t>(ota_mac_.b[0] | 0x02);
        blocks_ = static_cast<uint16_t>((image_size_ + Ota::kBlockSize - 1) / Ota::kBlockSize);
        t_start_ = nowUs();
    }

    int run(bool skip_reboot);

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

    int waitOtaMode(bool skip_reboot);
    int begin();
    int runBlocks();
    int finish();
    int waitNewImage();
    void printSummary() const;

    Link& link_;
    Mac node_mac_;
    Mac ota_mac_;
    uint16_t session_;
    const std::vector<uint8_t>& image_;
    uint32_t image_size_;
    uint16_t frag_;
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
    bool saw_erasing_ = false;
    bool got_done_ = false;
    bool have_result_ = false;
    L2OtaResultPayload result_{};

    int64_t t_start_ = 0;
    int64_t t_reboot_ = -1;
    int64_t t_heartbeat_ = -1;
    int64_t t_begin_ = -1;
    int64_t t_ready_ = -1;
    int64_t t_last_received_ = -1;
    int64_t t_end_ = -1;
    int64_t t_result_ = -1;

    Stats stats_;
};

bool OtaSender::sendFrame(const std::vector<uint8_t>& frame)
{
    if (!link_.send(frame)) {
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
        std::printf("FAIL: node replied ERR_SESSION (node session %u, ours %u)\n",
                    static_cast<unsigned>(st.session), static_cast<unsigned>(session_));
        return -1;
    }
    if (st.session != session_) {
        ++stats_.stale_status;
        return 0;
    }
    if (st.status >= code(Ota::Status::ERR_SIZE)) {
        std::printf("FAIL: node replied %s (code %u) for block %u\n", statusName(st.status),
                    static_cast<unsigned>(st.status), static_cast<unsigned>(st.block));
        return -1;
    }
    return 1;
}

/** Steps 1 and 2: REBOOT_TO_OTA, then the first OTA_MODE heartbeat. */
int OtaSender::waitOtaMode(bool skip_reboot)
{
    const int64_t t0 = nowUs();
    const int64_t deadline = t0 + kHeartbeatWaitUs;
    int64_t next_send = t0;
    int sends = 0;
    std::printf("waiting up to %.0f s for an OTA_MODE heartbeat from %s\n",
                toMs(kHeartbeatWaitUs) / 1000.0, macStr(ota_mac_).c_str());
    for (;;) {
        const int64_t now = nowUs();
        if (!skip_reboot && sends < kRebootMaxSends && now >= next_send) {
            if (!sendReboot()) {
                return kExitFail;
            }
            if (sends == 0) {
                t_reboot_ = now;
            }
            ++sends;
            next_send += kRebootResendUs;
            std::printf("REBOOT_TO_OTA session %u sent to %s (%d/%d)\n",
                        static_cast<unsigned>(session_), macStr(node_mac_).c_str(), sends,
                        kRebootMaxSends);
        }
        if (now >= deadline) {
            std::printf("FAIL: no OTA_MODE heartbeat from %s within %.0f s\n",
                        macStr(ota_mac_).c_str(), toMs(kHeartbeatWaitUs) / 1000.0);
            return kExitFail;
        }
        int64_t wake = deadline;
        if (!skip_reboot && sends < kRebootMaxSends && next_send < wake) {
            wake = next_send;
        }
        ParsedFrame pf;
        if (!link_.recvFrame(wake, pf)) {
            continue;
        }
        const auto type = static_cast<CellMsgType>(pf.cell.msg_type);
        if (type == CellMsgType::HEARTBEAT && fromMac(pf, ota_mac_)) {
            const uint16_t flags = pf.payload.heartbeat.status_flags;
            if ((flags & kHbOtaMode) != 0) {
                t_heartbeat_ = nowUs();
                std::printf("OTA_MODE heartbeat: node uptime %u ms, flags 0x%04x",
                            static_cast<unsigned>(pf.payload.heartbeat.uptime_ms),
                            static_cast<unsigned>(flags));
                if (t_reboot_ >= 0) {
                    std::printf(", %.1f ms after the first REBOOT_TO_OTA",
                                toMs(t_heartbeat_ - t_reboot_));
                }
                std::printf("\n");
                return kExitOk;
            }
        } else if (type == CellMsgType::GANTRY_STATUS && fromMac(pf, node_mac_) &&
                   old_slot_ < 0) {
            old_slot_ = pf.payload.gantry_status.active_slot;
            std::printf("pre-update GANTRY_STATUS: active_slot %d\n", old_slot_);
        }
    }
}

/** Step 3: OTA_BEGIN until READY. */
int OtaSender::begin()
{
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
            std::printf("FAIL: no READY within %.1f s (%d OTA_BEGIN sent)\n",
                        toMs(deadline - t0) / 1000.0, sends);
            return kExitFail;
        }
        ParsedFrame pf;
        if (!link_.recvFrame(next_send < deadline ? next_send : deadline, pf)) {
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
            std::printf("READY after %.1f ms (%d OTA_BEGIN sent, erasing seen in heartbeat: %s)\n",
                        toMs(t_ready_ - t_begin_), sends, saw_erasing_ ? "yes" : "no");
            return kExitOk;
        }
    }
}

/**
 * Step 4: send block acked_ until RECEIVED, then the next one, so block n+1
 * waits in the node's receive buffers while it writes block n.
 */
int OtaSender::runBlocks()
{
    if (!flushPending()) {
        return kExitFail;
    }
    while (acked_ < blocks_) {
        ParsedFrame pf;
        if (!link_.recvFrame(status_deadline_, pf)) {
            // 100 ms without a useful status.
            ++stats_.timeouts;
            if (++timeouts_ >= kMaxTimeouts) {
                std::printf("FAIL: %d consecutive %.0f ms timeouts on block %u "
                            "(outstanding %u)\n",
                            timeouts_, toMs(kStatusTimeoutUs), static_cast<unsigned>(acked_),
                            outstanding());
                return kExitFail;
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
                    std::printf("block %u/%u received, %.1f ms since READY\n",
                                static_cast<unsigned>(acked_), static_cast<unsigned>(blocks_),
                                toMs(t_last_received_ - t_ready_));
                }
                if (acked_ < blocks_) {
                    queueBlock(acked_);
                } else {
                    pending_.clear();
                }
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
                    std::printf("FAIL: %d MISSING rounds on block %u\n", missing_rounds_,
                                static_cast<unsigned>(acked_));
                    return kExitFail;
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
            std::printf("FAIL: OTA_END answered with MISSING %d times\n", kMaxEndRounds);
            return kExitFail;
        }
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
                    std::printf("OTA_STATUS DONE but no OTA_RESULT: emac_missed unknown\n");
                } else {
                    std::printf("no OTA_RESULT within %.0f s: outcome unknown, not failed\n",
                                toMs(kEndWaitUs) / 1000.0);
                }
                return kExitUnknown;
            }
            ParsedFrame pf;
            if (!link_.recvFrame(next_send < deadline ? next_send : deadline, pf)) {
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
                    std::printf("OTA_STATUS DONE after %.1f ms; waiting for OTA_RESULT\n",
                                toMs(t_result_ - t_end_));
                    if (now + kResultGraceUs < deadline) {
                        deadline = now + kResultGraceUs;
                    }
                }
            } else if (st.status == code(Ota::Status::MISSING) && st.block < blocks_) {
                ++stats_.missing_rx;
                std::printf("OTA_END: node still needs block %u; sending from there\n",
                            static_cast<unsigned>(st.block));
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
    const int64_t t0 = nowUs();
    const int64_t ref = t_result_ >= 0 ? t_result_ : t0;
    const int64_t deadline = t0 + kNewImageWaitUs;
    ParsedFrame pf;
    while (link_.recvFrame(deadline, pf)) {
        if (static_cast<CellMsgType>(pf.cell.msg_type) != CellMsgType::GANTRY_STATUS ||
            !fromMac(pf, node_mac_)) {
            continue;
        }
        const int slot = pf.payload.gantry_status.active_slot;
        std::printf("new image: GANTRY_STATUS from %s, active_slot %d, %.1f ms after %s\n",
                    macStr(node_mac_).c_str(), slot, toMs(nowUs() - ref),
                    t_result_ >= 0 ? "OTA_RESULT" : "the end of step 5");
        if (old_slot_ >= 0 && slot == old_slot_) {
            std::printf("FAIL: active_slot unchanged (%d): not booted into the new image\n",
                        slot);
            return kExitNotConfirmed;
        }
        if (old_slot_ < 0) {
            std::printf("(pre-update slot not seen, so the slot change is not checked)\n");
        }
        return kExitOk;
    }
    std::printf("FAIL: no GANTRY_STATUS from %s within %.0f s: reboot not confirmed\n",
                macStr(node_mac_).c_str(), toMs(kNewImageWaitUs) / 1000.0);
    return kExitNotConfirmed;
}

void OtaSender::printSummary() const
{
    std::printf("--- sender timings ---\n");
    if (t_reboot_ >= 0 && t_heartbeat_ >= 0) {
        std::printf("reboot -> OTA heartbeat : %9.1f ms (heartbeat every 250 ms)\n",
                    toMs(t_heartbeat_ - t_reboot_));
    }
    if (t_begin_ >= 0 && t_ready_ >= 0) {
        std::printf("BEGIN -> READY          : %9.1f ms\n", toMs(t_ready_ - t_begin_));
    }
    if (t_ready_ >= 0 && t_last_received_ >= 0) {
        const int64_t d = t_last_received_ - t_ready_;
        std::printf("READY -> last RECEIVED  : %9.1f ms (%u of %u blocks, %.0f kB/s)\n",
                    toMs(d), static_cast<unsigned>(acked_), static_cast<unsigned>(blocks_),
                    d > 0 ? static_cast<double>(image_size_) / 1024.0 / (toMs(d) / 1000.0)
                          : 0.0);
    }
    if (t_end_ >= 0 && t_result_ >= 0) {
        std::printf("END -> %-17s: %9.1f ms\n", have_result_ ? "RESULT" : "DONE",
                    toMs(t_result_ - t_end_));
    }
    if (t_result_ >= 0) {
        const int64_t t_first = t_reboot_ >= 0 ? t_reboot_ : t_begin_;
        std::printf("total                   : %9.1f ms (from the first %s)\n",
                    toMs(t_result_ - t_first), t_reboot_ >= 0 ? "REBOOT_TO_OTA" : "OTA_BEGIN");
    }
    std::printf("--- sender counts ---\n");
    std::printf("frames sent %u; OTA_DATA %u (first %u, resends %u, probes %u)\n",
                stats_.frames_sent, stats_.data_frames, stats_.first_sends, stats_.resends,
                stats_.probes);
    std::printf("MISSING received %u; timeouts %u; window stalls %u; max outstanding %u of %u\n",
                stats_.missing_rx, stats_.timeouts, stats_.window_stalls,
                stats_.max_outstanding, kWindow);
    std::printf("stale statuses %u; out-of-range processed_seq %u\n", stats_.stale_status,
                stats_.bad_processed);
}

int OtaSender::run(bool skip_reboot)
{
    const uint16_t last = static_cast<uint16_t>(blocks_ - 1);
    std::printf("interface MAC %s, node %s, OTA address %s\n", macStr(link_.mac()).c_str(),
                macStr(node_mac_).c_str(), macStr(ota_mac_).c_str());
    std::printf("image %u bytes, %u blocks (last %u bytes), frag %u -> %u per block, "
                "frame %u bytes, session %u\n",
                static_cast<unsigned>(image_size_), static_cast<unsigned>(blocks_),
                static_cast<unsigned>(blockLen(last)), static_cast<unsigned>(frag_),
                static_cast<unsigned>(fragCount(0)),
                static_cast<unsigned>(sizeof(L2OtaDataFrameHeader) + frag_),
                static_cast<unsigned>(session_));

    int rc = waitOtaMode(skip_reboot);
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
    if (rc == kExitFail) {
        printSummary();
        std::printf("RESULT: FAIL (aborted)\n");
        return kExitFail;
    }

    int exit_code = rc;  // kExitOk with a result, or kExitUnknown
    if (have_result_) {
        std::printf("--- OTA_RESULT ---\n");
        std::printf("status %s (%u); erase %u ms, stream %u ms, write %u ms, check %u ms; "
                    "emac_missed %u, emac_overflow %u\n",
                    statusName(result_.status), static_cast<unsigned>(result_.status),
                    static_cast<unsigned>(result_.erase_ms),
                    static_cast<unsigned>(result_.stream_ms),
                    static_cast<unsigned>(result_.write_ms),
                    static_cast<unsigned>(result_.check_ms),
                    static_cast<unsigned>(result_.emac_missed),
                    static_cast<unsigned>(result_.emac_overflow));
        if (result_.status != code(Ota::Status::DONE)) {
            std::printf("FAIL: OTA_RESULT status is %s, not DONE\n", statusName(result_.status));
            exit_code = kExitFail;
        } else if (result_.emac_missed != 0) {
            std::printf("FAIL: emac_missed = %u: frames were dropped for want of a receive "
                        "buffer (section 5.1 requires 0)\n",
                        static_cast<unsigned>(result_.emac_missed));
            exit_code = kExitEmacMissed;
        }
    }
    printSummary();
    if (exit_code == kExitFail) {
        std::printf("RESULT: FAIL\n");
        return exit_code;
    }

    const int rc6 = waitNewImage();
    if (exit_code == kExitOk && rc6 != kExitOk) {
        exit_code = rc6;
    }
    std::printf("RESULT: %s (exit %d)\n", exit_code == kExitOk ? "OK" : "FAIL", exit_code);
    return exit_code;
}

void usage(const char* argv0)
{
    std::fprintf(stderr,
                 "usage: %s <ifname> <node-mac> <image.bin> <session> [--frag N] "
                 "[--skip-reboot]\n"
                 "  --frag N       data bytes per OTA_DATA fragment (default %u, max %u)\n"
                 "  --skip-reboot  node already in OTA mode; do not send REBOOT_TO_OTA\n",
                 argv0, static_cast<unsigned>(kDefaultFrag),
                 static_cast<unsigned>(Ota::kMaxFragData));
}

bool parseU16(const char* s, uint16_t& out)
{
    char* end = nullptr;
    errno = 0;
    const unsigned long v = std::strtoul(s, &end, 0);
    if (errno != 0 || end == s || *end != '\0' || v > 0xFFFFul) {
        return false;
    }
    out = static_cast<uint16_t>(v);
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    std::vector<const char*> pos;
    uint16_t frag = kDefaultFrag;
    bool skip_reboot = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--skip-reboot") == 0) {
            skip_reboot = true;
        } else if (std::strcmp(argv[i], "--frag") == 0 && i + 1 < argc) {
            if (!parseU16(argv[++i], frag)) {
                std::fprintf(stderr, "bad --frag value '%s'\n", argv[i]);
                return kExitSetup;
            }
        } else if (argv[i][0] == '-' && argv[i][1] == '-') {
            usage(argv[0]);
            return kExitSetup;
        } else {
            pos.push_back(argv[i]);
        }
    }
    if (pos.size() != 4) {
        usage(argv[0]);
        return kExitSetup;
    }

    Mac node_mac;
    if (!parseMac(pos[1], node_mac)) {
        std::fprintf(stderr, "bad node MAC '%s' (want aa:bb:cc:dd:ee:ff)\n", pos[1]);
        return kExitSetup;
    }
    uint16_t session = 0;
    if (!parseU16(pos[3], session)) {
        std::fprintf(stderr, "bad session '%s' (0..65535)\n", pos[3]);
        return kExitSetup;
    }
    const uint32_t frags_per_block =
        frag == 0 ? 0 : (Ota::kBlockSize + frag - 1u) / static_cast<uint32_t>(frag);
    if (frag == 0 || frag > Ota::kMaxFragData || frags_per_block > Ota::kMaxFrags) {
        std::fprintf(stderr, "--frag must be %u..%u (at most %u fragments per block)\n",
                     static_cast<unsigned>((Ota::kBlockSize + Ota::kMaxFrags - 1) /
                                           Ota::kMaxFrags),
                     static_cast<unsigned>(Ota::kMaxFragData),
                     static_cast<unsigned>(Ota::kMaxFrags));
        return kExitSetup;
    }

    std::vector<uint8_t> image;
    if (!loadImage(pos[2], image)) {
        std::fprintf(stderr, "cannot read image '%s' (missing or empty)\n", pos[2]);
        return kExitSetup;
    }
    if (image.size() > static_cast<size_t>(0xFFFFu) * Ota::kBlockSize) {
        std::fprintf(stderr, "image too large (%zu bytes)\n", image.size());
        return kExitSetup;
    }
    if (image[0] != 0xE9) {
        // ESP-IDF app images start with magic byte 0xE9; esp_ota_end would reject it.
        std::fprintf(stderr, "warning: '%s' does not start with 0xE9; not an ESP app image?\n",
                     pos[2]);
    }

    Link link;
    if (!link.open(pos[0])) {
        return kExitSetup;
    }
    OtaSender sender(link, node_mac, session, image, frag);
    return sender.run(skip_reboot);
}
