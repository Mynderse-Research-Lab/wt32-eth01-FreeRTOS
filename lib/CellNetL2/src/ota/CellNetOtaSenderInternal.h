/**
 * @file CellNetOtaSenderInternal.h
 * @brief Shared types for the Linux CellNet OTA sender.
 *
 * Link, the statistics, and the sender class. The method bodies live in
 * CellNetOtaSender.cpp, CellNetOtaSenderPhases.cpp and
 * CellNetOtaSenderReport.cpp. Linux only.
 */

#pragma once

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

namespace cellnet_ota_sender {

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

struct Mac {
    uint8_t b[6] = {};
};

std::string strf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
int64_t nowUs();
double toMs(int64_t us);
const char* statusName(uint8_t s);
constexpr uint8_t code(Ota::Status s)
{
    return static_cast<uint8_t>(s);
}
std::string macStr(const Mac& m);
bool fromMac(const ParsedFrame& pf, const Mac& m);

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

}  // namespace cellnet_ota_sender

#endif  // __linux__
