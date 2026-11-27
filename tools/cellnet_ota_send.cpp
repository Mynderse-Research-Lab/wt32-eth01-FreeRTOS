/**
 * @file cellnet_ota_send.cpp
 * @brief Test sender for a WT32 firmware update over CellNet L2 (N-2).
 *
 * Test tool for N-2: it drives one complete CellNet OTA update from a Linux
 * host (the Raspberry Pi 5) so the node side can be proven end to end. The
 * protocol is lib/CellNetL2/CELLNET_OTA_PROTOCOL.md (sections 3 to 7, variant B, window
 * of section 5.1). The sender itself is CellNet::Ota::runOtaSender
 * (lib/CellNetL2/src/ota/CellNetOtaSender.cpp, plus the phase and report
 * sources beside it), shared with the supervisor's
 * /api/ota; this file is a thin CLI over it.
 *
 * Build on the Pi, from the repository root:
 *   g++ -O2 -std=c++17 -Wall -Wextra -I lib/CellNetL2/src -I lib/CellNetL2/src/ota \
 *       -I lib/CellNetL2/src/transport -I include \
 *       tools/cellnet_ota_send.cpp \
 *       lib/CellNetL2/src/ota/CellNetOtaSender.cpp \
 *       lib/CellNetL2/src/ota/CellNetOtaSenderPhases.cpp \
 *       lib/CellNetL2/src/ota/CellNetOtaSenderReport.cpp \
 *       lib/CellNetL2/src/CellNetL2Framing.cpp -o cellnet_ota_send
 *
 * Usage (raw socket, needs CAP_NET_RAW: run as root, or
 * `sudo setcap cap_net_raw+ep ./cellnet_ota_send`):
 *   cellnet_ota_send <ifname> <node-mac> <image.bin> <session> [--frag 456] [--skip-reboot]
 *
 *   ifname        interface on the cell switch, e.g. eth0
 *   node-mac      the node's normal MAC, aa:bb:cc:dd:ee:ff, or '-' to learn it
 *                 from the first GANTRY_STATUS (sender 0x03) within 3 s
 *   image.bin     the application image (idf/build/<app>.bin)
 *   session       OTA session, 0..65535 (decimal or 0x hex); use a counter
 *   --frag N      data bytes per OTA_DATA fragment (default 456: 486-byte frames)
 *   --skip-reboot the node is already in OTA mode; do not send REBOOT_TO_OTA
 *
 * Steps (CellNetOtaSender.h):
 *   0. Wait up to 3 s for one GANTRY_STATUS from the node: its MAC when '-',
 *      and the pre-update active_slot.
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
 *   1 usage or setup error (no node heard with '-'); 2 protocol failure or
 *   abort; 3 no OTA_RESULT (outcome unknown); 4 emac_missed != 0; 5 reboot
 *   into the new image not confirmed (no GANTRY_STATUS, or the slot did not
 *   change).
 */

#include "CellNetOtaReceiver.h"
#include "CellNetOtaSender.h"
#include "cell_net_l2_protocol.h"

#include <errno.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace Ota = CellNet::Ota;

constexpr uint16_t kDefaultFrag = 456;             // variant B, 486-byte frames

enum ExitCode : int {
    kExitOk = 0,
    kExitSetup = 1,
    kExitFail = 2,
};

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

bool parseMac(const char* s, uint8_t out[6])
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
        out[i] = static_cast<uint8_t>((hi << 4) | lo);
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

void printSummary(const Ota::SenderResult& r, const Ota::SenderProgress& last, bool rebooted,
                  size_t image_size)
{
    std::printf("--- sender timings ---\n");
    if (r.reboot_to_heartbeat_ms > 0) {
        std::printf("reboot -> OTA heartbeat : %9.1f ms (heartbeat every 250 ms)\n",
                    r.reboot_to_heartbeat_ms);
    }
    if (r.begin_to_ready_ms > 0) {
        std::printf("BEGIN -> READY          : %9.1f ms\n", r.begin_to_ready_ms);
    }
    if (r.ready_to_last_ms > 0) {
        std::printf("READY -> last RECEIVED  : %9.1f ms (%u of %u blocks, %.0f kB/s)\n",
                    r.ready_to_last_ms, static_cast<unsigned>(last.blocks_done),
                    static_cast<unsigned>(last.blocks_total),
                    static_cast<double>(image_size) / 1024.0 / (r.ready_to_last_ms / 1000.0));
    }
    if (r.end_to_result_ms > 0) {
        std::printf("END -> %-17s: %9.1f ms\n", r.have_result ? "RESULT" : "DONE",
                    r.end_to_result_ms);
    }
    if (r.total_ms > 0) {
        std::printf("total                   : %9.1f ms (from the first %s)\n", r.total_ms,
                    rebooted ? "REBOOT_TO_OTA" : "OTA_BEGIN");
    }
    std::printf("--- sender counts ---\n");
    std::printf("frames sent %u; OTA_DATA %u (first %u, resends %u, probes %u)\n",
                static_cast<unsigned>(r.frames_sent), static_cast<unsigned>(r.data_frames),
                static_cast<unsigned>(r.data_frames - r.resends - r.probes),
                static_cast<unsigned>(r.resends), static_cast<unsigned>(r.probes));
    // Window stalls, max outstanding and stale counts: the 'sender detail' log line.
    std::printf("MISSING received %u; timeouts %u\n", static_cast<unsigned>(r.missing_received),
                static_cast<unsigned>(r.timeouts));
}

void usage(const char* argv0)
{
    std::fprintf(stderr,
                 "usage: %s <ifname> <node-mac|-> <image.bin> <session> [--frag N] "
                 "[--skip-reboot]\n"
                 "  node-mac '-'   learn it from the first GANTRY_STATUS within 3 s\n"
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

    Ota::SenderOptions opt;
    opt.ifname = pos[0];
    if (std::strcmp(pos[1], "-") == 0) {
        opt.node_mac_known = false;
    } else if (parseMac(pos[1], opt.node_mac)) {
        opt.node_mac_known = true;
    } else {
        std::fprintf(stderr, "bad node MAC '%s' (want aa:bb:cc:dd:ee:ff, or '-')\n", pos[1]);
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
    opt.session = session;
    opt.frag_data = frag;
    opt.skip_reboot = skip_reboot;

    Ota::SenderProgress last{};
    const Ota::ProgressFn progress = [&last](const Ota::SenderProgress& p) { last = p; };
    const Ota::LogFn log = [](const std::string& line) { std::printf("%s\n", line.c_str()); };
    const Ota::SenderResult r = Ota::runOtaSender(opt, image, progress, log);

    if (r.code == kExitSetup) {
        std::fprintf(stderr, "%s\n", r.message.c_str());
        return kExitSetup;
    }
    if (r.have_result) {
        std::printf("--- OTA_RESULT ---\n");
        std::printf("status %s (%u); erase %u ms, stream %u ms, write %u ms, check %u ms; "
                    "emac_missed %u, emac_overflow %u\n",
                    statusName(r.result.status), static_cast<unsigned>(r.result.status),
                    static_cast<unsigned>(r.result.erase_ms),
                    static_cast<unsigned>(r.result.stream_ms),
                    static_cast<unsigned>(r.result.write_ms),
                    static_cast<unsigned>(r.result.check_ms),
                    static_cast<unsigned>(r.result.emac_missed),
                    static_cast<unsigned>(r.result.emac_overflow));
    }
    printSummary(r, last, !skip_reboot, image.size());
    if (r.code == kExitFail) {
        std::printf("RESULT: FAIL%s\n", r.have_result ? "" : " (aborted)");
        return kExitFail;
    }
    std::printf("RESULT: %s (exit %d)\n", r.code == kExitOk ? "OK" : "FAIL", r.code);
    return r.code;
}
