/**
 * @file CellNetOtaSender.h
 * @brief Linux sender for a WT32 firmware update over CellNet L2 (N-2).
 *
 * Drives one complete CellNet OTA update from a Linux host (the Raspberry Pi 5
 * supervisor) over an AF_PACKET raw socket. The protocol is
 * lib/CellNetL2/CELLNET_OTA_PROTOCOL.md (sections 3 to 7, variant B, window of section
 * 5.1), as proven on hardware by tools/cellnet_ota_send.cpp, which is now a
 * thin CLI over runOtaSender().
 *
 * Linux only: off Linux this header declares nothing and the sender
 * sources compile to nothing. Not part of the ESP component.
 *
 * Steps:
 *   0. Wait up to 3 s for one GANTRY_STATUS (sender_id 0x03) from the node:
 *      learns node_mac when it is not known (the first gantry heard wins) and
 *      records the pre-update active_slot. Skipped with skip_reboot and a
 *      known MAC (a node in OTA mode sends no GANTRY_STATUS).
 *   1. CELL_COMMAND 7 REBOOT_TO_OTA (param_u16 = session) to the node MAC,
 *      every 1 s, at most 5 times.
 *   2. Wait up to 30 s for a HEARTBEAT with OTA_MODE (bit 15) from the OTA MAC
 *      (node MAC with the first octet | 0x02). Everything after goes there.
 *   3. OTA_BEGIN, every 1 s, until READY (at most 2 s + 3.5 us per image byte).
 *   4. Blocks, one block ahead of the node's write, W = 18 unprocessed frames.
 *   5. OTA_END, every 2 s, until OTA_RESULT (at most 10 s).
 *   6. Wait up to 30 s for GANTRY_STATUS from the normal MAC: the new image.
 */

#ifndef CELL_NET_OTA_SENDER_H
#define CELL_NET_OTA_SENDER_H

#if defined(__linux__)

#include "cell_net_l2_protocol.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace CellNet {
namespace Ota {

struct SenderOptions {
    std::string ifname;            // e.g. "eth0"
    uint8_t node_mac[6] = {};      // the node's normal MAC
    bool node_mac_known = false;   // false: learn it from the first GANTRY_STATUS (sender_id 0x03) within 3 s
    uint16_t session = 1;
    uint16_t frag_data = 456;      // max 478
    bool skip_reboot = false;      // node already in OTA mode
};

struct SenderProgress {
    const char* phase;             // "discover","reboot","begin","blocks","end","confirm"
    uint32_t blocks_done = 0, blocks_total = 0;
    uint32_t bytes_done = 0, bytes_total = 0;
    double kbps = 0.0;             // KiB/s over the block phase so far
};

struct SenderResult {
    bool ok = false;               // DONE, emac_missed == 0, and new image seen
    int code = 1;                  // 0 ok, 1 setup, 2 protocol, 3 no result, 4 emac_missed != 0, 5 reboot not confirmed
    std::string message;           // one-line human summary
    bool have_result = false;
    L2OtaResultPayload result{};   // OTA_RESULT fields when have_result
    int old_slot = -1, new_slot = -1;  // GANTRY_STATUS.active_slot before and after (-1 unknown)
    uint8_t node_mac[6] = {};
    double reboot_to_heartbeat_ms = 0, begin_to_ready_ms = 0, ready_to_last_ms = 0,
           end_to_result_ms = 0, total_ms = 0;
    uint32_t frames_sent = 0, data_frames = 0, resends = 0, probes = 0,
             missing_received = 0, timeouts = 0;
};

using ProgressFn = std::function<void(const SenderProgress&)>;
using LogFn = std::function<void(const std::string&)>;

/**
 * Runs the whole update and blocks until done. Needs CAP_NET_RAW. progress/log
 * may be empty. Both are called on the caller's thread and must return quickly:
 * progress on entry to each phase and, in the block phase, at most every
 * 100 ms plus once when the phase ends; log once per line, without a newline.
 *
 * Timings are 0 when not measured. total_ms runs from the first REBOOT_TO_OTA
 * (or OTA_BEGIN with skip_reboot) to OTA_RESULT, or to OTA_STATUS DONE when no
 * OTA_RESULT came. Step 6 still runs for codes 3 and 4 (the code stays).
 */
SenderResult runOtaSender(const SenderOptions& opt, const std::vector<uint8_t>& image,
                          const ProgressFn& progress, const LogFn& log);

}  // namespace Ota
}  // namespace CellNet

#endif  // __linux__

#endif  // CELL_NET_OTA_SENDER_H
