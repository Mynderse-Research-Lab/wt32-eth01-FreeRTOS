/**
 * @file CellNetOtaSenderPhases.cpp
 * @brief Discover, reboot, begin, block transfer, end and confirm for the CellNet OTA sender.
 */

#if defined(__linux__)

#include "CellNetOtaSenderInternal.h"

#include "CellNetOtaReceiver.h"

namespace cellnet_ota_sender {

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

}  // namespace cellnet_ota_sender

#endif  // __linux__
