/**
 * @file gantry_cellnet_ota.cpp
 * @brief OTA over CellNet L2 (N-2): entry from normal mode, and the OTA-only
 *        mode the node reboots into. Protocol: lib/CellNetL2/CELLNET_OTA_PROTOCOL.md.
 *
 * The receiver logic is CellNet::Ota::OtaReceiver (host-tested); this file is
 * the ESP32 wiring around it: the noinit flag across esp_restart, the private
 * OTA address and EMAC filters (section 5.1), a receive queue, the heartbeat,
 * esp_ota_* flash access and the reboot at the end.
 */

#include "gantry_ota.h"

#include "CellNetL2Framing.h"
#include "CellNetOtaReceiver.h"
#include "EspEthL2Transport.h"
#include "EthernetLink.h"
#include "cell_net_l2_protocol.h"

#include "esp_attr.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/emac_ll.h"
#include "soc/emac_dma_struct.h"
#include "soc/emac_mac_struct.h"

#include <atomic>
#include <cstring>
#include <new>
#include <vector>

static const char *TAG = "CellNetOTA";

namespace {

using CellNet::CellNetL2Framing;
using CellNet::Ota::OtaReceiver;
using CellNet::Ota::Reply;
using CellNet::Ota::State;
using CellNet::Ota::Status;

// Survives esp_restart (ESP-IDF memory-types: "noinit" DRAM). Random at
// power-on, so it is accepted only with the magic, the complement and a
// software reset.
constexpr uint32_t kFlagMagic = 0x4F544143u;  // "CATO"
struct OtaModeFlag {
    uint32_t magic;
    uint16_t session;
    uint16_t session_inv;
};
__NOINIT_ATTR OtaModeFlag s_flag;

constexpr uint16_t kHbOtaMode = 0x8000;
constexpr uint16_t kHbErasing = 0x4000;
constexpr size_t kRxQueueDepth = 24;  // more than the sender's 18-frame window
constexpr uint32_t kHeartbeatMs = 250;
constexpr uint32_t kReadyResendMs = 250;
constexpr uint32_t kRestartDelayMs = 300;

struct RxItem {
    uint16_t len;
    uint8_t data[CellNetL2Framing::kMaxOtaDataFrameLen];
};

QueueHandle_t s_rxq = nullptr;
CellNet::EspEthL2Transport *s_tr = nullptr;
uint8_t s_ota_mac[6] = {};
uint8_t s_sup_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // learned at OTA_BEGIN
std::atomic<bool> s_erasing{false};
std::atomic<uint32_t> s_rxq_drops{0};
uint8_t s_tx_seq = 0;            // guarded by s_tx_mux
portMUX_TYPE s_tx_mux = portMUX_INITIALIZER_UNLOCKED;  // s_tx_seq, s_sup_mac
uint32_t s_mode_start_ms = 0;     // entry into OTA mode, for the 30 s deadline
constexpr uint32_t kHardCapMs = 120000;  // OTA mode never outlives this

uint32_t nowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// Next sequence number and the current destination, read together.
uint8_t nextTx(uint8_t dst[6]) {
    portENTER_CRITICAL(&s_tx_mux);
    const uint8_t seq = s_tx_seq++;
    std::memcpy(dst, s_sup_mac, 6);
    portEXIT_CRITICAL(&s_tx_mux);
    return seq;
}

uint16_t sat16(int64_t ms) {
    return ms < 0 ? 0 : (ms > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(ms));
}

void sendPadded(std::vector<uint8_t> frame) {
    if (frame.size() < CellNetL2Framing::kMinEthFrameLen) {
        frame.resize(CellNetL2Framing::kMinEthFrameLen, 0);
    }
    if (s_tr != nullptr) {
        (void)s_tr->sendFrame(frame.data(), frame.size());
    }
}

void sendStatus(const Reply &r) {
    if (!r.send) {
        return;
    }
    L2OtaStatusPayload p{};
    p.session = r.session;
    p.block = r.block;
    p.status = static_cast<uint8_t>(r.status);
    p.processed_seq = r.processed_seq;
    std::memcpy(p.missing, r.missing, sizeof(p.missing));
    uint8_t dst[6];
    const uint8_t seq = nextTx(dst);
    sendPadded(CellNetL2Framing::buildOtaStatusFrame(s_ota_mac, CellNodeId::GANTRY, seq,
                                                     static_cast<uint32_t>(esp_timer_get_time()),
                                                     p, dst));
}

class EspOtaFlash : public CellNet::Ota::OtaFlash {
public:
    bool begin(uint32_t image_size) override {
        return gantryOtaStartStream(nullptr, handle_, part_, image_size) == ESP_OK;
    }
    bool write(uint32_t offset, const uint8_t *data, size_t len) override {
        return esp_ota_write_with_offset(handle_, data, len, offset) == ESP_OK;
    }
    bool finish() override {
        const bool ok = gantryOtaFinishStream(handle_, part_) == ESP_OK;
        handle_ = 0;
        return ok;
    }
    void abort() override {
        if (handle_ != 0) {
            (void)esp_ota_abort(handle_);
            handle_ = 0;
        }
    }

private:
    esp_ota_handle_t handle_ = 0;
    const esp_partition_t *part_ = nullptr;
};

void heartbeatTask(void *) {
    for (;;) {
        L2HeartbeatPayload hb{};
        hb.uptime_ms = nowMs();
        hb.status_flags = static_cast<uint16_t>(kHbOtaMode | (s_erasing.load() ? kHbErasing : 0));
        uint8_t dst[6];
        const uint8_t seq = nextTx(dst);
        sendPadded(CellNetL2Framing::buildHeartbeatFrame(s_ota_mac, CellNodeId::GANTRY, seq,
                                                         static_cast<uint32_t>(esp_timer_get_time()),
                                                         hb, dst));
        // Backstop: whatever the receiver does, OTA mode ends in a reboot.
        if (nowMs() - s_mode_start_ms > kHardCapMs) {
            ESP_LOGE(TAG, "OTA mode hard cap reached: restarting");
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(kHeartbeatMs));
    }
}

// Receive path (emac_rx task): copy CellNet frames into the queue for the OTA
// task. The MAC filter already passes only frames addressed to the OTA address.
void onRxFrame(const uint8_t *buf, size_t len) {
    if (s_rxq == nullptr || len > CellNetL2Framing::kMaxOtaDataFrameLen) {
        return;
    }
    RxItem item;
    item.len = static_cast<uint16_t>(len);
    std::memcpy(item.data, buf, len);
    if (xQueueSend(s_rxq, &item, 0) != pdTRUE) {
        s_rxq_drops.fetch_add(1);
    }
}

[[noreturn]] void restartAfter(uint32_t ms, const char *why) {
    ESP_LOGW(TAG, "%s: restarting in %lu ms", why, static_cast<unsigned long>(ms));
    vTaskDelay(pdMS_TO_TICKS(ms));
    esp_restart();
    for (;;) {
    }
}

void otaTask(void *param) {
    const uint16_t session = static_cast<uint16_t>(reinterpret_cast<uintptr_t>(param));
    static EspOtaFlash flash;
    // The 30 s deadline counts from entry into OTA mode, not from this task.
    auto *rx = new OtaReceiver(flash, session, s_mode_start_ms);  // 4 KB buffer: heap DRAM

    uint32_t ready_ms = 0;
    bool got_data = false;
    uint32_t done_ms = 0;
    int64_t erase_us = 0;
    int64_t write_us = 0;
    int64_t check_us = 0;
    uint32_t stream_start_ms = 0;
    uint32_t stream_end_ms = 0;
    L2OtaResultPayload result{};

    RxItem item;
    for (;;) {
        if (xQueueReceive(s_rxq, &item, pdMS_TO_TICKS(20)) == pdTRUE) {
            CellNet::ParsedFrame f{};
            if (CellNetL2Framing::parseFrame(item.data, item.len, f) &&
                f.cell.sender_id == static_cast<uint8_t>(CellNodeId::SUPERVISOR)) {
                const auto type = static_cast<CellMsgType>(f.cell.msg_type);
                if (type == CellMsgType::OTA_BEGIN) {
                    portENTER_CRITICAL(&s_tx_mux);
                    std::memcpy(s_sup_mac, f.eth.src_mac, 6);
                    portEXIT_CRITICAL(&s_tx_mux);
                    const bool first = rx->state() == State::WAIT_BEGIN;
                    s_erasing = first;
                    const int64_t t = esp_timer_get_time();
                    const Reply r = rx->onBegin(f.payload.ota_begin.session,
                                                f.payload.ota_begin.block_size,
                                                f.payload.ota_begin.frag_data,
                                                f.payload.ota_begin.image_size, nowMs());
                    s_erasing = false;
                    if (first && r.status == Status::READY) {
                        erase_us = esp_timer_get_time() - t;
                        rx->restartProgressTimer(nowMs());  // the erase took seconds
                        (void)EMAC_DMA.dmamissedfr.val;  // read clears: count from READY
                        ready_ms = nowMs();
                        stream_start_ms = ready_ms;
                        ESP_LOGI(TAG, "READY: erased %u bytes in %lld ms",
                                 static_cast<unsigned>(f.payload.ota_begin.image_size),
                                 static_cast<long long>(erase_us / 1000));
                    }
                    sendStatus(r);
                } else if (type == CellMsgType::OTA_DATA) {
                    got_data = true;
                    sendStatus(rx->onData(f.cell.sequence, f.payload.ota_data.session,
                                          f.payload.ota_data.block, f.payload.ota_data.frag,
                                          f.payload.ota_data.flags, f.ota_data,
                                          f.payload.ota_data.data_len, nowMs()));
                    if (rx->writePending()) {
                        const int64_t t = esp_timer_get_time();
                        sendStatus(rx->commit(nowMs()));
                        write_us += esp_timer_get_time() - t;
                        stream_end_ms = nowMs();
                    }
                } else if (type == CellMsgType::OTA_END) {
                    const bool was_done = rx->state() == State::DONE;
                    const int64_t t = esp_timer_get_time();
                    const Reply r = rx->onEnd(f.payload.ota_end.session,
                                              f.payload.ota_end.image_size, nowMs());
                    if (r.status == Status::DONE) {
                        if (!was_done) {
                            check_us = esp_timer_get_time() - t;
                            const uint32_t reg = EMAC_DMA.dmamissedfr.val;
                            result.session = session;
                            result.status = static_cast<uint8_t>(Status::DONE);
                            result.erase_ms = sat16(erase_us / 1000);
                            result.stream_ms = sat16(static_cast<int64_t>(stream_end_ms) -
                                                     stream_start_ms);
                            result.write_ms = sat16(write_us / 1000);
                            result.check_ms = sat16(check_us / 1000);
                            result.emac_missed = static_cast<uint16_t>(reg & 0xFFFFu);
                            result.emac_overflow = static_cast<uint16_t>((reg >> 17) & 0x7FFu);
                            done_ms = nowMs();
                            ESP_LOGI(TAG,
                                     "DONE: erase %u ms, stream %u ms, write %u ms, check %u ms, "
                                     "emac missed %u, overflow %u, queue drops %lu",
                                     result.erase_ms, result.stream_ms, result.write_ms,
                                     result.check_ms, result.emac_missed, result.emac_overflow,
                                     static_cast<unsigned long>(s_rxq_drops.load()));
                        }
                        // Cached: every OTA_END after DONE gets the same result.
                        uint8_t dst[6];
                        const uint8_t seq = nextTx(dst);
                        sendPadded(CellNetL2Framing::buildOtaResultFrame(
                            s_ota_mac, CellNodeId::GANTRY, seq,
                            static_cast<uint32_t>(esp_timer_get_time()), result, dst));
                    } else {
                        sendStatus(r);
                    }
                }
            }
        }

        const uint32_t now = nowMs();
        if (rx->state() == State::RECEIVING && !got_data && now - ready_ms >= kReadyResendMs) {
            Reply ready;
            ready.send = true;
            ready.session = session;
            ready.status = Status::READY;
            sendStatus(ready);  // READY repeats until the first OTA_DATA
            ready_ms = now;
        }
        if (rx->state() == State::DONE && now - done_ms >= kRestartDelayMs) {
            restartAfter(0, "CellNet OTA complete");
        }
        if (rx->state() == State::FAILED) {
            restartAfter(kRestartDelayMs, "CellNet OTA failed");
        }
        if (rx->expired(now)) {
            const bool began = rx->state() == State::RECEIVING;
            if (began) {
                flash.abort();
            }
            Reply err;
            err.send = true;
            err.session = session;
            err.block = rx->nextBlock();
            err.status = Status::ERR_TIMEOUT;
            sendStatus(err);
            restartAfter(kRestartDelayMs, began ? "CellNet OTA stalled" : "no OTA_BEGIN in 30 s");
        }
    }
}

}  // namespace

esp_err_t gantryOtaRebootToCellNetMode(Gantry::Gantry *gantry, uint16_t session) {
    if (gantry != nullptr && gantry->isBusy()) {
        ESP_LOGW(TAG, "REBOOT_TO_OTA refused: gantry busy (move/home/pick)");
        return ESP_ERR_INVALID_STATE;
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (running == nullptr || (esp_ota_get_state_partition(running, &state) == ESP_OK &&
                               state != ESP_OTA_IMG_VALID && state != ESP_OTA_IMG_UNDEFINED)) {
        ESP_LOGW(TAG, "REBOOT_TO_OTA refused: running image not marked valid (state %d)",
                 static_cast<int>(state));
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t err = gantryOtaDisableMotors(gantry);
    if (err != ESP_OK) {
        return err;
    }
    s_flag.magic = kFlagMagic;
    s_flag.session = session;
    s_flag.session_inv = static_cast<uint16_t>(~session);
    ESP_LOGW(TAG, "REBOOT_TO_OTA session %u: motors off, restarting into OTA-only mode",
             static_cast<unsigned>(session));
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

bool gantryOtaCellNetModeRequested(uint16_t *session) {
    const bool valid = s_flag.magic == kFlagMagic &&
                       s_flag.session_inv == static_cast<uint16_t>(~s_flag.session) &&
                       esp_reset_reason() == ESP_RST_SW;
    const uint16_t s = s_flag.session;
    s_flag.magic = 0;  // cleared at once: a crash in OTA mode boots normally
    if (valid && session != nullptr) {
        *session = s;
    }
    return valid;
}

void gantryOtaRunCellNetMode(uint16_t session) {
    s_mode_start_ms = nowMs();
    ESP_LOGW(TAG, "=== CellNet OTA-only mode, session %u ===", static_cast<unsigned>(session));
    (void)Network::EthernetLink::enablePhyOscillator();
    static Network::EthernetLink eth;
    if (!eth.start() || !eth.waitForUp(10000)) {
        restartAfter(kRestartDelayMs, "Ethernet did not come up in OTA mode");
    }
    esp_eth_handle_t h = eth.getEthHandle();

    // Section 5.1 rule 1: receive only on a private address nothing else knows.
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_ETH));
    mac[0] = static_cast<uint8_t>(mac[0] | 0x02);  // locally administered
    std::memcpy(s_ota_mac, mac, 6);
    bool off = false;
    if (esp_eth_ioctl(h, ETH_CMD_S_MAC_ADDR, s_ota_mac) != ESP_OK ||
        esp_eth_ioctl(h, ETH_CMD_S_PROMISCUOUS, &off) != ESP_OK ||
        esp_eth_ioctl(h, ETH_CMD_S_ALL_MULTICAST, &off) != ESP_OK) {
        restartAfter(kRestartDelayMs, "could not set the OTA address and filters");
    }
    emac_ll_broadcast_frame_enable(&EMAC_MAC, false);

    s_rxq = xQueueCreate(kRxQueueDepth, sizeof(RxItem));
    static CellNet::EspEthL2Transport tr;
    s_tr = &tr;
    tr.setRxCallback(onRxFrame);
    if (s_rxq == nullptr || tr.attachEthHandle(h, eth.getNetif()) != ESP_OK) {
        restartAfter(kRestartDelayMs, "CellNet transport not ready in OTA mode");
    }
    ESP_LOGW(TAG, "OTA address %02x:%02x:%02x:%02x:%02x:%02x, broadcast off; waiting for "
                  "OTA_BEGIN",
             s_ota_mac[0], s_ota_mac[1], s_ota_mac[2], s_ota_mac[3], s_ota_mac[4], s_ota_mac[5]);

    if (xTaskCreatePinnedToCore(otaTask, "cellnet_ota", 8192,
                                reinterpret_cast<void *>(static_cast<uintptr_t>(session)), 5,
                                nullptr, 0) != pdPASS ||
        xTaskCreatePinnedToCore(heartbeatTask, "ota_hb", 3072, nullptr, 3, nullptr, 0) != pdPASS) {
        restartAfter(kRestartDelayMs, "OTA tasks not created");
    }
    vTaskDelete(nullptr);  // app_main ends here; nothing else starts in OTA mode
}
