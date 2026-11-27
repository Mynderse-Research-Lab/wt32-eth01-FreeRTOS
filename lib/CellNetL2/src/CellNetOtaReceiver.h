/**
 * @file CellNetOtaReceiver.h
 * @brief Pure-logic receiver state machine for OTA over CellNet (N-2).
 *
 * Implements lib/CellNetL2/CELLNET_OTA_PROTOCOL.md sections 4 to 6 without I/O: the
 * caller feeds parsed OTA frames in, sends the returned replies, and supplies
 * flash access through OtaFlash. Freestanding C++17, host-tested.
 */

#ifndef CELL_NET_OTA_RECEIVER_H
#define CELL_NET_OTA_RECEIVER_H

#include <cstddef>
#include <cstdint>

namespace CellNet {
namespace Ota {

constexpr uint16_t kBlockSize = 4096;
constexpr uint16_t kMaxFragData = 478;  // 508-byte frame cap - 30 header bytes
constexpr uint16_t kMaxFrags = 128;     // missing[] bitmap is 16 bytes
constexpr uint8_t kFlagAckReq = 0x01;
constexpr uint32_t kBeginDeadlineMs = 30000;  // boot to first valid OTA_BEGIN
constexpr uint32_t kProgressTimeoutMs = 2000; // no completed block or END

enum class Status : uint8_t {
    READY = 1,
    RECEIVED = 2,
    MISSING = 3,
    DONE = 4,
    ERR_SIZE = 5,
    ERR_SESSION = 6,
    ERR_BUSY = 7,
    ERR_FLASH = 8,
    ERR_VERIFY = 9,
    ERR_TIMEOUT = 10,
};

enum class State : uint8_t { WAIT_BEGIN, RECEIVING, DONE, FAILED };

/** One OTA_STATUS to send, when send is true. */
struct Reply {
    bool send = false;
    uint16_t session = 0;
    uint16_t block = 0;
    Status status = Status::READY;
    uint8_t processed_seq = 0;
    uint8_t missing[16] = {};
};

/** Flash access, provided by the firmware (esp_ota_*) or a test fake. */
class OtaFlash {
public:
    virtual ~OtaFlash() = default;
    /** Erase the image range up front (esp_ota_begin with the size). */
    virtual bool begin(uint32_t image_size) = 0;
    /** Write at an absolute offset into the erased range. */
    virtual bool write(uint32_t offset, const uint8_t* data, size_t len) = 0;
    /** Check the image and set the boot slot (esp_ota_end + set boot). */
    virtual bool finish() = 0;
    virtual void abort() = 0;
};

class OtaReceiver {
public:
    /** @param session from REBOOT_TO_OTA; @param now_ms boot time base. */
    OtaReceiver(OtaFlash& flash, uint16_t session, uint32_t now_ms);

    Reply onBegin(uint16_t session, uint16_t block_size, uint16_t frag_data,
                  uint32_t image_size, uint32_t now_ms);
    Reply onData(uint8_t seq, uint16_t session, uint16_t block, uint8_t frag, uint8_t flags,
                 const uint8_t* data, uint16_t len, uint32_t now_ms);
    Reply onEnd(uint16_t session, uint32_t image_size, uint32_t now_ms);

    /** True when a block is complete and its RECEIVED reply has been returned:
     *  send that reply first, then call commit() to write the block. */
    bool writePending() const { return write_pending_; }
    /** Write the completed block; returns an ERR_FLASH reply on failure. */
    Reply commit(uint32_t now_ms);

    /** Restart the 2 s progress timer. Call with a fresh time once READY is sent:
     *  onBegin() gets its time before the erase, which takes seconds. */
    void restartProgressTimer(uint32_t now_ms) { last_progress_ms_ = now_ms; }

    /** Deadline passed (30 s to OTA_BEGIN, or 2 s without progress). The caller
     *  then aborts and reboots into the normal image. */
    bool expired(uint32_t now_ms) const;

    State state() const { return state_; }
    uint16_t nextBlock() const { return next_block_; }
    uint16_t blockCount() const { return blocks_; }

    /** Fragments in block b, and bytes in fragment f of block b. */
    uint16_t fragCount(uint16_t b) const;
    uint16_t fragLen(uint16_t b, uint8_t f) const;

private:
    Reply reply(uint16_t block, Status status) const;
    Reply missingReply() const;
    uint32_t blockLen(uint16_t b) const;
    void fail();

    OtaFlash& flash_;
    uint16_t session_;
    uint32_t t0_ms_;
    State state_ = State::WAIT_BEGIN;
    uint16_t frag_data_ = 0;
    uint32_t image_size_ = 0;
    uint16_t blocks_ = 0;
    uint16_t next_block_ = 0;
    uint8_t have_[16] = {};
    uint16_t have_count_ = 0;
    bool write_pending_ = false;
    uint8_t processed_seq_ = 0;
    uint32_t last_progress_ms_ = 0;
    uint8_t buf_[kBlockSize] = {};
};

}  // namespace Ota
}  // namespace CellNet

#endif  // CELL_NET_OTA_RECEIVER_H
