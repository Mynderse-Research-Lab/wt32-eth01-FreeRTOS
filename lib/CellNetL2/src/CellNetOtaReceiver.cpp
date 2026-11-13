/**
 * @file CellNetOtaReceiver.cpp
 * @brief Pure-logic receiver state machine for OTA over CellNet (N-2).
 */

#include "CellNetOtaReceiver.h"

#include <cstring>

namespace CellNet {
namespace Ota {

namespace {

bool bitSet(const uint8_t* bits, uint16_t i) {
    return (bits[i / 8] & (1u << (i % 8))) != 0;
}

void setBit(uint8_t* bits, uint16_t i) {
    bits[i / 8] = static_cast<uint8_t>(bits[i / 8] | (1u << (i % 8)));
}

}  // namespace

OtaReceiver::OtaReceiver(OtaFlash& flash, uint16_t session, uint32_t now_ms)
    : flash_(flash), session_(session), t0_ms_(now_ms) {}

uint32_t OtaReceiver::blockLen(uint16_t b) const {
    if (b >= blocks_) {
        return 0;
    }
    const uint32_t start = static_cast<uint32_t>(b) * kBlockSize;
    const uint32_t rest = image_size_ - start;
    return rest < kBlockSize ? rest : kBlockSize;
}

uint16_t OtaReceiver::fragCount(uint16_t b) const {
    const uint32_t len = blockLen(b);
    return frag_data_ == 0 ? 0 : static_cast<uint16_t>((len + frag_data_ - 1) / frag_data_);
}

uint16_t OtaReceiver::fragLen(uint16_t b, uint8_t f) const {
    const uint32_t len = blockLen(b);
    const uint32_t start = static_cast<uint32_t>(f) * frag_data_;
    if (f >= fragCount(b) || start >= len) {
        return 0;
    }
    const uint32_t rest = len - start;
    return static_cast<uint16_t>(rest < frag_data_ ? rest : frag_data_);
}

Reply OtaReceiver::reply(uint16_t block, Status status) const {
    Reply r;
    r.send = true;
    r.session = session_;
    r.block = block;
    r.status = status;
    r.processed_seq = processed_seq_;
    return r;
}

Reply OtaReceiver::missingReply() const {
    Reply r = reply(next_block_, Status::MISSING);
    const uint16_t n = fragCount(next_block_);
    for (uint16_t i = 0; i < n; ++i) {
        if (!bitSet(have_, i)) {
            setBit(r.missing, i);
        }
    }
    return r;
}

void OtaReceiver::fail() {
    if (state_ == State::RECEIVING) {
        flash_.abort();
    }
    state_ = State::FAILED;
    write_pending_ = false;
}

Reply OtaReceiver::onBegin(uint16_t session, uint16_t block_size, uint16_t frag_data,
                           uint32_t image_size, uint32_t now_ms) {
    if (session != session_) {
        return reply(0, Status::ERR_SESSION);
    }
    if (state_ == State::RECEIVING || state_ == State::DONE) {
        // Idempotent: a repeat never erases again. Same parameters get READY.
        if (frag_data == frag_data_ && image_size == image_size_ && block_size == kBlockSize) {
            return reply(0, Status::READY);
        }
        return reply(0, Status::ERR_SIZE);
    }
    if (state_ == State::FAILED) {
        return reply(0, Status::ERR_SIZE);
    }
    const uint32_t blocks = (image_size + kBlockSize - 1) / kBlockSize;
    const uint32_t frags = frag_data == 0 ? 0 : (kBlockSize + frag_data - 1) / frag_data;
    if (block_size != kBlockSize || frag_data == 0 || frag_data > kMaxFragData ||
        frags > kMaxFrags || image_size == 0 || blocks > 0xFFFFu) {
        fail();
        return reply(0, Status::ERR_SIZE);
    }
    frag_data_ = frag_data;
    image_size_ = image_size;
    blocks_ = static_cast<uint16_t>(blocks);
    if (!flash_.begin(image_size)) {  // also rejects a size over the slot
        state_ = State::FAILED;
        return reply(0, Status::ERR_FLASH);
    }
    state_ = State::RECEIVING;
    next_block_ = 0;
    have_count_ = 0;
    std::memset(have_, 0, sizeof(have_));
    last_progress_ms_ = now_ms;  // the 2 s progress timer starts at READY
    return reply(0, Status::READY);
}

Reply OtaReceiver::onData(uint8_t seq, uint16_t session, uint16_t block, uint8_t frag,
                          uint8_t flags, const uint8_t* data, uint16_t len, uint32_t now_ms) {
    (void)now_ms;  // frames alone are not progress
    if (session != session_) {
        return reply(block, Status::ERR_SESSION);
    }
    if (state_ != State::RECEIVING) {
        return Reply{};
    }
    processed_seq_ = seq;  // taken out of the receive buffers, valid or not
    const bool ack_req = (flags & kFlagAckReq) != 0;

    if (block < next_block_ || (block == next_block_ && write_pending_)) {
        return reply(block, Status::RECEIVED);  // never stored or written again
    }
    if (block > next_block_ || block >= blocks_) {
        return ack_req ? missingReply() : Reply{};
    }
    if (data == nullptr || len == 0 || len != fragLen(block, frag)) {
        return ack_req ? missingReply() : Reply{};
    }
    if (!bitSet(have_, frag)) {
        std::memcpy(buf_ + static_cast<uint32_t>(frag) * frag_data_, data, len);
        setBit(have_, frag);
        ++have_count_;
    }
    if (have_count_ == fragCount(block)) {
        write_pending_ = true;  // caller sends RECEIVED, then commit()
        return reply(block, Status::RECEIVED);
    }
    const bool last = frag + 1u == fragCount(block);
    return (ack_req || last) ? missingReply() : Reply{};
}

Reply OtaReceiver::commit(uint32_t now_ms) {
    if (!write_pending_ || state_ != State::RECEIVING) {
        return Reply{};
    }
    const uint32_t offset = static_cast<uint32_t>(next_block_) * kBlockSize;
    if (!flash_.write(offset, buf_, blockLen(next_block_))) {
        const uint16_t b = next_block_;
        fail();
        return reply(b, Status::ERR_FLASH);
    }
    write_pending_ = false;
    ++next_block_;
    have_count_ = 0;
    std::memset(have_, 0, sizeof(have_));
    last_progress_ms_ = now_ms;
    return Reply{};
}

Reply OtaReceiver::onEnd(uint16_t session, uint32_t image_size, uint32_t now_ms) {
    if (session != session_) {
        return reply(0, Status::ERR_SESSION);
    }
    if (state_ == State::DONE) {
        return reply(blocks_, Status::DONE);  // cached: never finish() twice
    }
    if (state_ != State::RECEIVING) {
        return Reply{};
    }
    if (image_size != image_size_) {
        fail();
        return reply(0, Status::ERR_SIZE);
    }
    if (next_block_ < blocks_ || write_pending_) {
        return missingReply();  // not every block is written yet
    }
    last_progress_ms_ = now_ms;
    if (!flash_.finish()) {
        state_ = State::FAILED;
        return reply(blocks_, Status::ERR_VERIFY);
    }
    state_ = State::DONE;
    return reply(blocks_, Status::DONE);
}

bool OtaReceiver::expired(uint32_t now_ms) const {
    switch (state_) {
        case State::WAIT_BEGIN:
            return now_ms - t0_ms_ > kBeginDeadlineMs;
        case State::RECEIVING:
            return now_ms - last_progress_ms_ > kProgressTimeoutMs;
        default:
            return false;
    }
}

}  // namespace Ota
}  // namespace CellNet
