/**
 * @file test_cellnet_ota_receiver.cpp
 * @brief Host tests for the CellNet OTA receiver state machine (N-2), including
 *        the loss, duplicate and reorder sequences from the 2026-11-13 reviews
 *        of lib/CellNetL2/CELLNET_OTA_PROTOCOL.md.
 */

#include "CellNetOtaReceiver.h"
#include "unity.h"

#include <cstring>
#include <vector>

using namespace CellNet::Ota;

void setUp(void) {}
void tearDown(void) {}

namespace {

constexpr uint16_t kSession = 0x1234;
constexpr uint16_t kFrag = 456;  // variant B

class FakeFlash : public OtaFlash {
public:
    std::vector<uint8_t> mem;
    int begins = 0, writes = 0, finishes = 0, aborts = 0;
    bool fail_begin = false, fail_write = false, fail_finish = false;
    uint32_t slot = 1920 * 1024;

    bool begin(uint32_t size) override {
        ++begins;
        if (fail_begin || size > slot) {
            return false;
        }
        mem.assign(size, 0xFF);
        return true;
    }
    bool write(uint32_t offset, const uint8_t* data, size_t len) override {
        ++writes;
        if (fail_write || offset + len > mem.size()) {
            return false;
        }
        std::memcpy(mem.data() + offset, data, len);
        return true;
    }
    bool finish() override {
        ++finishes;
        return !fail_finish;
    }
    void abort() override { ++aborts; }
};

std::vector<uint8_t> makeImage(uint32_t size) {
    std::vector<uint8_t> img(size);
    for (uint32_t i = 0; i < size; ++i) {
        img[i] = static_cast<uint8_t>((i * 131u) ^ (i >> 8));
    }
    return img;
}

struct Sender {
    OtaReceiver& rx;
    const std::vector<uint8_t>& img;
    uint8_t seq = 0;
    uint32_t now = 1000;

    Reply frag(uint16_t b, uint8_t f, uint8_t flags = 0, uint16_t session = kSession) {
        const uint32_t off = static_cast<uint32_t>(b) * kBlockSize + static_cast<uint32_t>(f) * kFrag;
        return rx.onData(seq++, session, b, f, flags, img.data() + off, rx.fragLen(b, f), now);
    }
    // Send a whole block, ACK_REQ on the last frame; commit if it completed.
    Reply block(uint16_t b) {
        Reply r;
        const uint16_t n = rx.fragCount(b);
        for (uint16_t f = 0; f < n; ++f) {
            r = frag(b, static_cast<uint8_t>(f), f + 1u == n ? kFlagAckReq : 0);
        }
        if (rx.writePending()) {
            rx.commit(now);
        }
        return r;
    }
};

bool missingBit(const Reply& r, uint16_t i) {
    return (r.missing[i / 8] & (1u << (i % 8))) != 0;
}

}  // namespace

void test_geometry_for_gantry_image(void) {
    FakeFlash fl;
    OtaReceiver rx(fl, kSession, 0);
    TEST_ASSERT_EQUAL(Status::READY, rx.onBegin(kSession, kBlockSize, kFrag, 843360, 0).status);
    TEST_ASSERT_EQUAL_UINT16(206, rx.blockCount());
    TEST_ASSERT_EQUAL_UINT16(9, rx.fragCount(0));
    TEST_ASSERT_EQUAL_UINT16(448, rx.fragLen(0, 8));
    TEST_ASSERT_EQUAL_UINT16(9, rx.fragCount(205));  // last block holds 3,680 bytes
    TEST_ASSERT_EQUAL_UINT16(32, rx.fragLen(205, 8));
    TEST_ASSERT_EQUAL_UINT16(0, rx.fragLen(0, 9));
}

void test_happy_path_writes_image_once(void) {
    FakeFlash fl;
    const auto img = makeImage(3 * kBlockSize + 1000);
    OtaReceiver rx(fl, kSession, 0);
    TEST_ASSERT_EQUAL(Status::READY,
                      rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0).status);
    Sender s{rx, img};
    for (uint16_t b = 0; b < rx.blockCount(); ++b) {
        TEST_ASSERT_EQUAL(Status::RECEIVED, s.block(b).status);
    }
    TEST_ASSERT_EQUAL(Status::DONE, rx.onEnd(kSession, img.size(), s.now).status);
    TEST_ASSERT_EQUAL(State::DONE, rx.state());
    TEST_ASSERT_EQUAL_INT(1, fl.begins);
    TEST_ASSERT_EQUAL_INT(4, fl.writes);
    TEST_ASSERT_EQUAL_INT(1, fl.finishes);
    TEST_ASSERT_EQUAL_MEMORY(img.data(), fl.mem.data(), img.size());
}

void test_begin_validation(void) {
    FakeFlash fl;
    OtaReceiver a(fl, kSession, 0);
    TEST_ASSERT_EQUAL(Status::ERR_SIZE, a.onBegin(kSession, 2048, kFrag, 10000, 0).status);
    TEST_ASSERT_EQUAL(State::FAILED, a.state());
    OtaReceiver b(fl, kSession, 0);
    TEST_ASSERT_EQUAL(Status::ERR_SIZE, b.onBegin(kSession, kBlockSize, 479, 10000, 0).status);
    OtaReceiver c(fl, kSession, 0);  // 31-byte fragments would need 133 per block
    TEST_ASSERT_EQUAL(Status::ERR_SIZE, c.onBegin(kSession, kBlockSize, 31, 10000, 0).status);
    OtaReceiver d(fl, kSession, 0);
    TEST_ASSERT_EQUAL(Status::ERR_SIZE, d.onBegin(kSession, kBlockSize, kFrag, 0, 0).status);
    TEST_ASSERT_EQUAL_INT(0, fl.begins);  // nothing erased for a bad request
    OtaReceiver e(fl, kSession, 0);  // larger than the slot: flash refuses
    TEST_ASSERT_EQUAL(Status::ERR_FLASH,
                      e.onBegin(kSession, kBlockSize, kFrag, 2 * 1024 * 1024, 0).status);
    TEST_ASSERT_EQUAL(State::FAILED, e.state());
}

void test_session_mismatch_rejected_everywhere(void) {
    FakeFlash fl;
    const auto img = makeImage(kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    TEST_ASSERT_EQUAL(Status::ERR_SESSION, rx.onBegin(0x9999, kBlockSize, kFrag, img.size(), 0).status);
    TEST_ASSERT_EQUAL(State::WAIT_BEGIN, rx.state());
    TEST_ASSERT_EQUAL_INT(0, fl.begins);  // a stale OTA_BEGIN cannot capture the node
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    TEST_ASSERT_EQUAL(Status::ERR_SESSION, s.frag(0, 0, 0, 0x9999).status);
    TEST_ASSERT_EQUAL(Status::ERR_SESSION, rx.onEnd(0x9999, img.size(), 0).status);
    TEST_ASSERT_EQUAL(State::RECEIVING, rx.state());
}

void test_repeated_begin_never_erases_again(void) {  // READY lost
    FakeFlash fl;
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, 50000, 0);
    TEST_ASSERT_EQUAL(Status::READY, rx.onBegin(kSession, kBlockSize, kFrag, 50000, 500).status);
    TEST_ASSERT_EQUAL(Status::READY, rx.onBegin(kSession, kBlockSize, kFrag, 50000, 900).status);
    TEST_ASSERT_EQUAL_INT(1, fl.begins);
    TEST_ASSERT_EQUAL(Status::ERR_SIZE, rx.onBegin(kSession, kBlockSize, kFrag, 60000, 900).status);
}

void test_lost_fragment_reported_and_resent(void) {
    FakeFlash fl;
    const auto img = makeImage(2 * kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    Reply r;
    for (uint8_t f = 0; f < 9; ++f) {
        if (f != 3) {
            r = s.frag(0, f, f == 8 ? kFlagAckReq : 0);
        }
    }
    TEST_ASSERT_EQUAL(Status::MISSING, r.status);
    TEST_ASSERT_EQUAL_UINT16(0, r.block);
    TEST_ASSERT_TRUE(missingBit(r, 3));
    TEST_ASSERT_FALSE(missingBit(r, 2));
    TEST_ASSERT_FALSE(missingBit(r, 8));
    TEST_ASSERT_EQUAL(Status::RECEIVED, s.frag(0, 3, kFlagAckReq).status);
    TEST_ASSERT_TRUE(rx.writePending());
}

void test_lost_missing_does_not_livelock(void) {
    FakeFlash fl;
    const auto img = makeImage(kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    for (uint8_t f = 0; f < 9; ++f) {
        if (f != 5) {
            s.frag(0, f, f == 8 ? kFlagAckReq : 0);  // this MISSING is "lost"
        }
    }
    // The sender times out and probes with a duplicate of the last frame.
    const Reply probe = s.frag(0, 8, kFlagAckReq);
    TEST_ASSERT_TRUE(probe.send);
    TEST_ASSERT_EQUAL(Status::MISSING, probe.status);
    TEST_ASSERT_TRUE(missingBit(probe, 5));
    // The resent fragment is lost again; another probe still gets an answer.
    TEST_ASSERT_TRUE(s.frag(0, 0, kFlagAckReq).send);
}

void test_lost_received_never_writes_twice(void) {
    FakeFlash fl;
    const auto img = makeImage(2 * kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    s.block(0);  // RECEIVED 0 "lost"; block 0 already written
    TEST_ASSERT_EQUAL_INT(1, fl.writes);
    for (uint8_t f = 0; f < 9; ++f) {  // the sender resends all of block 0
        const Reply r = s.frag(0, f, kFlagAckReq);
        TEST_ASSERT_EQUAL(Status::RECEIVED, r.status);
        TEST_ASSERT_EQUAL_UINT16(0, r.block);
    }
    TEST_ASSERT_FALSE(rx.writePending());
    TEST_ASSERT_EQUAL_INT(1, fl.writes);
    TEST_ASSERT_EQUAL(Status::RECEIVED, s.block(1).status);
    TEST_ASSERT_EQUAL(Status::DONE, rx.onEnd(kSession, img.size(), s.now).status);
    TEST_ASSERT_EQUAL_MEMORY(img.data(), fl.mem.data(), img.size());
}

void test_duplicates_while_write_pending(void) {
    FakeFlash fl;
    const auto img = makeImage(2 * kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    for (uint8_t f = 0; f < 9; ++f) {
        s.frag(0, f);
    }
    TEST_ASSERT_TRUE(rx.writePending());
    TEST_ASSERT_EQUAL(Status::RECEIVED, s.frag(0, 4, kFlagAckReq).status);
    TEST_ASSERT_EQUAL_INT(0, fl.writes);
    rx.commit(s.now);
    TEST_ASSERT_EQUAL_INT(1, fl.writes);
    TEST_ASSERT_EQUAL_UINT16(1, rx.nextBlock());
}

void test_reordered_fragments(void) {
    FakeFlash fl;
    const auto img = makeImage(kBlockSize + 700);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    for (int f = 8; f >= 0; --f) {
        s.frag(0, static_cast<uint8_t>(f));
    }
    TEST_ASSERT_TRUE(rx.writePending());
    rx.commit(s.now);
    s.frag(1, 1);  // the 700-byte block is 2 fragments, out of order
    TEST_ASSERT_EQUAL(Status::RECEIVED, s.frag(1, 0).status);
    rx.commit(s.now);
    TEST_ASSERT_EQUAL(Status::DONE, rx.onEnd(kSession, img.size(), s.now).status);
    TEST_ASSERT_EQUAL_MEMORY(img.data(), fl.mem.data(), img.size());
}

void test_later_block_and_bad_length_dropped(void) {
    FakeFlash fl;
    const auto img = makeImage(3 * kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    TEST_ASSERT_FALSE(s.frag(1, 0).send);  // block 1 before block 0: dropped
    const Reply r = s.frag(2, 0, kFlagAckReq);
    TEST_ASSERT_EQUAL(Status::MISSING, r.status);
    TEST_ASSERT_EQUAL_UINT16(0, r.block);  // status for the block still expected
    const Reply bad = rx.onData(s.seq++, kSession, 0, 0, kFlagAckReq, img.data(), 100, s.now);
    TEST_ASSERT_EQUAL(Status::MISSING, bad.status);
    TEST_ASSERT_TRUE(missingBit(bad, 0));
    TEST_ASSERT_EQUAL_UINT16(0, rx.nextBlock());
}

void test_end_rules(void) {
    FakeFlash fl;
    const auto img = makeImage(2 * kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    s.block(0);
    const Reply early = rx.onEnd(kSession, img.size(), s.now);
    TEST_ASSERT_EQUAL(Status::MISSING, early.status);
    TEST_ASSERT_EQUAL_UINT16(1, early.block);
    TEST_ASSERT_EQUAL_INT(0, fl.finishes);
    for (uint8_t f = 0; f < 9; ++f) {
        s.frag(1, f);
    }
    TEST_ASSERT_EQUAL(Status::MISSING, rx.onEnd(kSession, img.size(), s.now).status);  // not written yet
    rx.commit(s.now);
    TEST_ASSERT_EQUAL(Status::DONE, rx.onEnd(kSession, img.size(), s.now).status);
    TEST_ASSERT_EQUAL(Status::DONE, rx.onEnd(kSession, img.size(), s.now).status);  // duplicate
    TEST_ASSERT_EQUAL_INT(1, fl.finishes);
}

void test_end_size_mismatch_aborts(void) {
    FakeFlash fl;
    const auto img = makeImage(kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    TEST_ASSERT_EQUAL(Status::ERR_SIZE, rx.onEnd(kSession, img.size() + 1, 0).status);
    TEST_ASSERT_EQUAL(State::FAILED, rx.state());
    TEST_ASSERT_EQUAL_INT(1, fl.aborts);
}

void test_flash_failures(void) {
    {
        FakeFlash fl;
        fl.fail_write = true;
        const auto img = makeImage(kBlockSize);
        OtaReceiver rx(fl, kSession, 0);
        rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
        Sender s{rx, img};
        for (uint8_t f = 0; f < 9; ++f) {
            s.frag(0, f);
        }
        TEST_ASSERT_EQUAL(Status::ERR_FLASH, rx.commit(0).status);
        TEST_ASSERT_EQUAL(State::FAILED, rx.state());
        TEST_ASSERT_EQUAL_INT(1, fl.aborts);
    }
    {
        FakeFlash fl;
        fl.fail_finish = true;
        const auto img = makeImage(kBlockSize);
        OtaReceiver rx(fl, kSession, 0);
        rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
        Sender s{rx, img};
        s.block(0);
        TEST_ASSERT_EQUAL(Status::ERR_VERIFY, rx.onEnd(kSession, img.size(), 0).status);
        TEST_ASSERT_EQUAL(State::FAILED, rx.state());
    }
}

void test_deadlines(void) {
    FakeFlash fl;
    const auto img = makeImage(2 * kBlockSize);
    OtaReceiver rx(fl, kSession, 1000);
    TEST_ASSERT_FALSE(rx.expired(1000 + kBeginDeadlineMs));
    TEST_ASSERT_TRUE(rx.expired(1001 + kBeginDeadlineMs));
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 5000);
    Sender s{rx, img};
    s.now = 6500;
    s.frag(0, 0);  // frames alone are not progress
    s.frag(0, 0, kFlagAckReq);
    TEST_ASSERT_TRUE(rx.expired(5000 + kProgressTimeoutMs + 1));
    s.now = 6900;
    s.block(0);  // a completed, written block is
    TEST_ASSERT_FALSE(rx.expired(6900 + kProgressTimeoutMs));
    TEST_ASSERT_TRUE(rx.expired(6901 + kProgressTimeoutMs));
}

void test_processed_seq_echoed(void) {
    FakeFlash fl;
    const auto img = makeImage(kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 0);
    Sender s{rx, img};
    s.seq = 250;  // wraps through 255
    for (uint8_t f = 0; f < 7; ++f) {
        s.frag(0, f);
    }
    const Reply r = s.frag(0, 7, kFlagAckReq);  // seq 257 & 0xFF = 1
    TEST_ASSERT_EQUAL_UINT8(1, r.processed_seq);
    const Reply dup = rx.onData(9, kSession, 0, 0, kFlagAckReq, img.data(), 7, s.now);  // bad length
    TEST_ASSERT_EQUAL_UINT8(9, dup.processed_seq);  // counted even when dropped
}

void test_progress_timer_restarts_after_slow_erase(void) {
    // Firmware passes onBegin a time taken before the erase (2.8 s measured);
    // it must restart the timer once READY goes out (2026-11-13 review).
    FakeFlash fl;
    const auto img = makeImage(kBlockSize);
    OtaReceiver rx(fl, kSession, 0);
    TEST_ASSERT_EQUAL(Status::READY, rx.onBegin(kSession, kBlockSize, kFrag, img.size(), 1000).status);
    const uint32_t after_erase = 1000 + 3000;
    TEST_ASSERT_TRUE(rx.expired(after_erase));  // what the review found
    rx.restartProgressTimer(after_erase);
    TEST_ASSERT_FALSE(rx.expired(after_erase + kProgressTimeoutMs));
    TEST_ASSERT_TRUE(rx.expired(after_erase + kProgressTimeoutMs + 1));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_geometry_for_gantry_image);
    RUN_TEST(test_happy_path_writes_image_once);
    RUN_TEST(test_begin_validation);
    RUN_TEST(test_session_mismatch_rejected_everywhere);
    RUN_TEST(test_repeated_begin_never_erases_again);
    RUN_TEST(test_lost_fragment_reported_and_resent);
    RUN_TEST(test_lost_missing_does_not_livelock);
    RUN_TEST(test_lost_received_never_writes_twice);
    RUN_TEST(test_duplicates_while_write_pending);
    RUN_TEST(test_reordered_fragments);
    RUN_TEST(test_later_block_and_bad_length_dropped);
    RUN_TEST(test_end_rules);
    RUN_TEST(test_end_size_mismatch_aborts);
    RUN_TEST(test_flash_failures);
    RUN_TEST(test_deadlines);
    RUN_TEST(test_processed_seq_echoed);
    RUN_TEST(test_progress_timer_restarts_after_slow_erase);
    return UNITY_END();
}
