// Host-side unit tests for W5500 SPI frame construction.
//
// Tests the pure C++ logic from lib/W5500/src/W5500.h:
//   - Block select computation
//   - Control byte formation
//   - Register address constants
//   - Frame encoding (the byte sequence sent over SPI)
//
// This test runs WITHOUT ESP-IDF — it verifies the math that the
// on-target firmware will use when it talks to the W5500 over SPI.

#include "W5500.h"
#include "unity.h"

#include <cstring>

void setUp(void) {}
void tearDown(void) {}

// =============================================================================
// Block Select computation
// =============================================================================

static void test_block_common(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, kBlockCommon());
}

static void test_block_socket_reg(void) {
    TEST_ASSERT_EQUAL_UINT8(1, kBlockSocketReg(0));
    TEST_ASSERT_EQUAL_UINT8(5, kBlockSocketReg(1));
    TEST_ASSERT_EQUAL_UINT8(9, kBlockSocketReg(2));
    TEST_ASSERT_EQUAL_UINT8(29, kBlockSocketReg(7));
}

static void test_block_socket_buffers(void) {
    TEST_ASSERT_EQUAL_UINT8(2, kBlockSocketTxBuf(0));
    TEST_ASSERT_EQUAL_UINT8(3, kBlockSocketRxBuf(0));
    TEST_ASSERT_EQUAL_UINT8(6, kBlockSocketTxBuf(1));
    TEST_ASSERT_EQUAL_UINT8(7, kBlockSocketRxBuf(1));
}

// =============================================================================
// Control byte construction
// =============================================================================

static void test_ctrl_common_read(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, makeCtrlByte(kBlockCommon(), kRwRead));
}

static void test_ctrl_common_write(void) {
    TEST_ASSERT_EQUAL_UINT8(0x04, makeCtrlByte(kBlockCommon(), kRwWrite));
}

static void test_ctrl_socket0_reg_read(void) {
    TEST_ASSERT_EQUAL_UINT8(0x08, makeCtrlByte(kBlockSocketReg(0), kRwRead));
}

static void test_ctrl_socket0_reg_write(void) {
    TEST_ASSERT_EQUAL_UINT8(0x0C, makeCtrlByte(kBlockSocketReg(0), kRwWrite));
}

static void test_ctrl_socket0_txbuf_write(void) {
    TEST_ASSERT_EQUAL_UINT8(0x14, makeCtrlByte(kBlockSocketTxBuf(0), kRwWrite));
}

static void test_ctrl_socket0_rxbuf_read(void) {
    TEST_ASSERT_EQUAL_UINT8(0x18, makeCtrlByte(kBlockSocketRxBuf(0), kRwRead));
}

// =============================================================================
// SPI Frame header byte sequences
// =============================================================================

// Helper: build a 3-byte SPI header (address MSB, address LSB, control byte)
static void buildHeader(uint16_t addr, uint8_t ctrlByte, uint8_t out[3]) {
    out[0] = static_cast<uint8_t>(addr >> 8);
    out[1] = static_cast<uint8_t>(addr & 0xFF);
    out[2] = ctrlByte;
}

static void test_frame_read_versionr(void) {
    uint8_t hdr[3];
    buildHeader(REG_VERSIONR, makeCtrlByte(kBlockCommon(), kRwRead), hdr);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);  // addr MSB
    TEST_ASSERT_EQUAL_UINT8(0x39, hdr[1]);  // addr LSB
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[2]);  // ctrl: common read
}

static void test_frame_write_mr_rst(void) {
    uint8_t hdr[3];
    buildHeader(REG_MR, makeCtrlByte(kBlockCommon(), kRwWrite), hdr);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[1]);
    TEST_ASSERT_EQUAL_UINT8(0x04, hdr[2]);  // ctrl: common write
}

static void test_frame_open_socket0(void) {
    uint8_t hdr[3];
    buildHeader(Sn_CR, makeCtrlByte(kBlockSocketReg(0), kRwWrite), hdr);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
    TEST_ASSERT_EQUAL_UINT8(0x01, hdr[1]);  // Sn_CR offset
    TEST_ASSERT_EQUAL_UINT8(0x0C, hdr[2]);  // sock0 reg write
}

static void test_frame_write_txbuf(void) {
    uint8_t hdr[3];
    buildHeader(0x0000, makeCtrlByte(kBlockSocketTxBuf(0), kRwWrite), hdr);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, hdr[1]);
    TEST_ASSERT_EQUAL_UINT8(0x14, hdr[2]);  // sock0 TX buf write
}

// =============================================================================
// Register addresses
// =============================================================================

static void test_reg_common(void) {
    TEST_ASSERT_EQUAL_UINT16(0x0000, REG_MR);
    TEST_ASSERT_EQUAL_UINT16(0x0009, REG_SHAR);
    TEST_ASSERT_EQUAL_UINT16(0x000F, REG_SIPR);
    TEST_ASSERT_EQUAL_UINT16(0x0039, REG_VERSIONR);
}

static void test_reg_socket_offsets(void) {
    TEST_ASSERT_EQUAL_UINT16(0x0000, Sn_MR);
    TEST_ASSERT_EQUAL_UINT16(0x0001, Sn_CR);
    TEST_ASSERT_EQUAL_UINT16(0x0002, Sn_IR);
    TEST_ASSERT_EQUAL_UINT16(0x0003, Sn_SR);
    TEST_ASSERT_EQUAL_UINT16(0x0020, Sn_TX_FSR);
    TEST_ASSERT_EQUAL_UINT16(0x0022, Sn_TX_RD);
    TEST_ASSERT_EQUAL_UINT16(0x0024, Sn_TX_WR);
    TEST_ASSERT_EQUAL_UINT16(0x0026, Sn_RX_RSR);
    TEST_ASSERT_EQUAL_UINT16(0x0028, Sn_RX_RD);
}

static void test_reg_buf_size(void) {
    TEST_ASSERT_EQUAL_UINT16(0x001E, Sn_RXBUF_SIZE);
    TEST_ASSERT_EQUAL_UINT16(0x001F, Sn_TXBUF_SIZE);
}

// =============================================================================
// Socket mode / command / status constants
// =============================================================================

static void test_const_socket_mode(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, Sn_MR_CLOSE);
    TEST_ASSERT_EQUAL_UINT8(0x01, Sn_MR_TCP);
    TEST_ASSERT_EQUAL_UINT8(0x02, Sn_MR_UDP);
    TEST_ASSERT_EQUAL_UINT8(0x04, Sn_MR_MACRAW);
}

static void test_const_socket_cmds(void) {
    TEST_ASSERT_EQUAL_UINT8(0x01, Sn_CR_OPEN);
    TEST_ASSERT_EQUAL_UINT8(0x10, Sn_CR_CLOSE);
    TEST_ASSERT_EQUAL_UINT8(0x20, Sn_CR_SEND);
    TEST_ASSERT_EQUAL_UINT8(0x40, Sn_CR_RECV);
}

static void test_const_socket_status(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, SOCK_CLOSED);
    TEST_ASSERT_EQUAL_UINT8(0x42, SOCK_MACRAW);
}

static void test_const_interrupt(void) {
    TEST_ASSERT_EQUAL_UINT8(0x10, Sn_IR_SENDOK);
    TEST_ASSERT_EQUAL_UINT8(0x08, Sn_IR_TIMEOUT);
    TEST_ASSERT_EQUAL_UINT8(0x04, Sn_IR_RECV);
}

static void test_const_mr_bits(void) {
    TEST_ASSERT_EQUAL_UINT8(0x80, MR_RST);
    TEST_ASSERT_EQUAL_UINT8(0x20, MR_WOL);
    TEST_ASSERT_EQUAL_UINT8(0x10, MR_PB);
}

// =============================================================================
// Integration: full MACRAW send frame SPI byte sequence
// =============================================================================

static void test_integration_macraw_send(void) {
    // Simulate sending a 60-byte EtherCAT frame.
    // Steps:
    //   1. Write TX buffer: socket 0 TX buf, addr 0, N bytes
    //   2. Read Sn_TX_WR
    //   3. Write Sn_TX_WR + len
    //   4. Write Sn_CR SEND
    //   5. Read Sn_IR for SENDOK

    const uint16_t frameLen = 60;
    const uint16_t oldTxWr  = 0x100;
    const uint16_t newTxWr  = oldTxWr + frameLen;

    // Step 1: Write TX buffer
    {
        uint8_t hdr[3];
        buildHeader(0x0000, makeCtrlByte(kBlockSocketTxBuf(0), kRwWrite), hdr);
        TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
        TEST_ASSERT_EQUAL_UINT8(0x00, hdr[1]);
        TEST_ASSERT_EQUAL_UINT8(0x14, hdr[2]);
    }

    // Step 2: Read Sn_TX_WR
    {
        uint8_t hdr[3];
        buildHeader(Sn_TX_WR, makeCtrlByte(kBlockSocketReg(0), kRwRead), hdr);
        TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
        TEST_ASSERT_EQUAL_UINT8(0x24, hdr[1]);
        TEST_ASSERT_EQUAL_UINT8(0x08, hdr[2]);
    }

    // Step 3: Write Sn_TX_WR = newTxWr
    {
        uint8_t hdr[3];
        buildHeader(Sn_TX_WR, makeCtrlByte(kBlockSocketReg(0), kRwWrite), hdr);
        TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
        TEST_ASSERT_EQUAL_UINT8(0x24, hdr[1]);
        TEST_ASSERT_EQUAL_UINT8(0x0C, hdr[2]);
    }

    // Step 4: Write Sn_CR = SEND
    {
        uint8_t hdr[3];
        buildHeader(Sn_CR, makeCtrlByte(kBlockSocketReg(0), kRwWrite), hdr);
        TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
        TEST_ASSERT_EQUAL_UINT8(0x01, hdr[1]);
        TEST_ASSERT_EQUAL_UINT8(0x0C, hdr[2]);
    }

    // Step 5: Read Sn_IR
    {
        uint8_t hdr[3];
        buildHeader(Sn_IR, makeCtrlByte(kBlockSocketReg(0), kRwRead), hdr);
        TEST_ASSERT_EQUAL_UINT8(0x00, hdr[0]);
        TEST_ASSERT_EQUAL_UINT8(0x02, hdr[1]);
        TEST_ASSERT_EQUAL_UINT8(0x08, hdr[2]);
    }

    // Math check
    TEST_ASSERT_EQUAL_UINT16(0x013C, newTxWr);
}

// =============================================================================
// Test runner
// =============================================================================

int main(void) {
    UNITY_BEGIN();

    // Block select
    RUN_TEST(test_block_common);
    RUN_TEST(test_block_socket_reg);
    RUN_TEST(test_block_socket_buffers);

    // Control byte
    RUN_TEST(test_ctrl_common_read);
    RUN_TEST(test_ctrl_common_write);
    RUN_TEST(test_ctrl_socket0_reg_read);
    RUN_TEST(test_ctrl_socket0_reg_write);
    RUN_TEST(test_ctrl_socket0_txbuf_write);
    RUN_TEST(test_ctrl_socket0_rxbuf_read);

    // SPI frame headers
    RUN_TEST(test_frame_read_versionr);
    RUN_TEST(test_frame_write_mr_rst);
    RUN_TEST(test_frame_open_socket0);
    RUN_TEST(test_frame_write_txbuf);

    // Register addresses
    RUN_TEST(test_reg_common);
    RUN_TEST(test_reg_socket_offsets);
    RUN_TEST(test_reg_buf_size);

    // Constants
    RUN_TEST(test_const_socket_mode);
    RUN_TEST(test_const_socket_cmds);
    RUN_TEST(test_const_socket_status);
    RUN_TEST(test_const_interrupt);
    RUN_TEST(test_const_mr_bits);

    // Integration
    RUN_TEST(test_integration_macraw_send);

    return UNITY_END();
}
