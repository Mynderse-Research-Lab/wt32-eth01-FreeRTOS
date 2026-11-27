/**
 * @file test_cell_net_l2_ota.cpp
 * @brief CellNet L2 OTA framing host tests.
 */

#include "CellNetL2.h"
#include "CellNetL2Framing.h"
#include "IL2Transport.h"
#include "cell_net_l2_protocol.h"
#include "unity.h"

#include <cstring>
#include <deque>
#include <vector>

void test_framing_ota_sizes(void) {
  using F = CellNet::CellNetL2Framing;
  TEST_ASSERT_EQUAL_size_t(34, F::expectedFrameLength(CellMsgType::OTA_BEGIN));
  TEST_ASSERT_EQUAL_size_t(30, F::expectedFrameLength(CellMsgType::OTA_DATA));
  TEST_ASSERT_EQUAL_size_t(44, F::expectedFrameLength(CellMsgType::OTA_STATUS));
  TEST_ASSERT_EQUAL_size_t(30, F::expectedFrameLength(CellMsgType::OTA_END));
  TEST_ASSERT_EQUAL_size_t(38, F::expectedFrameLength(CellMsgType::OTA_RESULT));
}

void test_framing_ota_data_roundtrip_and_limits(void) {
  using F = CellNet::CellNetL2Framing;
  const uint8_t src[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
  const uint8_t dst[6] = {0xa8, 0x03, 0x2a, 0x22, 0xbf, 0x8f};
  uint8_t data[478];
  for (size_t i = 0; i < sizeof(data); ++i) {
    data[i] = static_cast<uint8_t>(i * 7u);
  }
  L2OtaDataHeader h{};
  h.session = 0xBEEF;
  h.block = 205;
  h.frag = 8;
  h.flags = 1;
  h.data_len = 456;
  auto bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 200, 5, h, data, dst);
  TEST_ASSERT_EQUAL_size_t(486, bytes.size());  // variant B frame
  CellNet::ParsedFrame p{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), p));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CellMsgType::OTA_DATA), p.cell.msg_type);
  TEST_ASSERT_EQUAL_UINT8(200, p.cell.sequence);
  TEST_ASSERT_EQUAL_HEX16(0xBEEF, p.payload.ota_data.session);
  TEST_ASSERT_EQUAL_UINT16(205, p.payload.ota_data.block);
  TEST_ASSERT_EQUAL_UINT8(8, p.payload.ota_data.frag);
  TEST_ASSERT_EQUAL_UINT8(1, p.payload.ota_data.flags);
  TEST_ASSERT_EQUAL_UINT16(456, p.payload.ota_data.data_len);
  TEST_ASSERT_NOT_NULL(p.ota_data);
  TEST_ASSERT_EQUAL_MEMORY(data, p.ota_data, 456);

  // A short last fragment is padded on the wire; data_len still governs.
  h.data_len = 4;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  bytes.resize(F::kMinEthFrameLen, 0);
  CellNet::ParsedFrame small{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), small));
  TEST_ASSERT_EQUAL_UINT16(4, small.payload.ota_data.data_len);

  // data_len larger than the frame carries: rejected.
  h.data_len = 40;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  bytes.resize(bytes.size() - 1);
  CellNet::ParsedFrame trunc{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), trunc));

  // data_len 0: rejected.
  h.data_len = 0;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  bytes.resize(F::kMinEthFrameLen, 0);
  CellNet::ParsedFrame empty{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), empty));

  // 508 bytes is the cap for OTA_DATA; 509 is rejected.
  h.data_len = 478;
  bytes = F::buildOtaDataFrame(src, CellNodeId::SUPERVISOR, 1, 5, h, data, dst);
  TEST_ASSERT_EQUAL_size_t(F::kMaxOtaDataFrameLen, bytes.size());
  CellNet::ParsedFrame cap{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), cap));
  bytes.push_back(0);
  CellNet::ParsedFrame over{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), over));
}

void test_framing_ota_fixed_roundtrips(void) {
  using F = CellNet::CellNetL2Framing;
  const uint8_t src[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};

  L2OtaBeginPayload b{};
  b.session = 7;
  b.block_size = 4096;
  b.frag_data = 456;
  b.image_size = 843360;
  auto bytes = F::buildOtaBeginFrame(src, CellNodeId::SUPERVISOR, 1, 2, b);
  CellNet::ParsedFrame pb{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), pb));
  TEST_ASSERT_EQUAL_UINT32(843360, pb.payload.ota_begin.image_size);
  TEST_ASSERT_EQUAL_UINT16(456, pb.payload.ota_begin.frag_data);

  L2OtaStatusPayload st{};
  st.session = 7;
  st.block = 12;
  st.status = 3;
  st.processed_seq = 250;
  st.missing[0] = 0x08;
  bytes = F::buildOtaStatusFrame(src, CellNodeId::GANTRY, 1, 2, st);
  CellNet::ParsedFrame ps{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), ps));
  TEST_ASSERT_EQUAL_UINT8(250, ps.payload.ota_status.processed_seq);
  TEST_ASSERT_EQUAL_HEX8(0x08, ps.payload.ota_status.missing[0]);

  L2OtaEndPayload e{};
  e.session = 7;
  e.image_size = 843360;
  bytes = F::buildOtaEndFrame(src, CellNodeId::SUPERVISOR, 1, 2, e);
  bytes.resize(F::kMinEthFrameLen, 0);
  CellNet::ParsedFrame pe{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), pe));
  TEST_ASSERT_EQUAL_UINT32(843360, pe.payload.ota_end.image_size);

  L2OtaResultPayload r{};
  r.session = 7;
  r.status = 4;
  r.erase_ms = 2788;
  r.emac_missed = 0;
  r.emac_overflow = 1;
  bytes = F::buildOtaResultFrame(src, CellNodeId::GANTRY, 1, 2, r);
  CellNet::ParsedFrame pr{};
  TEST_ASSERT_TRUE(F::parseFrame(bytes.data(), bytes.size(), pr));
  TEST_ASSERT_EQUAL_UINT16(2788, pr.payload.ota_result.erase_ms);
  TEST_ASSERT_EQUAL_UINT16(1, pr.payload.ota_result.emac_overflow);

  // The 64-byte cap still applies to every type but OTA_DATA.
  bytes.resize(F::kMaxPaddedRxLen + 1, 0);
  CellNet::ParsedFrame big{};
  TEST_ASSERT_FALSE(F::parseFrame(bytes.data(), bytes.size(), big));
}
