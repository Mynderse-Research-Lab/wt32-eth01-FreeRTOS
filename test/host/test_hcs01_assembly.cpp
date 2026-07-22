// Host unit tests for HCS01 control/status words and positioning assemblies.
// Byte maps from docs/LOW_LEVEL_GANTRY_CONTROL.md / HCS01 sources.

#include "unity.h"

#include "Hcs01Assembly.h"
#include "Hcs01ControlStatus.h"

void setUp(void) {}
void tearDown(void) {}

using eip::Bytes;
using eip::hcs01::Hcs01ControlWord;
using eip::hcs01::Hcs01PositioningActual;
using eip::hcs01::Hcs01PositioningCommand;
using eip::hcs01::Hcs01StatusWord;
using eip::hcs01::ReadyForOperation;

namespace {
}  // namespace

static void test_control_word_enable_sequence(void) {
  Hcs01ControlWord cw = Hcs01ControlWord::makeDriveEnable();
  uint16_t raw = cw.encode();
  TEST_ASSERT_BITS_HIGH(0x8000, raw);  // bit15 Drive ON
  TEST_ASSERT_BITS_HIGH(0x2000, raw);  // bit13 Drive Halt/Start

  Hcs01ControlWord safe = Hcs01ControlWord::makeBusFailureSafe();
  TEST_ASSERT_EQUAL_HEX16(0x0000, safe.encode());
}

static void test_control_word_roundtrip(void) {
  Hcs01ControlWord cw;
  cw.command_value_accept = true;
  cw.relative_positioning = true;
  cw.immediate_block_change = true;
  cw.drive_on = true;
  cw.drive_halt = true;
  cw.positioning_jog = eip::hcs01::PositioningJogMode::kPositioning;

  uint16_t raw = cw.encode();
  // bit0 + bit3 + bit4 + bit13 + bit15 = 0xA019
  TEST_ASSERT_EQUAL_HEX16(0xA019, raw);

  Hcs01ControlWord decoded = Hcs01ControlWord::decode(raw);
  TEST_ASSERT_TRUE(decoded.command_value_accept);
  TEST_ASSERT_TRUE(decoded.relative_positioning);
  TEST_ASSERT_TRUE(decoded.immediate_block_change);
  TEST_ASSERT_TRUE(decoded.drive_on);
  TEST_ASSERT_TRUE(decoded.drive_halt);
}

static void test_status_word_decode(void) {
  // bit10 command ack, bit4 in position, bits15/14 = 11 (in operation),
  // bit2 homed.
  const uint16_t raw = 0xC000 | 0x0400 | 0x0010 | 0x0004;
  Hcs01StatusWord sw = Hcs01StatusWord::decode(raw);
  TEST_ASSERT_TRUE(sw.command_value_ack);
  TEST_ASSERT_TRUE(sw.command_value_reached);
  TEST_ASSERT_TRUE(sw.in_reference);
  TEST_ASSERT_EQUAL(ReadyForOperation::kInOperation, sw.ready);
  TEST_ASSERT_TRUE(sw.isReadyForOperation());
}

static void test_status_word_roundtrip(void) {
  Hcs01StatusWord sw;
  sw.operating_mode_ack = eip::hcs01::OperatingModeAck::kOperatingMode;
  sw.in_standstill = true;
  sw.class1_error = true;
  sw.ready = ReadyForOperation::kReadyAb;

  uint16_t raw = sw.encode();
  Hcs01StatusWord decoded = Hcs01StatusWord::decode(raw);
  TEST_ASSERT_EQUAL(sw.operating_mode_ack, decoded.operating_mode_ack);
  TEST_ASSERT_TRUE(decoded.in_standstill);
  TEST_ASSERT_TRUE(decoded.class1_error);
  TEST_ASSERT_EQUAL(sw.ready, decoded.ready);
}

static void test_positioning_command_serialize(void) {
  Hcs01PositioningCommand cmd;
  cmd.control.drive_on = true;
  cmd.control.drive_halt = true;
  cmd.control.command_value_accept = true;
  cmd.positioning_command_value = 0x00010000;
  cmd.positioning_velocity = 5000;

  Bytes bytes = cmd.serialize();
  TEST_ASSERT_EQUAL_UINT32(eip::hcs01::kOutput101Size, bytes.size());
  // control: 0xA001 (drive_on + drive_halt + command_value_accept)
  TEST_ASSERT_EQUAL_HEX8(0x01, bytes[0]);
  TEST_ASSERT_EQUAL_HEX8(0xA0, bytes[1]);
  // position 0x00010000 -> 00 00 01 00
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[3]);
  TEST_ASSERT_EQUAL_HEX8(0x01, bytes[4]);
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[5]);
  // velocity 5000 -> 0x1388
  TEST_ASSERT_EQUAL_HEX8(0x88, bytes[6]);
  TEST_ASSERT_EQUAL_HEX8(0x13, bytes[7]);
}

static void test_positioning_actual_deserialize(void) {
  Bytes frame(eip::hcs01::kInput102Size, 0x00);
  // status: command ack (bit10) + in position (bit4) + in operation (bits15/14=11)
  const uint16_t status = 0xC000 | 0x0400 | 0x0010;
  frame[0] = static_cast<uint8_t>(status & 0xFF);
  frame[1] = static_cast<uint8_t>((status >> 8) & 0xFF);
  // position feedback 0x00005000 at offset 2
  frame[2] = 0x00;
  frame[3] = 0x50;
  frame[4] = 0x00;
  frame[5] = 0x00;
  // velocity feedback 100 at offset 6
  frame[6] = 0x64;
  // diagnostic 0xDEADBEEF at offset 10
  frame[10] = 0xEF;
  frame[11] = 0xBE;
  frame[12] = 0xAD;
  frame[13] = 0xDE;

  Hcs01PositioningActual actual;
  TEST_ASSERT_TRUE(actual.deserialize(frame));
  TEST_ASSERT_TRUE(actual.status.command_value_ack);
  TEST_ASSERT_TRUE(actual.status.command_value_reached);
  TEST_ASSERT_EQUAL(ReadyForOperation::kInOperation, actual.status.ready);
  TEST_ASSERT_EQUAL_INT32(0x00005000, actual.position_feedback);
  TEST_ASSERT_EQUAL_INT32(100, actual.velocity_feedback);
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, actual.diagnostic_message);
}

static void test_positioning_actual_rejects_short(void) {
  Bytes too_short(8, 0x00);
  Hcs01PositioningActual actual;
  TEST_ASSERT_FALSE(actual.deserialize(too_short));
}

static void test_move_handshake_bits(void) {
  // Issue move: toggle command bit0; expect status bit10 to toggle on accept.
  Hcs01ControlWord issue;
  issue.command_value_accept = true;
  TEST_ASSERT_BITS_HIGH(0x0001, issue.encode());

  Hcs01StatusWord accepted = Hcs01StatusWord::decode(0x0400);  // bit10 set
  TEST_ASSERT_TRUE(accepted.command_value_ack);

  Hcs01StatusWord done = Hcs01StatusWord::decode(0x0410);  // bit10 + bit4
  TEST_ASSERT_TRUE(done.command_value_ack);
  TEST_ASSERT_TRUE(done.command_value_reached);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_control_word_enable_sequence);
  RUN_TEST(test_control_word_roundtrip);
  RUN_TEST(test_status_word_decode);
  RUN_TEST(test_status_word_roundtrip);
  RUN_TEST(test_positioning_command_serialize);
  RUN_TEST(test_positioning_actual_deserialize);
  RUN_TEST(test_positioning_actual_rejects_short);
  RUN_TEST(test_move_handshake_bits);
  return UNITY_END();
}
