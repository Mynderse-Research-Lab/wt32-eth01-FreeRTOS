// Host unit tests for the pure EtherNet/IP + CIP encoding layer
// (lib/EtherNetIP). Every test pins exact bytes-on-the-wire and/or round-trips
// an encode->decode so the framing is regression-locked before any live socket
// transport is written. See docs/LOW_LEVEL_GANTRY_CONTROL.md for the byte maps.

#include "unity.h"

#include <array>
#include <cstdint>
#include <vector>

#include "CipMessageRouter.h"
#include "EipByteBuffer.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "Kinetix5100Assembly.h"

using eip::Bytes;

void setUp(void) {}
void tearDown(void) {}

namespace {

void assertBytesEqual(const Bytes& expected, const Bytes& actual) {
  TEST_ASSERT_EQUAL_UINT32(expected.size(), actual.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), actual.data(), expected.size());
}

}  // namespace

// --- ByteWriter / ByteReader little-endian round trip ------------------------
static void test_byte_buffer_le_roundtrip(void) {
  Bytes buf;
  eip::ByteWriter w(buf);
  w.u8(0xAB);
  w.u16(0x1234);
  w.u32(0xDEADBEEF);
  w.i32(-2);

  // u16 0x1234 -> 34 12 ; u32 0xDEADBEEF -> EF BE AD DE ; i32 -2 -> FE FF FF FF
  const Bytes expected = {0xAB, 0x34, 0x12, 0xEF, 0xBE, 0xAD, 0xDE,
                          0xFE, 0xFF, 0xFF, 0xFF};
  assertBytesEqual(expected, buf);

  eip::ByteReader r(buf);
  uint8_t a = 0;
  uint16_t b = 0;
  uint32_t c = 0;
  int32_t d = 0;
  TEST_ASSERT_TRUE(r.u8(a));
  TEST_ASSERT_TRUE(r.u16(b));
  TEST_ASSERT_TRUE(r.u32(c));
  TEST_ASSERT_TRUE(r.i32(d));
  TEST_ASSERT_EQUAL_HEX8(0xAB, a);
  TEST_ASSERT_EQUAL_HEX16(0x1234, b);
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, c);
  TEST_ASSERT_EQUAL_INT32(-2, d);
  TEST_ASSERT_EQUAL_UINT32(0, r.remaining());
}

static void test_byte_reader_bounds(void) {
  const Bytes buf = {0x01, 0x02};
  eip::ByteReader r(buf);
  uint32_t v = 0;
  TEST_ASSERT_FALSE(r.u32(v));  // only 2 bytes available
  uint16_t w = 0;
  TEST_ASSERT_TRUE(r.u16(w));
  TEST_ASSERT_EQUAL_HEX16(0x0201, w);
}

// --- Encapsulation header ----------------------------------------------------
static void test_register_session_request_bytes(void) {
  Bytes frame = eip::buildRegisterSessionRequest();
  // 24-byte header + 4-byte data (protocol version 1, options 0).
  TEST_ASSERT_EQUAL_UINT32(eip::kEncapHeaderSize + 4, frame.size());
  TEST_ASSERT_EQUAL_HEX8(0x65, frame[0]);  // command 0x0065 (LE)
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(0x04, frame[2]);  // length = 4 (LE)
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[3]);
  TEST_ASSERT_EQUAL_HEX8(0x01, frame[24]);  // protocol version = 1
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[25]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[26]);  // options = 0
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[27]);
}

static void test_encapsulation_decode_roundtrip(void) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kSendRRData);
  h.session_handle = 0x11223344;
  h.status = 0;
  const Bytes data = {0xAA, 0xBB, 0xCC};
  Bytes frame = eip::encodeEncapsulation(h, data);

  eip::EncapHeader out;
  size_t off = 0, len = 0;
  TEST_ASSERT_TRUE(eip::decodeEncapsulation(frame, out, off, len));
  TEST_ASSERT_EQUAL_HEX16(static_cast<uint16_t>(eip::EncapCommand::kSendRRData),
                          out.command);
  TEST_ASSERT_EQUAL_HEX32(0x11223344, out.session_handle);
  TEST_ASSERT_EQUAL_UINT32(eip::kEncapHeaderSize, off);
  TEST_ASSERT_EQUAL_UINT32(3, len);
}

static void test_encapsulation_decode_rejects_short_frame(void) {
  const Bytes too_short(10, 0x00);
  eip::EncapHeader out;
  size_t off = 0, len = 0;
  TEST_ASSERT_FALSE(eip::decodeEncapsulation(too_short, out, off, len));
}

static void test_encapsulation_decode_rejects_length_overrun(void) {
  // Valid 24-byte header that claims 50 bytes of data but supplies none.
  Bytes frame(eip::kEncapHeaderSize, 0x00);
  frame[0] = 0x6F;  // SendRRData
  frame[2] = 50;    // length low byte = 50
  eip::EncapHeader out;
  size_t off = 0, len = 0;
  TEST_ASSERT_FALSE(eip::decodeEncapsulation(frame, out, off, len));
}

static void test_encapsulation_rejects_malformed_headers(void) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kRegisterSession);
  h.session_handle = 0x0A0B0C0D;
  h.status = 1; // Non-zero status!
  const Bytes data = {0x01, 0x00, 0x00, 0x00};
  Bytes frame = eip::encodeEncapsulation(h, data);
  
  uint32_t handle = 0;
  // Should reject because status != 0
  TEST_ASSERT_FALSE(eip::parseRegisterSessionReply(frame, handle));

  // Change to wrong command
  h.status = 0;
  h.setCommand(eip::EncapCommand::kSendRRData);
  frame = eip::encodeEncapsulation(h, data);
  TEST_ASSERT_FALSE(eip::parseRegisterSessionReply(frame, handle));
}

static void test_register_session_reply_parse(void) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kRegisterSession);
  h.session_handle = 0x0A0B0C0D;
  h.status = 0;
  const Bytes data = {0x01, 0x00, 0x00, 0x00};
  Bytes frame = eip::encodeEncapsulation(h, data);

  uint32_t handle = 0;
  TEST_ASSERT_TRUE(eip::parseRegisterSessionReply(frame, handle));
  TEST_ASSERT_EQUAL_HEX32(0x0A0B0C0D, handle);
}

// --- Common Packet Format ----------------------------------------------------
static void test_cpf_encode_bytes(void) {
  std::vector<eip::CpfItem> items;
  items.emplace_back(eip::CpfItemType::kNullAddress, Bytes{});
  items.emplace_back(eip::CpfItemType::kUnconnectedData, Bytes{0xDE, 0xAD});
  Bytes cpf = eip::encodeCpf(items);

  // count=2 ; null addr (type 0x0000 len 0) ; unconn data (0x00B2 len 2 + data)
  const Bytes expected = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0xB2, 0x00, 0x02, 0x00, 0xDE, 0xAD};
  assertBytesEqual(expected, cpf);
}

static void test_send_rr_data_wrap_unwrap(void) {
  const Bytes cip = {0x0E, 0x03, 0x20, 0x04, 0x24, 0x9A, 0x30, 0x03};
  Bytes payload = eip::encodeSendRRDataPayload(cip);

  Bytes recovered;
  TEST_ASSERT_TRUE(eip::decodeSendRRDataPayload(payload, recovered));
  assertBytesEqual(cip, recovered);
}

static void test_cpf_decode_rejects_truncated_item(void) {
  // count=1, type 0x00B2, length=4, but only 2 data bytes present.
  const Bytes bad = {0x01, 0x00, 0xB2, 0x00, 0x04, 0x00, 0xAA, 0xBB};
  std::vector<eip::CpfItem> items;
  TEST_ASSERT_FALSE(eip::decodeCpf(bad, items));
}

// --- CIP Message Router + EPATH ---------------------------------------------
static void test_epath_8bit_segments(void) {
  // Assembly class 0x04, instance 154 (0x9A).
  Bytes path = eip::buildEPath(0x04, 154);
  const Bytes expected = {0x20, 0x04, 0x24, 0x9A};
  assertBytesEqual(expected, path);
}

static void test_epath_16bit_instance(void) {
  // Instance 1000 (0x03E8) forces the 16-bit logical segment + pad byte.
  Bytes path = eip::buildEPath(0x04, 1000);
  const Bytes expected = {0x20, 0x04, 0x25, 0x00, 0xE8, 0x03};
  assertBytesEqual(expected, path);
}

static void test_epath_with_attribute(void) {
  Bytes path = eip::buildEPath(0x04, 154, true, 3);
  const Bytes expected = {0x20, 0x04, 0x24, 0x9A, 0x30, 0x03};
  assertBytesEqual(expected, path);
}

static void test_message_router_request_bytes(void) {
  Bytes epath = eip::buildEPath(0x04, 154, true, 3);
  Bytes req = eip::buildMessageRouterRequest(
      static_cast<uint8_t>(eip::CipService::kGetAttributeSingle), epath);
  // service 0x0E, path size 3 words, then the 6 EPATH bytes.
  const Bytes expected = {0x0E, 0x03, 0x20, 0x04, 0x24, 0x9A, 0x30, 0x03};
  assertBytesEqual(expected, req);
}

static void test_message_router_response_parse_success(void) {
  const Bytes reply = {0x8E, 0x00, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78};
  eip::MessageRouterResponse resp;
  TEST_ASSERT_TRUE(eip::parseMessageRouterResponse(reply, resp));
  TEST_ASSERT_TRUE(resp.isSuccess());
  TEST_ASSERT_EQUAL_HEX8(0x8E, resp.reply_service);
  TEST_ASSERT_EQUAL_UINT32(4, resp.data.size());
  const Bytes expected_data = {0x12, 0x34, 0x56, 0x78};
  assertBytesEqual(expected_data, resp.data);
}

static void test_message_router_response_parse_error_status(void) {
  // general_status 0x05 (path destination unknown), 0 additional status.
  const Bytes reply = {0x8E, 0x00, 0x05, 0x00};
  eip::MessageRouterResponse resp;
  TEST_ASSERT_TRUE(eip::parseMessageRouterResponse(reply, resp));
  TEST_ASSERT_FALSE(resp.isSuccess());
  TEST_ASSERT_EQUAL_HEX8(0x05, resp.general_status);
}

// --- Connection Manager: network params, ForwardOpen -------------------------
static void test_network_connection_params_packing(void) {
  // size 52, point-to-point (2), scheduled priority (2), fixed size.
  uint16_t params = eip::makeNetworkConnectionParams(
      52, eip::ConnectionType::kPointToPoint, eip::ConnectionPriority::kScheduled);
  // 0x34 (size) | (2<<10) | (2<<13) = 0x34 | 0x0800 | 0x4000 = 0x4834
  TEST_ASSERT_EQUAL_HEX16(0x4834, params);
}

static void test_transport_class_trigger(void) {
  // Class 1, cyclic trigger (0), client direction -> 0x01.
  TEST_ASSERT_EQUAL_HEX8(0x01, eip::makeTransportClassTrigger(1, 0));
  // Class 3, change-of-state trigger (1), server direction -> 0x93.
  TEST_ASSERT_EQUAL_HEX8(0x93, eip::makeTransportClassTrigger(3, 1, true));
}

static void test_assembly_connection_path_kinetix(void) {
  // Kinetix 5100 EDS Connection1: "20 04 24 BF 2C 68 2C 9A"
  const Bytes path = eip::buildAssemblyConnectionPath(191, 104, 154);
  const Bytes expected = {0x20, 0x04, 0x24, 0xBF, 0x2C, 0x68, 0x2C, 0x9A};
  assertBytesEqual(expected, path);
}

static void test_multicast_ip_from_connection_id(void) {
  TEST_ASSERT_EQUAL_HEX32(0xEFC03412u,
                          eip::multicastIpFromConnectionId(0x00001234));
  TEST_ASSERT_EQUAL_HEX32(0xEFC0F0FCu,
                          eip::multicastIpFromConnectionId(0xFCF00000u));
}

static void test_forward_open_request_framing(void) {
  eip::ForwardOpenParams p;
  p.connection_path = eip::buildConnectionPointPath(0x04, 104);
  Bytes req = eip::buildForwardOpenRequest(p);

  // service 0x54, path size 2 words, CM path 20 06 24 01.
  TEST_ASSERT_EQUAL_HEX8(0x54, req[0]);
  TEST_ASSERT_EQUAL_HEX8(0x02, req[1]);
  const Bytes cm_path = {0x20, 0x06, 0x24, 0x01};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(cm_path.data(), req.data() + 2, 4);
  // First ForwardOpen body byte (priority/time tick) sits right after the path.
  TEST_ASSERT_EQUAL_HEX8(p.priority_time_tick, req[6]);
}

static void test_forward_open_reply_parse(void) {
  // Craft a ForwardOpen success response data block.
  Bytes data;
  eip::ByteWriter w(data);
  w.u32(0x10000001);  // O->T connection id
  w.u32(0x20000002);  // T->O connection id
  w.u16(0x0005);      // connection serial
  w.u16(0x004D);      // originator vendor id
  w.u32(0xCAFEF00D);  // originator serial
  w.u32(20000);       // O->T API (us)
  w.u32(20000);       // T->O API (us)
  w.u8(0);            // application reply size (words)
  w.u8(0);            // reserved

  eip::ForwardOpenReply reply;
  TEST_ASSERT_TRUE(eip::parseForwardOpenReply(data, reply));
  TEST_ASSERT_EQUAL_HEX32(0x10000001, reply.ot_connection_id);
  TEST_ASSERT_EQUAL_HEX32(0x20000002, reply.to_connection_id);
  TEST_ASSERT_EQUAL_HEX16(0x0005, reply.connection_serial);
  TEST_ASSERT_EQUAL_HEX32(0xCAFEF00D, reply.originator_serial);
  TEST_ASSERT_EQUAL_UINT32(20000, reply.ot_api_us);
}

// --- ListIdentity reply parse -----------------------------------------------
static void test_list_identity_reply_parse(void) {
  // Build the Identity item payload (CPF item type 0x000C data).
  Bytes id;
  eip::ByteWriter iw(id);
  iw.u16(1);  // encap protocol version
  // socket address: 16 bytes (sin_family, port, addr, zero) - opaque to parser.
  std::array<uint8_t, 16> sock{};
  sock[1] = 0x02;  // AF_INET (big-endian)
  iw.bytes(sock.data(), sock.size());
  iw.u16(0x004D);      // vendor id (Rockwell)
  iw.u16(0x000C);      // device type
  iw.u16(0x1234);      // product code
  iw.u8(2);            // revision major
  iw.u8(5);            // revision minor
  iw.u16(0x0030);      // status
  iw.u32(0xDEADBEEF);  // serial number
  const char* name = "Kinetix 5100";
  iw.u8(static_cast<uint8_t>(12));
  iw.bytes(reinterpret_cast<const uint8_t*>(name), 12);
  iw.u8(3);  // state

  std::vector<eip::CpfItem> items;
  items.emplace_back(eip::CpfItemType::kListIdentityResponse, id);
  Bytes cpf = eip::encodeCpf(items);

  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kListIdentity);
  Bytes frame = eip::encodeEncapsulation(h, cpf);

  eip::Identity out;
  TEST_ASSERT_TRUE(eip::parseListIdentityReply(frame, out));
  TEST_ASSERT_EQUAL_HEX16(0x004D, out.vendor_id);
  TEST_ASSERT_EQUAL_HEX16(0x1234, out.product_code);
  TEST_ASSERT_EQUAL_UINT8(2, out.revision_major);
  TEST_ASSERT_EQUAL_UINT8(5, out.revision_minor);
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, out.serial_number);
  TEST_ASSERT_EQUAL_STRING("Kinetix 5100", out.product_name.c_str());
  TEST_ASSERT_EQUAL_UINT8(3, out.state);
}

// --- Kinetix 5100 assemblies -------------------------------------------------
static void test_k5100_output_104_serialize(void) {
  eip::k5100::OutputAssembly104 out;
  out.operating_mode = static_cast<int8_t>(eip::k5100::OperatingMode::kPosition);
  out.servo_on = true;       // bit0
  out.start_motion = true;   // bit4 -> control byte 0x11
  out.speed_reference = 1000;        // 0x000003E8
  out.position_reference = 0x00010000;
  out.non_cyclic_move_type =
      static_cast<int8_t>(eip::k5100::NonCyclicMoveType::kRelative);
  out.travel_mode = 2;
  out.position_command_override = true;  // byte 27 bit0
  out.torque_reference = -100;           // 0xFFFFFF9C
  out.torque_ramp_time = 100;
  out.starting_index = 5;

  Bytes bytes = out.serialize();
  TEST_ASSERT_EQUAL_UINT32(eip::k5100::kOutput104Size, bytes.size());
  TEST_ASSERT_EQUAL_HEX8(0x01, bytes[0]);   // operating mode
  TEST_ASSERT_EQUAL_HEX8(0x11, bytes[1]);   // control: servo_on | start_motion
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[2]);   // reserved
  // speed reference 1000 -> E8 03 00 00
  TEST_ASSERT_EQUAL_HEX8(0xE8, bytes[4]);
  TEST_ASSERT_EQUAL_HEX8(0x03, bytes[5]);
  // position reference 0x00010000 -> 00 00 01 00
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[16]);
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[17]);
  TEST_ASSERT_EQUAL_HEX8(0x01, bytes[18]);
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[19]);
  TEST_ASSERT_EQUAL_HEX8(0x01, bytes[24]);  // non-cyclic move type = relative
  TEST_ASSERT_EQUAL_HEX8(0x02, bytes[26]);  // travel mode
  TEST_ASSERT_EQUAL_HEX8(0x01, bytes[27]);  // flags: override
  // torque reference -100 -> 9C FF FF FF
  TEST_ASSERT_EQUAL_HEX8(0x9C, bytes[28]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, bytes[29]);
  TEST_ASSERT_EQUAL_HEX8(0x64, bytes[32]);  // torque ramp time 100
  TEST_ASSERT_EQUAL_HEX8(0x05, bytes[36]);  // starting index
  TEST_ASSERT_EQUAL_HEX8(0x00, bytes[39]);  // trailing pad
}

static void test_k5100_input_154_deserialize(void) {
  // Build a 52-byte instance-154 frame with known field placements.
  Bytes frame(eip::k5100::kInput154Size, 0x00);
  frame[0] = 0x01;   // run_mode (bit0)
  frame[1] = 7;      // diagnostic sequence count
  frame[8] = 0x02;   // fault (bit1)
  // byte9: ready(bit3)=1, active(bit2)=1, at_reference(bit7)=1 -> 0x8C
  frame[9] = 0x8C;
  frame[11] = 0x01;  // operating mode = position
  frame[12] = 0x03;  // active index = 3
  frame[15] = 0x01;  // motor type = rotary
  // actual speed 1500 -> DC 05 00 00 at bytes 16-19
  frame[16] = 0xDC;
  frame[17] = 0x05;
  frame[20] = 0x42;  // fault code low (0x0042)
  frame[24] = 0x00;  // actual position 0x00010000 at 24-27
  frame[26] = 0x01;
  frame[28] = 0x14;  // actual torque 0x14 (20) at 28-31
  frame[48] = 0x09;  // parameter monitor 5 low byte at 48-51

  eip::k5100::InputAssembly154 in;
  TEST_ASSERT_TRUE(in.deserialize(frame));
  TEST_ASSERT_TRUE(in.run_mode);
  TEST_ASSERT_FALSE(in.connection_faulted);
  TEST_ASSERT_EQUAL_INT8(7, in.diagnostic_sequence_count);
  TEST_ASSERT_TRUE(in.fault);
  TEST_ASSERT_TRUE(in.ready);
  TEST_ASSERT_TRUE(in.active);
  TEST_ASSERT_TRUE(in.at_reference);
  TEST_ASSERT_FALSE(in.stopped);
  TEST_ASSERT_EQUAL_INT8(1, in.operating_mode);
  TEST_ASSERT_EQUAL_INT8(3, in.active_index);
  TEST_ASSERT_EQUAL_INT8(1, in.motor_type);
  TEST_ASSERT_EQUAL_INT32(1500, in.actual_speed);
  TEST_ASSERT_EQUAL_HEX16(0x0042, in.fault_code);
  TEST_ASSERT_EQUAL_INT32(0x00010000, in.actual_position);
  TEST_ASSERT_EQUAL_INT32(20, in.actual_torque);
  TEST_ASSERT_EQUAL_INT32(9, in.parameter_monitor[4]);
}

static void test_k5100_input_154_rejects_short(void) {
  Bytes too_short(20, 0x00);
  eip::k5100::InputAssembly154 in;
  TEST_ASSERT_FALSE(in.deserialize(too_short));
}

static void test_k5100_output_roundtrips_through_input_offsets(void) {
  // Sanity: an output serialize followed by re-reading select DINTs via the
  // ByteReader yields the same values (guards against offset drift).
  eip::k5100::OutputAssembly104 out;
  out.speed_reference = 12345;
  out.position_reference = -98765;
  Bytes bytes = out.serialize();

  eip::ByteReader r(bytes);
  TEST_ASSERT_TRUE(r.skip(4));  // to speed reference
  int32_t speed = 0;
  TEST_ASSERT_TRUE(r.i32(speed));
  TEST_ASSERT_EQUAL_INT32(12345, speed);
  TEST_ASSERT_TRUE(r.skip(8));  // accel + decel
  int32_t pos = 0;
  TEST_ASSERT_TRUE(r.i32(pos));
  TEST_ASSERT_EQUAL_INT32(-98765, pos);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_byte_buffer_le_roundtrip);
  RUN_TEST(test_byte_reader_bounds);
  RUN_TEST(test_register_session_request_bytes);
  RUN_TEST(test_encapsulation_decode_roundtrip);
  RUN_TEST(test_encapsulation_decode_rejects_short_frame);
  RUN_TEST(test_encapsulation_decode_rejects_length_overrun);
  RUN_TEST(test_encapsulation_rejects_malformed_headers);
  RUN_TEST(test_register_session_reply_parse);
  RUN_TEST(test_cpf_encode_bytes);
  RUN_TEST(test_send_rr_data_wrap_unwrap);
  RUN_TEST(test_cpf_decode_rejects_truncated_item);
  RUN_TEST(test_epath_8bit_segments);
  RUN_TEST(test_epath_16bit_instance);
  RUN_TEST(test_epath_with_attribute);
  RUN_TEST(test_message_router_request_bytes);
  RUN_TEST(test_message_router_response_parse_success);
  RUN_TEST(test_message_router_response_parse_error_status);
  RUN_TEST(test_network_connection_params_packing);
  RUN_TEST(test_transport_class_trigger);
  RUN_TEST(test_assembly_connection_path_kinetix);
  RUN_TEST(test_multicast_ip_from_connection_id);
  RUN_TEST(test_forward_open_request_framing);
  RUN_TEST(test_forward_open_reply_parse);
  RUN_TEST(test_list_identity_reply_parse);
  RUN_TEST(test_k5100_output_104_serialize);
  RUN_TEST(test_k5100_input_154_deserialize);
  RUN_TEST(test_k5100_input_154_rejects_short);
  RUN_TEST(test_k5100_output_roundtrips_through_input_offsets);
  return UNITY_END();
}
