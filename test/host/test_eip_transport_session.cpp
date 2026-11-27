/**
 * @file test_eip_transport_session.cpp
 * @brief Session, CPF and scanner host tests.
 */

#include "test_eip_transport_rigs.h"

using namespace eip_transport_test;

void test_session_register(void) {
  FakeTcpClient tcp;
  tcp.enqueueResponse(makeRegisterSessionReply(0x0A0B0C0D));

  eip::EipSession session(tcp);
  TEST_ASSERT_TRUE(session.registerSession());
  TEST_ASSERT_EQUAL_HEX32(0x0A0B0C0D, session.sessionHandle());

  TEST_ASSERT_EQUAL_HEX8(0x65, tcp.lastSent()[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, tcp.lastSent()[1]);
}

void test_class1_cpf_bytes(void) {
  const Bytes assembly = {0xDE, 0xAD};
  Bytes cpf = eip::buildClass1OutputCpf(0x10000001, 7, 42, assembly, true);

  const Bytes expected = {
      0x02, 0x00,
      0x02, 0x80, 0x08, 0x00,
      0x01, 0x00, 0x00, 0x10,
      0x07, 0x00, 0x00, 0x00,
      0xB1, 0x00, 0x08, 0x00,
      0x2A, 0x00,
      0x01, 0x00, 0x00, 0x00,
      0xDE, 0xAD};
  TEST_ASSERT_EQUAL_UINT32(expected.size(), cpf.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), cpf.data(), expected.size());
}

void test_io_connection_output_frame(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x20000002;
  cfg.session_handle = 0x12345678;
  cfg.ot_include_run_idle_header = false;
  cfg.to_include_run_idle_header = false;
  io.setConfig(cfg);

  const Bytes assembly = {0x11, 0x22, 0x33};
  Bytes frame = io.buildOutputFrame(assembly);

  // Class 1 implicit O->T is a raw CPF UDP payload (no encapsulation header).
  // Layout: 2 item count + 12 sequenced address + 4 item header + 2 seq + 3 data = 23.
  TEST_ASSERT_EQUAL_UINT32(23u, frame.size());
  TEST_ASSERT_EQUAL_HEX8(0x02, frame[0]);  // item count (little-endian)
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(0x02, frame[2]);  // sequenced address type 0x8002
  TEST_ASSERT_EQUAL_HEX8(0x80, frame[3]);
  // bytes 4-5: sequenced address item length = 8
  TEST_ASSERT_EQUAL_HEX8(0x08, frame[4]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[5]);
  // bytes 6-9: connection ID 0x20000002 (little-endian)
  TEST_ASSERT_EQUAL_HEX8(0x02, frame[6]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[7]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[8]);
  TEST_ASSERT_EQUAL_HEX8(0x20, frame[9]);
  // bytes 10-13: encapsulation sequence 1
  TEST_ASSERT_EQUAL_HEX8(0x01, frame[10]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[11]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[12]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[13]);
  // bytes 14-15: connected data type 0x00B1
  TEST_ASSERT_EQUAL_HEX8(0xB1, frame[14]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[15]);
  // bytes 16-17: item length = 5
  TEST_ASSERT_EQUAL_HEX8(0x05, frame[16]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[17]);
  // bytes 18-19: CIP sequence count
  TEST_ASSERT_EQUAL_HEX8(0x01, frame[18]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[19]);
  // bytes 20-22: assembly data
  TEST_ASSERT_EQUAL_HEX8(0x11, frame[20]);
  TEST_ASSERT_EQUAL_HEX8(0x22, frame[21]);
  TEST_ASSERT_EQUAL_HEX8(0x33, frame[22]);
  // Validate the produced CPF is also self-parsable.
  {
    Bytes parsed;
    TEST_ASSERT_TRUE(io.parseInputFrame(frame, parsed));
    TEST_ASSERT_EQUAL_UINT32(assembly.size(), parsed.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly.data(), parsed.data(), parsed.size());
  }
  TEST_ASSERT_EQUAL_UINT16(1, io.encapSequence());
  TEST_ASSERT_EQUAL_UINT16(1, io.cipSequence());
}

void test_io_connection_parse_roundtrip(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x10000001;
  cfg.session_handle = 0xABCDEF01;
  cfg.ot_include_run_idle_header = true;
  cfg.to_include_run_idle_header = true;
  io.setConfig(cfg);

  const Bytes assembly_in = {0xAA, 0xBB};
  Bytes frame = io.buildOutputFrame(assembly_in);

  Bytes parsed;
  TEST_ASSERT_TRUE(io.parseInputFrame(frame, parsed));
  TEST_ASSERT_EQUAL_UINT32(assembly_in.size(), parsed.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly_in.data(), parsed.data(), parsed.size());
}

void test_io_connection_parse_t_to_o_cpf(void) {
  FakeUdpEndpoint udp;
  eip::EipIoConnection io(udp);
  eip::IoConnectionConfig cfg;
  cfg.connection_id = 0x10000001;
  cfg.to_include_run_idle_header = false;
  io.setConfig(cfg);

  // Raw T->O CPF: 1 sequenced address item + 1 connected data item.
  const Bytes assembly = {0x10, 0x11, 0x12, 0x13};
  Bytes cpf;
  eip::ByteWriter w(cpf);
  w.u16(2);                            // item count
  w.u16(0x8002);                       // sequenced address type
  w.u16(8);                            // length
  w.u32(0x20000002);                   // connection ID
  w.u32(1);                            // sequence
  w.u16(0x00B1);                       // connected data type
  w.u16(static_cast<uint16_t>(2 + assembly.size()));  // length
  w.u16(1);                            // CIP sequence count
  w.bytes(assembly);

  Bytes parsed;
  TEST_ASSERT_TRUE(io.parseInputFrame(cpf, parsed));
  TEST_ASSERT_EQUAL_UINT32(assembly.size(), parsed.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly.data(), parsed.data(), parsed.size());
}

void test_session_explicit_roundtrip(void) {
  FakeTcpClient tcp;
  tcp.enqueueResponse(makeRegisterSessionReply(0x11111111));

  eip::EipSession session(tcp);
  TEST_ASSERT_TRUE(session.registerSession());

  eip::EncapHeader rh;
  rh.setCommand(eip::EncapCommand::kSendRRData);
  rh.session_handle = 0x11111111;
  rh.status = 0;
  const Bytes cip_reply = {0x8E, 0x00, 0x00, 0x00};
  Bytes payload = eip::encodeSendRRDataPayload(cip_reply);
  tcp.enqueueResponse(eip::encodeEncapsulation(rh, payload));

  const Bytes cip_req = {0x0E, 0x03, 0x20, 0x04, 0x24, 0x9A, 0x30, 0x03};
  Bytes cip_resp;
  TEST_ASSERT_TRUE(session.sendExplicit(cip_req, cip_resp));
  TEST_ASSERT_EQUAL_UINT32(cip_reply.size(), cip_resp.size());
}

void test_scanner_connect_forward_open(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  const uint32_t handle = 0x11111111;
  const eip::ForwardOpenReply fo = makeSampleForwardOpenReply();

  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(fo)));

  eip::EipScanner scanner(tcp, udp, makeTestScannerConfig());
  TEST_ASSERT_TRUE(scanner.connect());
  TEST_ASSERT_EQUAL(eip::EipScanner::State::kConnected, scanner.state());
  TEST_ASSERT_EQUAL_HEX32(fo.ot_connection_id, scanner.openReply().ot_connection_id);
  TEST_ASSERT_EQUAL_HEX32(fo.to_connection_id, scanner.openReply().to_connection_id);
  TEST_ASSERT_TRUE(tcp.sentContainsCipService(
      static_cast<uint8_t>(eip::CipService::kForwardOpen)));
  scanner.disconnect();
}

void test_scanner_full_lifecycle(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  udp.setEcho(true);
  const uint32_t handle = 0x22222222;
  const eip::ForwardOpenReply fo = makeSampleForwardOpenReply();

  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(fo)));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardCloseMrReply()));
  tcp.enqueueResponse(makeRegisterSessionReply(0));  // UnRegisterSession reply

  eip::EipScanner scanner(tcp, udp, makeTestScannerConfig());
  TEST_ASSERT_TRUE(scanner.connect());

  Bytes input;
  TEST_ASSERT_TRUE(scanner.exchangeOnce(scanner.recvTimeoutMs(), input));
  TEST_ASSERT_GREATER_THAN(0, input.size());

  scanner.disconnect();
  TEST_ASSERT_EQUAL(eip::EipScanner::State::kIdle, scanner.state());
  TEST_ASSERT_TRUE(tcp.sentContainsCipService(
      static_cast<uint8_t>(eip::CipService::kForwardClose)));
}

void test_scanner_recv_timeout(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  const uint32_t handle = 0x33333333;

  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(makeSendRRDataEncapReply(
      handle, makeForwardOpenMrReply(makeSampleForwardOpenReply())));

  eip::EipScanner scanner(tcp, udp, makeTestScannerConfig());
  TEST_ASSERT_TRUE(scanner.connect());

  Bytes input;
  TEST_ASSERT_FALSE(scanner.exchangeOnce(1, input));

  scanner.disconnect();
}

void test_parse_class1_input_cpf_demux_id(void) {
  const Bytes assembly = {0x01, 0x02, 0x03, 0x04};
  const Bytes frame =
      eip::buildClass1OutputCpf(0xAABBCCDD, 9, 3, assembly, false);

  uint32_t cid = 0;
  Bytes out;
  TEST_ASSERT_TRUE(eip::parseClass1InputCpf(frame, cid, out, false));
  TEST_ASSERT_EQUAL_HEX32(0xAABBCCDD, cid);
  TEST_ASSERT_EQUAL_UINT32(assembly.size(), out.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(assembly.data(), out.data(), out.size());
}

void test_parse_class1_input_cpf_view(void) {
  const Bytes assembly = {0x11, 0x22, 0x33, 0x44, 0x55};
  const Bytes frame =
      eip::buildClass1OutputCpf(0xDEADBEEF, 5, 7, assembly, false);

  uint32_t cid_copy = 0;
  Bytes out_copy;
  TEST_ASSERT_TRUE(
      eip::parseClass1InputCpf(frame, cid_copy, out_copy, false));

  uint32_t cid_view = 0;
  const uint8_t* assy = nullptr;
  size_t assy_len = 0;
  TEST_ASSERT_TRUE(eip::parseClass1InputCpfView(
      frame.data(), frame.size(), cid_view, assy, assy_len, false));
  TEST_ASSERT_EQUAL_HEX32(cid_copy, cid_view);
  TEST_ASSERT_EQUAL_UINT32(out_copy.size(), assy_len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(out_copy.data(), assy, assy_len);
  // View pointer must lie inside the input frame buffer.
  TEST_ASSERT_TRUE(assy >= frame.data());
  TEST_ASSERT_TRUE(assy + assy_len <= frame.data() + frame.size());

  // Truncated frame
  TEST_ASSERT_FALSE(eip::parseClass1InputCpfView(frame.data(), 3, cid_view,
                                                 assy, assy_len, false));

  // Bad item length (claims more bytes than remain)
  Bytes bad = frame;
  // Sequenced-address item length at bytes 4-5; inflate past end.
  bad[4] = 0xFF;
  bad[5] = 0x00;
  TEST_ASSERT_FALSE(eip::parseClass1InputCpfView(bad.data(), bad.size(),
                                                 cid_view, assy, assy_len,
                                                 false));

  // No connected-data item: only sequenced address
  Bytes addr_only;
  {
    eip::ByteWriter w(addr_only);
    w.u16(1);  // one item
    w.u16(0x8002);
    w.u16(8);
    w.u32(0x12345678);
    w.u32(1);
  }
  TEST_ASSERT_FALSE(eip::parseClass1InputCpfView(
      addr_only.data(), addr_only.size(), cid_view, assy, assy_len, false));
}

