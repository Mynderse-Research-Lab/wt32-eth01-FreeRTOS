// Host unit tests for EipProcessImage, GantryEipLinearAxis, GantryEipRotaryAxis,
// and scanner/axis process-image integration.

#include "unity.h"

#include <algorithm>
#include <cstring>
#include <vector>

#include "CipMessageRouter.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipTransport.h"
#include "GantryEipLinearAxis.h"
#include "GantryEipRotaryAxis.h"
#include "Hcs01Assembly.h"
#include "Kinetix5100Assembly.h"
#include "KinetixFaultCodes.h"

void setUp(void) {}
void tearDown(void) {}

using eip::Bytes;

namespace {

class FakeTcpClient : public eip::ITcpClient {
 public:
  void enqueueResponse(Bytes response) {
    responses_.push_back(std::move(response));
  }
  bool connect(const char*, uint16_t) override { return true; }
  ssize_t send(const uint8_t* data, size_t len) override {
    last_sent_.assign(data, data + len);
    return static_cast<ssize_t>(len);
  }
  ssize_t recv(uint8_t* buf, size_t max_len, uint32_t) override {
    if (response_index_ >= responses_.size()) return -1;
    const Bytes& r = responses_[response_index_++];
    const size_t n = std::min(max_len, r.size());
    std::memcpy(buf, r.data(), n);
    return static_cast<ssize_t>(n);
  }
  void close() override {}
  bool isConnected() const override { return true; }

 private:
  std::vector<Bytes> responses_;
  size_t response_index_ = 0;
  Bytes last_sent_;
};

class FakeUdpEndpoint : public eip::IUdpEndpoint {
 public:
  void setEcho(bool echo) { echo_ = echo; }
  bool bind(uint16_t, uint32_t = 0) override { return true; }
  ssize_t sendTo(const uint8_t* data, size_t len, const char*, uint16_t) override {
    last_sent_.assign(data, data + len);
    if (echo_) echo_frame_.assign(data, data + len);
    return static_cast<ssize_t>(len);
  }
  ssize_t recvFrom(uint8_t* buf, size_t max_len, uint32_t) override {
    if (echo_ && !echo_frame_.empty()) {
      const size_t n = std::min(max_len, echo_frame_.size());
      std::memcpy(buf, echo_frame_.data(), n);
      echo_frame_.clear();
      return static_cast<ssize_t>(n);
    }
    return -1;
  }
  void close() override {}

 private:
  bool echo_ = false;
  Bytes last_sent_;
  Bytes echo_frame_;
};

Bytes makeRegisterSessionReply(uint32_t handle) {
  eip::EncapHeader h;
  h.setCommand(eip::EncapCommand::kRegisterSession);
  h.session_handle = handle;
  h.status = 0;
  return eip::encodeEncapsulation(h, Bytes{0x01, 0x00, 0x00, 0x00});
}

Bytes makeSendRRDataEncapReply(uint32_t session_handle, const Bytes& cip_reply) {
  eip::EncapHeader rh;
  rh.setCommand(eip::EncapCommand::kSendRRData);
  rh.session_handle = session_handle;
  rh.status = 0;
  return eip::encodeEncapsulation(rh, eip::encodeSendRRDataPayload(cip_reply));
}

Bytes makeForwardOpenMrReply(const eip::ForwardOpenReply& fo) {
  Bytes data;
  eip::ByteWriter w(data);
  w.u32(fo.ot_connection_id);
  w.u32(fo.to_connection_id);
  w.u16(fo.connection_serial);
  w.u16(fo.originator_vendor_id);
  w.u32(fo.originator_serial);
  w.u32(fo.ot_api_us);
  w.u32(fo.to_api_us);
  w.u8(0);
  w.u8(0);

  Bytes cip;
  eip::ByteWriter cw(cip);
  cw.u8(static_cast<uint8_t>(eip::CipService::kForwardOpen) |
        eip::kCipReplyServiceMask);
  cw.u8(0);
  cw.u8(0);
  cw.u8(0);
  cw.bytes(data);
  return cip;
}

eip::ForwardOpenReply makeSampleForwardOpenReply() {
  eip::ForwardOpenReply fo;
  fo.ot_connection_id = 0x10000001;
  fo.to_connection_id = 0x20000002;
  fo.connection_serial = 0x0005;
  fo.originator_vendor_id = 0x004D;
  fo.originator_serial = 0xCAFEF00D;
  fo.ot_api_us = 20000;
  fo.to_api_us = 20000;
  return fo;
}

Bytes makeK5100Input154(int32_t position, bool fault, bool cmd_in_progress,
                        bool at_reference, uint16_t fault_code = 0,
                        uint16_t warning_code = 0, bool warning_present = false) {
  Bytes frame(eip::k5100::kInput154Size, 0);
  if (fault) frame[8] |= 0x02;
  if (warning_present) frame[9] |= 0x02;
  if (cmd_in_progress) frame[9] |= 0x10;
  if (at_reference) frame[9] |= 0x80;
  frame[20] = static_cast<uint8_t>(fault_code & 0xFF);
  frame[21] = static_cast<uint8_t>((fault_code >> 8) & 0xFF);
  frame[22] = static_cast<uint8_t>(warning_code & 0xFF);
  frame[23] = static_cast<uint8_t>((warning_code >> 8) & 0xFF);
  frame[24] = static_cast<uint8_t>(position & 0xFF);
  frame[25] = static_cast<uint8_t>((position >> 8) & 0xFF);
  frame[26] = static_cast<uint8_t>((position >> 16) & 0xFF);
  frame[27] = static_cast<uint8_t>((position >> 24) & 0xFF);
  return frame;
}

Bytes makeHcs01Input102(int32_t position_puu, bool class1_error,
                        bool command_reached) {
  eip::hcs01::Hcs01PositioningActual actual;
  actual.status = eip::hcs01::Hcs01StatusWord::decode(0xC410);
  if (class1_error) {
    actual.status.class1_error = true;
    actual.status.ready = eip::hcs01::ReadyForOperation::kNotReady;
  }
  if (!command_reached) {
    actual.status.command_value_reached = false;
  }
  actual.position_feedback = position_puu;
  actual.velocity_feedback = 100;
  actual.diagnostic_message = 0;

  Bytes frame;
  eip::ByteWriter w(frame);
  w.u16(actual.status.encode());
  w.i32(actual.position_feedback);
  w.i32(actual.velocity_feedback);
  w.u32(actual.diagnostic_message);
  return frame;
}

int32_t readLeI32(const Bytes& data, size_t offset) {
  return static_cast<int32_t>(
      static_cast<uint32_t>(data[offset]) |
      (static_cast<uint32_t>(data[offset + 1]) << 8) |
      (static_cast<uint32_t>(data[offset + 2]) << 16) |
      (static_cast<uint32_t>(data[offset + 3]) << 24));
}

}  // namespace

static void test_process_image_command_feedback(void) {
  eip::EipProcessImage image;
  const Bytes cmd = {0x01, 0x02, 0x03};
  const Bytes fb = {0xAA, 0xBB};

  image.setCommand(cmd);
  TEST_ASSERT_TRUE(image.hasCommand());
  Bytes out_cmd;
  TEST_ASSERT_TRUE(image.getCommand(out_cmd));
  TEST_ASSERT_EQUAL_UINT32(cmd.size(), out_cmd.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(cmd.data(), out_cmd.data(), cmd.size());

  TEST_ASSERT_FALSE(image.feedbackFresh());
  image.setFeedback(fb);
  TEST_ASSERT_TRUE(image.feedbackFresh());
  Bytes out_fb;
  TEST_ASSERT_TRUE(image.getFeedback(out_fb));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(fb.data(), out_fb.data(), fb.size());
  image.consumeFeedbackFresh();
  TEST_ASSERT_FALSE(image.feedbackFresh());
}

static void test_process_image_online(void) {
  eip::EipProcessImage image;
  TEST_ASSERT_FALSE(image.isOnline());
  image.setOnline(true);
  TEST_ASSERT_TRUE(image.isOnline());
  image.setOnline(false);
  TEST_ASSERT_FALSE(image.isOnline());
}

static void armAxisForMove(Gantry::GantryEipLinearAxis& axis,
                           eip::EipProcessImage& image, int32_t position_puu) {
  Bytes fb = makeK5100Input154(position_puu, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;  // active + ready + HomedStatus
  image.setFeedback(fb);
  // enable(): ServoLow + ServoSettle; HomedStatus skips Home34 unlock.
  for (int i = 0; i < 50; ++i) {
    axis.update();
  }
}

static void test_linear_x_belt_move_command(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(12.5f, 50.0f, 0.0f, 0.0f));

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_UINT32(eip::k5100::kOutput104Size, cmd.size());
  TEST_ASSERT_EQUAL_INT32(12500, readLeI32(cmd, 16));
  // Preload: OperatingMode=Position, TravelMode=2, Absolute, StartMotion low.
  TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
  TEST_ASSERT_EQUAL_UINT8(2, cmd[26]);
  TEST_ASSERT_EQUAL_UINT8(0, cmd[24]);  // NonCyclicMoveType Absolute
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 4; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_UINT8(2, cmd[26]);
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);

  for (int i = 0; i < 5; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_UINT8(2, cmd[26]);
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);
}

static void test_linear_move_stays_busy_until_target(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(12.5f, 50.0f, 0.0f, 0.0f));

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 30; ++i) {
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "busy cleared before reaching target");
  }

  // AtReference + in band completes without host StopMotion.
  Bytes at_target = makeK5100Input154(12500, false, false, true);
  at_target[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(at_target);
  axis.update();
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "should clear busy on AtReference");

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE_MESSAGE((cmd[1] & 0x04) == 0,
                           "Position Absolute must not StopMotion on arrival");
}

static void test_linear_soft_home_joint_frame(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  Bytes abs_pos = makeK5100Input154(500000, false, false, false);
  abs_pos[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(abs_pos);
  armAxisForMove(axis, image, 500000);

  axis.setCurrentPulses(0);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, axis.getCurrentMm());
  TEST_ASSERT_EQUAL_INT32(0, axis.getEncoderPulses());

  TEST_ASSERT_TRUE(axis.moveToMm(10.0f, 50.0f, 0.0f, 0.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(500000 + 10000, readLeI32(cmd, 16));
  TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
}

static void test_linear_absolute_keeps_target_on_overshoot_feedback(void) {
  // Drive profiles the stop; host must not flip signed Speed jog refs.
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 52428.8;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(1.0f, 50.0f, 0.0f, 0.0f));

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 12; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes overshoot = makeK5100Input154(52429 + 100000, false, false, false);
  overshoot[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(overshoot);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(52429, readLeI32(cmd, 16));  // still Absolute target
  TEST_ASSERT_TRUE(readLeI32(cmd, 4) > 0);              // cruise magnitude
  TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
}

static void test_linear_move_does_not_stop_at_half_travel(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 30.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(150.0f, 200.0f, 3000.0f, 3000.0f));

  Bytes mid = makeK5100Input154(75000, false, false, false);
  mid[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 20; ++i) {
    image.setFeedback(mid);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "busy cleared at half travel");
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_TRUE_MESSAGE((cmd[1] & 0x04) == 0,
                             "StopMotion must not assert at rem≈D/2");
    TEST_ASSERT_EQUAL_INT8(1, (int8_t)cmd[0]);
  }
}

static void test_linear_z_ballscrew_scaling(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 4000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(2.0f, 10.0f, 0.0f, 0.0f));

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(8000, readLeI32(cmd, 16));
  TEST_ASSERT_TRUE(axis.pulsesPerMm() > 3999.0 && axis.pulsesPerMm() < 4001.0);
}

static void test_linear_feedback_and_alarm(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 5500);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f, axis.getCurrentMm());
  // Sticky CommandInProgress alone must not look busy.
  Bytes cip = makeK5100Input154(5500, false, true, false);
  cip[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(cip);
  TEST_ASSERT_FALSE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.moveToMm(6.0f, 10.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  image.setFeedback(makeK5100Input154(5500, true, false, true));
  TEST_ASSERT_TRUE(axis.isAlarmActive());
}

static void test_kinetix_a603_warning_decode(void) {
  char disp[8];
  eip::k5100::DriveCodeInfo info{};
  TEST_ASSERT_TRUE(
      eip::k5100::lookupDriveCode(0x0603, 'A', disp, sizeof(disp), info));
  TEST_ASSERT_EQUAL_STRING("A603", info.display);
  TEST_ASSERT_NOT_NULL(strstr(info.name, "Invalid I/O"));

  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeK5100Input154(0, false, false, true, 0, 0x0603, true));
  TEST_ASSERT_FALSE(axis.isAlarmActive());  // warnings do not hard-fault the axis
  TEST_ASSERT_EQUAL_UINT16(0x0603, axis.getDriveWarningCode());

  char summary[192];
  TEST_ASSERT_TRUE(axis.getDriveAlarmSummary(summary, sizeof(summary)));
  TEST_ASSERT_NOT_NULL(strstr(summary, "A603"));
}

static void test_rotary_move_and_feedback(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_UINT32(eip::hcs01::kOutput101Size, cmd.size());
  TEST_ASSERT_EQUAL_INT32(4500, readLeI32(cmd, 2));

  image.setFeedback(makeHcs01Input102(4500, false, true));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 45.0f, axis.getCurrentDeg());
  TEST_ASSERT_FALSE(axis.isBusy());
}

static void test_rotary_alarm(void) {
  eip::EipProcessImage image;
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());

  image.setFeedback(makeHcs01Input102(0, true, true));
  TEST_ASSERT_TRUE(axis.isAlarmActive());
}

static void test_scanner_process_image_exchange(void) {
  FakeTcpClient tcp;
  FakeUdpEndpoint udp;
  udp.setEcho(true);
  eip::EipProcessImage image;

  const uint32_t handle = 0x44444444;
  tcp.enqueueResponse(makeRegisterSessionReply(handle));
  tcp.enqueueResponse(
      makeSendRRDataEncapReply(handle, makeForwardOpenMrReply(makeSampleForwardOpenReply())));

  eip::ScannerConfig scfg;
  scfg.target_ip = "192.168.1.100";
  eip::EipScanner scanner(tcp, udp, scfg);
  scanner.setProcessImage(&image);

  Gantry::EipLinearAxisConfig lcfg;
  lcfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, lcfg);
  TEST_ASSERT_TRUE(scanner.connect());
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  image.setOnline(true);
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(3.0f, 20.0f, 0.0f, 0.0f));

  Bytes sent_before;
  TEST_ASSERT_TRUE(image.getCommand(sent_before));

  Bytes io_out;
  TEST_ASSERT_TRUE(scanner.exchangeOnce(scanner.recvTimeoutMs(), io_out));
  TEST_ASSERT_TRUE(image.isOnline());
  TEST_ASSERT_TRUE(image.feedbackFresh());

  Bytes cmd_after;
  TEST_ASSERT_TRUE(image.getCommand(cmd_after));
  TEST_ASSERT_EQUAL_UINT32(sent_before.size(), cmd_after.size());

  scanner.disconnect();
  TEST_ASSERT_FALSE(image.isOnline());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_process_image_command_feedback);
  RUN_TEST(test_process_image_online);
  RUN_TEST(test_linear_x_belt_move_command);
  RUN_TEST(test_linear_move_stays_busy_until_target);
  RUN_TEST(test_linear_soft_home_joint_frame);
  RUN_TEST(test_linear_absolute_keeps_target_on_overshoot_feedback);
  RUN_TEST(test_linear_move_does_not_stop_at_half_travel);
  RUN_TEST(test_linear_z_ballscrew_scaling);
  RUN_TEST(test_linear_feedback_and_alarm);
  RUN_TEST(test_kinetix_a603_warning_decode);
  RUN_TEST(test_rotary_move_and_feedback);
  RUN_TEST(test_rotary_alarm);
  RUN_TEST(test_scanner_process_image_exchange);
  return UNITY_END();
}
