// Host unit tests for EipProcessImage, GantryEipLinearAxis, GantryEipRotaryAxis,
// and scanner/axis process-image integration.

#include "unity.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
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
#include "GantryThetaOrigin.h"
#include "GantryUtils.h"
#include "Hcs01Assembly.h"
#include "Kinetix5100Assembly.h"
#include "KinetixFaultCodes.h"
#include "axis_drivetrain_params.h"

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
  ssize_t sendTo(const uint8_t* data, size_t len, uint32_t, uint16_t) override {
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
                        bool command_reached, uint32_t diagnostic = 0) {
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
  actual.diagnostic_message = diagnostic;

  Bytes frame;
  eip::ByteWriter w(frame);
  w.u16(actual.status.encode());
  w.i32(actual.position_feedback);
  w.i32(actual.velocity_feedback);
  w.u32(actual.diagnostic_message);
  return frame;
}

void writeLeI32(Bytes& data, size_t offset, int32_t value) {
  const uint32_t u = static_cast<uint32_t>(value);
  data[offset] = static_cast<uint8_t>(u & 0xFF);
  data[offset + 1] = static_cast<uint8_t>((u >> 8) & 0xFF);
  data[offset + 2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  data[offset + 3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

int32_t readLeI32(const Bytes& data, size_t offset) {
  return static_cast<int32_t>(
      static_cast<uint32_t>(data[offset]) |
      (static_cast<uint32_t>(data[offset + 1]) << 8) |
      (static_cast<uint32_t>(data[offset + 2]) << 16) |
      (static_cast<uint32_t>(data[offset + 3]) << 24));
}

uint16_t readLeU16(const Bytes& data, size_t offset) {
  return static_cast<uint16_t>(data[offset] |
                               (static_cast<uint16_t>(data[offset + 1]) << 8));
}

}  // namespace

static void test_process_image_command_feedback(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
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
  
  const Bytes fb = {0xAA, 0xBB};
  image.setFeedback(fb);
  Bytes out_fb;
  
  // Offline: should return false even with data
  TEST_ASSERT_FALSE(image.getFeedback(out_fb));
  
  image.setOnline(true);
  TEST_ASSERT_TRUE(image.isOnline());
  // Online: should return true
  TEST_ASSERT_TRUE(image.getFeedback(out_fb));
  
  image.setOnline(false);
  TEST_ASSERT_FALSE(image.isOnline());
  // Offline again: should return false
  TEST_ASSERT_FALSE(image.getFeedback(out_fb));
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

// kPreload=4 + kStart=5 @ 100 Hz. Extra ticks keep tests off the phase edge.
static void pumpAbsoluteToRun(Gantry::GantryEipLinearAxis& axis,
                              eip::EipProcessImage& image, int32_t position_puu) {
  Bytes fb = makeK5100Input154(position_puu, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 12; ++i) {
    image.setFeedback(fb);
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

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);  // StartMotion edge after preload

  for (int i = 0; i < 5; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_UINT8(2, cmd[26]);
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);  // Run
}

static void test_linear_held_start_waits_for_release(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMmHeld(12.5f, 50.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isStartHeld());
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  Bytes cmd;
  for (int i = 0; i < 20; ++i) {
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE(axis.isStartHeld());
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);
  }

  TEST_ASSERT_TRUE(axis.releaseStartMotion());
  TEST_ASSERT_FALSE(axis.isStartHeld());
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
}

static int32_t cmdPositionPuu(const Bytes& cmd) {
  int32_t v = 0;
  std::memcpy(&v, &cmd[16], sizeof(v));  // Position reference, bytes 16..19 LE
  return v;
}

static void test_linear_queued_move_waits_for_run_and_edge(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  // Nothing in flight: nothing to queue behind.
  TEST_ASSERT_FALSE(axis.queueHeldMm(2.0f, 40.0f, 0.0f, 0.0f));

  TEST_ASSERT_TRUE(axis.moveToMmHeld(12.5f, 50.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.releaseStartMotion());
  TEST_ASSERT_TRUE(axis.queueHeldMm(2.0f, 40.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isStartHeld());
  TEST_ASSERT_FALSE(axis.queueHeldMm(3.0f, 40.0f, 0.0f, 0.0f));

  // The plunge edge is still high: the queued target is not on the wire yet.
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
  TEST_ASSERT_EQUAL_INT32(12500, cmdPositionPuu(cmd));

  // Moving toward 12.5 mm; once kStart ends the queued target goes out with SM=0.
  Bytes fb = makeK5100Input154(5000, false, false, false);
  fb[9] |= 0x04 | 0x08;
  for (int i = 0; i < 8; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);
  TEST_ASSERT_EQUAL_INT32(2000, cmdPositionPuu(cmd));
  TEST_ASSERT_TRUE(axis.isStartHeld());
  TEST_ASSERT_TRUE(axis.isBusy());

  // The edge waits until the target has been on the wire for 3 RPI.
  TEST_ASSERT_FALSE(axis.releaseStartMotion());
  std::this_thread::sleep_for(std::chrono::milliseconds(7));
  TEST_ASSERT_TRUE(axis.releaseStartMotion());
  TEST_ASSERT_FALSE(axis.isStartHeld());
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
  TEST_ASSERT_EQUAL_INT32(2000, cmdPositionPuu(cmd));
  TEST_ASSERT_TRUE(axis.isBusy());
}

static void test_linear_offline_aborts_move(void) {
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
  pumpAbsoluteToRun(axis, image, 0);
  TEST_ASSERT_TRUE(axis.isBusy());

  // Connection lost mid-move
  image.setOnline(false);
  axis.update(); // Should detect offline and abort immediately

  TEST_ASSERT_FALSE(axis.isBusy()); // Move phase reset to Idle
}

static void test_linear_second_absolute_preloads_before_start_edge(void) {
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
  pumpAbsoluteToRun(axis, image, 0);

  Bytes at = makeK5100Input154(12500, false, false, true);
  at[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 6; ++i) {
    image.setFeedback(at);
    axis.update();
    if (!axis.isBusy()) break;
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.moveToMm(5.0f, 50.0f, 0.0f, 0.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(5000, readLeI32(cmd, 16));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) == 0);

  Bytes fb = makeK5100Input154(12500, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE((cmd[1] & 0x10) != 0);
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

  // AtReference + in band must be stable for several ticks before Hold.
  Bytes at_target = makeK5100Input154(12500, false, false, true);
  at_target[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(at_target);
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "need multi-tick stable arrival");
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "need multi-tick stable arrival");
  axis.update();
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "should clear busy after stable AtReference");

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_TRUE_MESSAGE((cmd[1] & 0x04) == 0,
                           "Position Absolute must not StopMotion on arrival");
}

static void test_linear_arrival_requires_stable_ticks(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(5.0f, 50.0f, 0.0f, 0.0f));

  Bytes mid = makeK5100Input154(1000, false, false, false);
  mid[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(mid);
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes at = makeK5100Input154(5000, false, false, true);
  at[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  // Far-from-target feedback resets the stable arrival counter.
  Bytes glitch = makeK5100Input154(2000, false, false, false);
  glitch[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(glitch);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "counter reset by glitch");
  axis.update();
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "counter reset by glitch");
  axis.update();
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "clears after 3 stable ticks");
}
static void test_axis_debounces_command_value_reached_jitter(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(5.0f, 50.0f, 0.0f, 0.0f));

  Bytes mid = makeK5100Input154(1000, false, false, false);
  mid[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(mid);
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());

  Bytes at = makeK5100Input154(5000, false, false, true);
  at[9] |= 0x04 | 0x08 | 0x20;
  
  Bytes at_jitter = makeK5100Input154(4999, false, false, false); // No AtReference bit
  at_jitter[9] |= 0x04 | 0x08;
  at_jitter[16] = 100; // Inject some velocity so band_stopped is FALSE

  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  // Jitter! The bit drops, even though we are in position band
  image.setFeedback(at_jitter);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  // Wait again
  image.setFeedback(at);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());
  axis.update();
  TEST_ASSERT_FALSE(axis.isBusy());
}

static void test_axis_handles_drive_power_loss_while_network_online(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  
  TEST_ASSERT_TRUE(axis.moveToMm(10.0f, 50.0f, 0.0f, 0.0f));
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());

  // Simulate Drive Power Loss: Network stays online, but Class 1 reports a fault
  Bytes fault = makeK5100Input154(1000, true, false, false);
  image.setFeedback(fault);
  
  axis.update();
  // Axis enters kAbortStop state to command a stop, so it remains busy
  TEST_ASSERT_TRUE(axis.isBusy());
  TEST_ASSERT_TRUE(axis.isAlarmActive());
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

static void test_linear_move_rejects_drive_limit_stop(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);

  // Fault + Stopped (isLikelyLimitStop) — move must be rejected.
  Bytes limit = makeK5100Input154(0, true, false, false);
  limit[9] |= 0x04 | 0x40;  // active + stopped (ready clear via fault)
  image.setFeedback(limit);
  TEST_ASSERT_FALSE(axis.moveToMm(10.0f, 20.0f, 0.0f, 0.0f));
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

static void test_kinetix_a014_a015_helpers(void) {
  TEST_ASSERT_TRUE(eip::k5100::isWarningA014(0x0014));
  TEST_ASSERT_TRUE(eip::k5100::isWarningA015(0x0015));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA014(0x0015));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA015(0x0014));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA014(0x0603));

  char disp[8];
  eip::k5100::DriveCodeInfo info{};
  TEST_ASSERT_TRUE(
      eip::k5100::lookupDriveCode(0x0015, 'A', disp, sizeof(disp), info));
  TEST_ASSERT_EQUAL_STRING("A015", info.display);
  TEST_ASSERT_NOT_NULL(strstr(info.name, "Negative"));
  TEST_ASSERT_TRUE(
      eip::k5100::lookupDriveCode(0x0014, 'A', disp, sizeof(disp), info));
  TEST_ASSERT_EQUAL_STRING("A014", info.display);
  TEST_ASSERT_NOT_NULL(strstr(info.name, "Positive"));
}

static void test_kinetix_format_helpers_and_a603(void) {
  TEST_ASSERT_EQUAL_UINT16(0x0603, eip::k5100::codeDigits(0x1603));
  TEST_ASSERT_EQUAL_UINT16(0x0603, eip::k5100::codeDigits(0x0603));
  TEST_ASSERT_TRUE(eip::k5100::isWarningA603(0x0603));
  TEST_ASSERT_TRUE(eip::k5100::isWarningA603(0x1603));
  TEST_ASSERT_FALSE(eip::k5100::isWarningA603(0x0014));

  char disp[8];
  eip::k5100::formatDisplayCode(disp, sizeof(disp), 'A', 0x0603);
  TEST_ASSERT_EQUAL_STRING("A603", disp);
  eip::k5100::formatDisplayCode(disp, sizeof(disp), 'E', 0x0602);
  TEST_ASSERT_EQUAL_STRING("E602", disp);

  char summary[192];
  TEST_ASSERT_FALSE(eip::k5100::formatDriveTripSummary(summary, sizeof(summary),
                                                       false, false, false, 0, 0));
  TEST_ASSERT_EQUAL_STRING("clear", summary);

  TEST_ASSERT_TRUE(eip::k5100::formatDriveTripSummary(
      summary, sizeof(summary), false, true, false, 0, 0x0603));
  TEST_ASSERT_NOT_NULL(strstr(summary, "A603"));
  TEST_ASSERT_NOT_NULL(strstr(summary, "WARN"));

  TEST_ASSERT_TRUE(eip::k5100::formatDriveTripSummary(
      summary, sizeof(summary), false, false, true, 0, 0));
  TEST_ASSERT_NOT_NULL(strstr(summary, "ConnectionFaulted"));
}

static void test_linear_absolute_aborts_busy_on_a015(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(50.0f, 20.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  pumpAbsoluteToRun(axis, image, 0);
  TEST_ASSERT_TRUE(axis.isBusy());

  // A015 warning during Run must abort and clear busy (escape moves still OK).
  Bytes a015 = makeK5100Input154(1000, false, false, false, 0, 0x0015, true);
  a015[9] |= 0x04 | 0x08 | 0x20 | 0x40;  // active+ready+CIP + stopped
  image.setFeedback(a015);
  for (int i = 0; i < 40; ++i) {
    image.setFeedback(a015);
    axis.update();
    if (!axis.isBusy()) break;
  }
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "A015 should clear Absolute busy");

  // Escape move must not be rejected solely for A015 warning.
  TEST_ASSERT_TRUE(axis.moveToMm(0.0f, 20.0f, 0.0f, 0.0f));

  // While still on A015, Absolute must not re-abort (creep/escape path).
  Bytes a015_run = makeK5100Input154(1000, false, false, false, 0, 0x0015, true);
  a015_run[9] |= 0x04 | 0x08 | 0x20;
  for (int i = 0; i < 12; ++i) {
    image.setFeedback(a015_run);
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());
  for (int i = 0; i < 10; ++i) {
    image.setFeedback(a015_run);
    axis.update();
  }
  TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                           "escape while A015 still set must not re-abort");
}

static void test_stop_motion_clears_busy_when_position_stable(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(100.0f, 20.0f, 0.0f, 0.0f));

  Bytes run_fb = makeK5100Input154(25000, false, false, false);
  run_fb[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(run_fb);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.stopMotion());
  TEST_ASSERT_TRUE(axis.isBusy());

  // Held position + stopped bit: kStopping clears busy after stable ticks.
  Bytes held = makeK5100Input154(25000, false, false, false);
  held[9] |= 0x04 | 0x08 | 0x20 | 0x40;
  for (int i = 0; i < 40; ++i) {
    image.setFeedback(held);
    axis.update();
    if (!axis.isBusy()) break;
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT8(0, static_cast<int8_t>(cmd[0]));  // OM=NotSpecified
  TEST_ASSERT_EQUAL_UINT8(10, cmd[26]);                     // TM=10
  TEST_ASSERT_TRUE((cmd[1] & 0x01) != 0);                   // servo_on
  TEST_ASSERT_TRUE((cmd[1] & 0x04) == 0);                   // StopMotion cleared
}

// Bench regression: during a 1 mm/s creep the drive asserts its `stopped` bit
// and reports near-zero speed. kStopping must keep isBusy() until position
// actually holds still.
static void test_stop_motion_stays_busy_while_position_creeps(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(600.0f, 1.0f, 0.0f, 0.0f));

  Bytes run_fb = makeK5100Input154(0, false, false, false);
  run_fb[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(run_fb);
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.stopMotion());

  // 10 PUU/tick ≈ 1 mm/s — same as the bench creep. Drive claims stopped.
  int32_t puu = 0;
  for (int i = 0; i < 50; ++i) {
    Bytes fb = makeK5100Input154(puu, false, false, false);
    fb[9] |= 0x04 | 0x08 | 0x20 | 0x40;
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                             "creeping feedback must keep kStopping busy");
    puu += 10;
  }
}

// Bench regression: the drive A603s and ignores the abort if it changes anything
// beyond the StopMotion bit. The settle image (OM=0 TM=10, zero decel) swapped
// mode mid-profile and asked for a decel-to-stop with no decel ramp.
// 1 s stop cap: position still changing and speed still high. Busy must
// survive the 200 ms stable window and clear on the 100th 100 Hz tick.
static void test_stop_motion_times_out_while_still_moving(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);
  TEST_ASSERT_TRUE(axis.moveToMm(400.0f, 80.0f, 2000.0f, 100.0f));
  pumpAbsoluteToRun(axis, image, 0);
  TEST_ASSERT_TRUE(axis.stopMotion());

  int32_t puu = 0;
  for (int i = 0; i < 99; ++i) {
    Bytes fb = makeK5100Input154(puu, false, false, false);
    fb[9] |= 0x04 | 0x08 | 0x20;  // active + ready + homed, not stopped
    writeLeI32(fb, 16, 100);      // 10.0 RPM, above the near-stop gate
    image.setFeedback(fb);
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                             "StopMotion must stay busy until the 1 s cap");
    puu += 20;  // 0.02 mm/tick, above the 0.005 mm stable gate
  }
  Bytes fb = makeK5100Input154(puu, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  writeLeI32(fb, 16, 100);
  image.setFeedback(fb);
  axis.update();
  TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(),
                            "100th tick must force hold while still moving");
}

// Servo-on with Active stuck at 0. The 400 ms settle must expire into hold
// and must not issue method 34.
static void test_servo_arm_times_out_when_active_stays_clear(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  Bytes fb = makeK5100Input154(5000, false, false, false);
  fb[9] |= 0x08;  // ready, Active clear
  image.setFeedback(fb);
  TEST_ASSERT_TRUE(axis.enable());

  for (int i = 0; i < 10; ++i) {
    image.setFeedback(fb);
    axis.update();
  }
  TEST_ASSERT_FALSE_MESSAGE(axis.moveToMm(6.0f, 10.0f, 100.0f, 100.0f),
                            "move must wait out the servo-arm settle");

  bool saw_home = false;
  for (int i = 0; i < 50; ++i) {
    image.setFeedback(fb);
    axis.update();
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    if (cmd[0] == 3) saw_home = true;
  }
  TEST_ASSERT_FALSE(saw_home);
  // Holding copies feedback into the command target. Settle does not.
  Bytes later = makeK5100Input154(8000, false, false, false);
  later[9] |= 0x08;
  image.setFeedback(later);
  axis.update();
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 8.0f, axis.getTargetMm(),
                                   "settle cap must leave the axis holding");
}

// HomedStatus stays 0, so arm issues method 34 and the 0.5 s wait expires
// without redefining joint zero.
static void test_home34_times_out_when_homed_stays_clear(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  Bytes fb = makeK5100Input154(5000, false, false, false);
  fb[9] |= 0x04 | 0x08;  // active + ready, HomedStatus clear
  image.setFeedback(fb);
  TEST_ASSERT_TRUE(axis.enable());

  bool saw_method_34 = false;
  bool left_home = false;
  for (int i = 0; i < 80; ++i) {
    image.setFeedback(fb);
    axis.update();
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    if (cmd[0] == 3 && cmd[3] == 34) saw_method_34 = true;
    if (saw_method_34 && cmd[0] == 0) left_home = true;
  }
  TEST_ASSERT_TRUE_MESSAGE(saw_method_34, "unhomed arm must issue method 34");
  TEST_ASSERT_TRUE_MESSAGE(left_home, "Home34 wait must return to settle");
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 5.0f, axis.getTargetMm(),
                                   "timeout must not zero the joint");
}

static void test_move_while_disabled_does_not_arm_motion(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  Bytes fb = makeK5100Input154(0, false, false, false);
  fb[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(fb);
  TEST_ASSERT_FALSE(axis.moveToMm(10.0f, 20.0f, 100.0f, 100.0f));
  TEST_ASSERT_FALSE(axis.isBusy());
  for (int i = 0; i < 80; ++i) {
    image.setFeedback(fb);
    axis.update();
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_NOT_EQUAL(3, cmd[0]);
  }
  TEST_ASSERT_FALSE(axis.isBusy());
}

static void test_abort_image_differs_from_move_only_by_stop_bit(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipLinearAxisConfig cfg;
  cfg.puu_per_mm = 1000.0;
  cfg.speed_ref_per_mm_s = 15.0;
  Gantry::GantryEipLinearAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  armAxisForMove(axis, image, 0);

  TEST_ASSERT_TRUE(axis.moveToMm(600.0f, 1.0f, 0.0f, 0.0f));

  // Settle into Run and capture the accepted in-flight image.
  pumpAbsoluteToRun(axis, image, 1000);
  TEST_ASSERT_TRUE(axis.isBusy());
  Bytes run_cmd;
  TEST_ASSERT_TRUE(image.getCommand(run_cmd));
  TEST_ASSERT_TRUE((run_cmd[1] & 0x04) == 0);  // StopMotion clear while running

  // Rising A014 aborts mid-profile — the case that ran forever on the bench.
  Bytes a014 = makeK5100Input154(1100, false, false, false, 0, 0x0014, true);
  a014[9] |= 0x04 | 0x08 | 0x20;
  image.setFeedback(a014);
  axis.update();

  Bytes stop_cmd;
  TEST_ASSERT_TRUE(image.getCommand(stop_cmd));
  TEST_ASSERT_EQUAL_UINT32(run_cmd.size(), stop_cmd.size());
  TEST_ASSERT_TRUE_MESSAGE((stop_cmd[1] & 0x04) != 0, "abort must set StopMotion");
  for (size_t i = 0; i < run_cmd.size(); ++i) {
    if (i == 1) continue;  // control bits carry the StopMotion edge
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(
        run_cmd[i], stop_cmd[i],
        "abort image must not change OM/TM/speed/accel/decel mid-profile");
  }
}

static void test_rotary_enable_holds_feedback_and_keeps_halt(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(9000, false, true));
  TEST_ASSERT_TRUE(axis.enable());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw0 = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x0020, cw0);  // C0500 / clear_errors
  TEST_ASSERT_BITS_HIGH(0x0002, cw0);  // stay OM
  TEST_ASSERT_BITS_LOW(0x8000, cw0);   // Drive ON after fault-clear
  TEST_ASSERT_EQUAL_INT32(9000, readLeI32(cmd, 2));
  TEST_ASSERT_FALSE(axis.isBusy());

  for (int i = 0; i < 12; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x8000, cw);  // Drive ON
  TEST_ASSERT_BITS_HIGH(0x4000, cw);  // Drive Enable
  TEST_ASSERT_BITS_LOW(0x2000, cw);   // Halt active — do not start
  TEST_ASSERT_BITS_LOW(0x0020, cw);   // clear_errors released

  TEST_ASSERT_TRUE(axis.disable());
  Bytes off;
  TEST_ASSERT_TRUE(image.getCommand(off));
  TEST_ASSERT_EQUAL_INT32(9000, readLeI32(off, 2));
  const uint16_t off_cw = readLeU16(off, 0);
  TEST_ASSERT_BITS_HIGH(0x0002, off_cw);  // OM bit1 — must not enter A0050
  TEST_ASSERT_BITS_LOW(0xA000, off_cw);   // Drive ON + Halt off
}

static void test_rotary_arst_while_disabled_stays_drive_off(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(1785000, true, true, 0x4009u));
  TEST_ASSERT_TRUE(axis.clearAlarm());
  TEST_ASSERT_FALSE(axis.isEnabled());
  for (int i = 0; i < 12; ++i) {
    axis.update();
  }
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x0002, cw);  // OM
  TEST_ASSERT_BITS_LOW(0x8000, cw);   // Drive ON must stay 0
  TEST_ASSERT_BITS_LOW(0x0020, cw);   // C0500 pulse released
}

static void test_rotary_move_and_feedback(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(0, false, false));
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));

  Bytes preload;
  TEST_ASSERT_TRUE(image.getCommand(preload));
  TEST_ASSERT_EQUAL_UINT32(eip::hcs01::kOutput101Size, preload.size());
  TEST_ASSERT_EQUAL_INT32(4500, readLeI32(preload, 2));
  TEST_ASSERT_BITS_LOW(0x2000, readLeU16(preload, 0));
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 16; ++i) {
    axis.update();
  }
  Bytes started;
  TEST_ASSERT_TRUE(image.getCommand(started));
  TEST_ASSERT_EQUAL_INT32(4500, readLeI32(started, 2));
  TEST_ASSERT_BITS_HIGH(0x2000, readLeU16(started, 0));
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 4; ++i) {
    image.setFeedback(makeHcs01Input102(4500, false, true));
    axis.update();
  }
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 45.0f, axis.getCurrentDeg());
  TEST_ASSERT_FALSE(axis.isBusy());
}

// Class 1 stays up, ready never reaches 3. After the 1 s AH cap the hold
// command stops tracking feedback.
static void test_theta_ah_times_out_when_ready_stays_below_operation(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());

  auto frame = [](int32_t puu) {
    eip::hcs01::Hcs01StatusWord st;
    st.ready = eip::hcs01::ReadyForOperation::kReadyAb;
    st.operating_mode_ack = eip::hcs01::OperatingModeAck::kOperatingMode;
    eip::hcs01::Hcs01PositioningActual actual;
    actual.status = st;
    actual.position_feedback = puu;
    Bytes out;
    eip::ByteWriter w(out);
    w.u16(actual.status.encode());
    w.i32(actual.position_feedback);
    w.i32(0);
    w.u32(0);
    return out;
  };

  image.setFeedback(frame(1000));
  TEST_ASSERT_TRUE(axis.enable());
  for (int i = 0; i < 160; ++i) {
    image.setFeedback(frame(1000));
    axis.update();
  }
  Bytes held;
  TEST_ASSERT_TRUE(image.getCommand(held));
  TEST_ASSERT_EQUAL_INT32(1000, readLeI32(held, 2));

  for (int i = 0; i < 5; ++i) {
    image.setFeedback(frame(8000));
    axis.update();
  }
  Bytes after;
  TEST_ASSERT_TRUE(image.getCommand(after));
  TEST_ASSERT_EQUAL_INT32_MESSAGE(1000, readLeI32(after, 2),
                                  "AH cap must stop tracking feedback");
}

static void test_rotary_offline_aborts_move(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 4; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());

  image.setOnline(false);
  axis.update();
  TEST_ASSERT_FALSE(axis.isBusy());
}
static void test_rotary_consecutive_move_ignores_stale_in_pos(void) {
  // Live test_cycle I: H leaves bit4=1 at ~179°, then 179→0 must stay busy
  // until the new target, not halt at the previous pose.
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));
  for (int i = 0; i < 16; ++i) {
    axis.update();
  }
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(makeHcs01Input102(4500, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 45.0f, axis.getCurrentDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(0.0f, 3600.0f, 18000.0f, 18000.0f));
  TEST_ASSERT_TRUE(axis.isBusy());
  for (int i = 0; i < 16; ++i) {
    image.setFeedback(makeHcs01Input102(4500, false, true));
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(),
                             "stale in-pos at previous target cleared busy");
  }
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 45.0f, axis.getCurrentDeg());

  image.setFeedback(makeHcs01Input102(2000, false, false));
  axis.update();
  TEST_ASSERT_TRUE(axis.isBusy());

  for (int i = 0; i < 16; ++i) {
    image.setFeedback(makeHcs01Input102(0, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, axis.getCurrentDeg());
}

static void test_rotary_unaligned_home_offsets_and_clamps_to_drive_travel(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeHcs01Input102(1785127, false, true));
  TEST_ASSERT_TRUE(axis.captureSoftHome());
  TEST_ASSERT_TRUE(axis.isDriveOriginAligned());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 178.5127f, axis.getDriveAbsDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 178.5127f, axis.getCurrentDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 36000.0f, axis.getMaxDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -36000.0f, axis.getMinDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(180.0f, 30.0f, 180.0f, 180.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const int32_t cmd_puu = readLeI32(cmd, 2);
  TEST_ASSERT_EQUAL_INT32(1800000, cmd_puu);
}

static void test_rotary_aligned_home_is_drive_identity(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.captureSoftHome());
  TEST_ASSERT_TRUE(axis.isDriveOriginAligned());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, axis.getDriveAbsDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, axis.getCurrentDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 36000.0f, axis.getMaxDeg());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -36000.0f, axis.getMinDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(180.0f, 30.0f, 180.0f, 180.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const int32_t cmd_puu = readLeI32(cmd, 2);
  TEST_ASSERT_EQUAL_INT32(1800000, cmd_puu);
}

static void test_rotary_unaligned_small_delta_keeps_offset_command(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  image.setFeedback(makeHcs01Input102(9000, false, true));
  TEST_ASSERT_TRUE(axis.hasLiveFeedback());
  TEST_ASSERT_TRUE(axis.captureSoftHome());
  TEST_ASSERT_TRUE(axis.isDriveOriginAligned());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, axis.getCurrentDeg());

  TEST_ASSERT_TRUE(axis.moveToDeg(10.0f, 30.0f, 0.0f, 0.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(1000, readLeI32(cmd, 2));
}

static void test_rotary_set_puu_per_deg(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_FALSE(axis.setPuuPerDeg(0.0));
  TEST_ASSERT_FALSE(axis.setPuuPerDeg(-1.0));
  TEST_ASSERT_TRUE(axis.setPuuPerDeg(200.0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 200.0f, static_cast<float>(axis.pulsesPerDeg()));
  TEST_ASSERT_TRUE(axis.moveToDeg(10.0f, 5.0f, 20.0f, 30.0f));
  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  TEST_ASSERT_EQUAL_INT32(2000, readLeI32(cmd, 2));
  // 5 deg/s; velocity does not scale with PUU/deg (S-0-0259 is 0.0001 rpm).
  TEST_ASSERT_EQUAL_INT32(8333, readLeI32(cmd, 6));
  TEST_ASSERT_EQUAL_INT32(349, readLeI32(cmd, 10));
  TEST_ASSERT_EQUAL_INT32(524, readLeI32(cmd, 14));
}

static void test_hcs01_units_match_drive_scaling(void) {
    // S-0-0259 per the drive export (S-0-0044 = 2, S-0-0046 = -4): 0.0001 rpm.
    // 15 deg/s = 2.5 rpm. The old 0.0001 deg/min reading sent 9,000,000
    // (900 rpm, past the 600 rpm motor max: E2049, speed ignored).
    TEST_ASSERT_EQUAL_INT32(25000, Gantry::hcs01VelocityUnits(15.0));
    TEST_ASSERT_EQUAL_INT32(100000, Gantry::hcs01VelocityUnits(60.0));
    // Capped at S-0-0113, 600 rpm = 3600 deg/s.
    TEST_ASSERT_EQUAL_INT32(6000000, Gantry::hcs01VelocityUnits(3600.0));
    TEST_ASSERT_EQUAL_INT32(6000000, Gantry::hcs01VelocityUnits(10000.0));
    TEST_ASSERT_EQUAL_INT32(0, Gantry::hcs01VelocityUnits(-5.0));
    // S-0-0260 / S-0-0359: 0.001 rad/s2, no PUU factor.
    TEST_ASSERT_EQUAL_INT32(31416, Gantry::hcs01AccelUnits(1800.0));
    TEST_ASSERT_EQUAL_INT32(314159, Gantry::hcs01AccelUnits(18000.0));
    TEST_ASSERT_EQUAL_INT32(1000000, Gantry::hcs01AccelUnits(1.0e6));
}

// Home / bring-up orient pass accel 0 and fall back to the config defaults.
// Those must reach the drive in the same units as an explicit accel
// (was AXIS_THETA_ACCEL * PUU/deg = 180,000,000: F2028 after ~40 deg).
static void test_rotary_default_accel_matches_explicit_accel(void) {
    constexpr double kPuuPerDeg = 10000.0;
    constexpr float kAccelDegS2 = 18000.0f;
    eip::EipProcessImage image;
    image.setOnline(true);
    // Same factory main.cpp uses for the live theta axis.
    const Gantry::EipRotaryAxisConfig cfg =
        Gantry::eipRotaryAxisConfigFromDeg(kPuuPerDeg, 3600.0, kAccelDegS2, kAccelDegS2);
    Gantry::GantryEipRotaryAxis axis(image, cfg);
    TEST_ASSERT_TRUE(axis.begin());
    image.setFeedback(makeHcs01Input102(0, false, false));
    TEST_ASSERT_TRUE(axis.enable());

    Bytes cmd;
    TEST_ASSERT_TRUE(axis.moveToDeg(90.0f, 15.0f, kAccelDegS2, kAccelDegS2));
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    const int32_t explicit_acc = readLeI32(cmd, 10);
    const int32_t explicit_dec = readLeI32(cmd, 14);
    TEST_ASSERT_TRUE(axis.stopMotion());

    TEST_ASSERT_TRUE(axis.moveToDeg(90.0f, 15.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    TEST_ASSERT_EQUAL_INT32(25000, readLeI32(cmd, 6));
    TEST_ASSERT_EQUAL_INT32_MESSAGE(explicit_acc, readLeI32(cmd, 10),
                                    "default accel must use drive units");
    TEST_ASSERT_EQUAL_INT32_MESSAGE(explicit_dec, readLeI32(cmd, 14),
                                    "default decel must use drive units");
}

static void test_theta_move_timeout_scales_with_distance(void) {
    TEST_ASSERT_EQUAL_UINT32(Gantry::kThetaMoveTimeoutMinUs,
                             Gantry::thetaMoveTimeoutUs(10.0, 100.0));
    TEST_ASSERT_EQUAL_UINT32(Gantry::kThetaMoveTimeoutMinUs,
                             Gantry::thetaMoveTimeoutUs(300.0, 0.0));
    // 300 deg at the 15 deg/s home speed takes 20 s; allow twice that.
    TEST_ASSERT_EQUAL_UINT32(40000000u, Gantry::thetaMoveTimeoutUs(300.0, 15.0));
    TEST_ASSERT_EQUAL_UINT32(40000000u, Gantry::thetaMoveTimeoutUs(-300.0, 15.0));
}

// A slow move longer than 15 s must not time out at 15 s (home t from far
// away at 15 deg/s), but must still time out once it overruns.
static void test_rotary_long_slow_move_outlives_fixed_timeout(void) {
    eip::EipProcessImage image;
    image.setOnline(true);
    Gantry::EipRotaryAxisConfig cfg;
    cfg.puu_per_deg = 100.0;
    Gantry::GantryEipRotaryAxis axis(image, cfg);
    TEST_ASSERT_TRUE(axis.begin());
    image.setFeedback(makeHcs01Input102(0, false, false));
    TEST_ASSERT_TRUE(axis.enable());
    // 170 deg at 5 deg/s: 34 s expected, 68 s timeout. 100 Hz updates.
    TEST_ASSERT_TRUE(axis.moveToDeg(170.0f, 5.0f, 0.0f, 0.0f));
    for (int i = 0; i < 1700; ++i) {
        image.setFeedback(makeHcs01Input102(1000, false, false));
        axis.update();
    }
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "17 s into a 34 s move must still be busy");
    for (int i = 0; i < 5500; ++i) {
        image.setFeedback(makeHcs01Input102(1000, false, false));
        axis.update();
    }
    TEST_ASSERT_FALSE_MESSAGE(axis.isBusy(), "move must time out after 68 s");
}

static void test_rotary_halt_clears_start_bit(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());
  TEST_ASSERT_FALSE(axis.isBusy());
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 30.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(axis.isBusy());

  TEST_ASSERT_TRUE(axis.stopMotion());
  TEST_ASSERT_FALSE(axis.isBusy());

  Bytes cmd;
  TEST_ASSERT_TRUE(image.getCommand(cmd));
  const uint16_t cw = readLeU16(cmd, 0);
  TEST_ASSERT_BITS_HIGH(0x8000, cw);
  TEST_ASSERT_BITS_LOW(0x2000, cw);
}

static void test_rotary_alarm(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 100.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());

  image.setFeedback(makeHcs01Input102(0, true, true));
  TEST_ASSERT_TRUE(axis.isAlarmActive());
}

static void test_rotary_thetalim_cannot_expand_past_offset_envelope(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  image.setFeedback(makeHcs01Input102(1785127, false, true));
  TEST_ASSERT_TRUE(axis.captureSoftHome());

  axis.setAngleRange(-180.0f, 180.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -180.0f, axis.getMinDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 180.0f, axis.getMaxDeg());

  axis.setAngleRange(-10.0f, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -10.0f, axis.getMinDeg());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.5f, axis.getMaxDeg());
}

static void test_rotary_alarm_summary_and_cip_status(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());

  char buf[128] = {};
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.getDriveAlarmSummary(buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING("clear", buf);
  TEST_ASSERT_TRUE(axis.formatCipStatus(buf, sizeof(buf)));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "T->O live"));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "c1err=0"));

  image.setFeedback(makeHcs01Input102(0, true, true, 0x2057u));
  TEST_ASSERT_TRUE(axis.getDriveAlarmSummary(buf, sizeof(buf)));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "F2057"));
  TEST_ASSERT_NOT_NULL(std::strstr(buf, "c1err=1"));
}

static void test_axis_log_due_period(void) {
  int64_t last = 0;
  TEST_ASSERT_FALSE(Gantry::axisLogDue(0, 1000, &last));
  TEST_ASSERT_TRUE(Gantry::axisLogDue(10, 1000, &last));
  TEST_ASSERT_EQUAL(1000, static_cast<int>(last));
  TEST_ASSERT_FALSE(Gantry::axisLogDue(10, 50000, &last));
  TEST_ASSERT_TRUE(Gantry::axisLogDue(10, 101000, &last));
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

static void test_rotary_settles_within_hcs01_window_and_sub_window_move(void) {
  eip::EipProcessImage image;
  image.setOnline(true);
  Gantry::EipRotaryAxisConfig cfg;
  cfg.puu_per_deg = 10000.0;
  Gantry::GantryEipRotaryAxis axis(image, cfg);
  TEST_ASSERT_TRUE(axis.begin());
  TEST_ASSERT_TRUE(axis.enable());

  // 1. Move 0 -> 45 deg: settle cleanly once drive reports reached=1 at 44.94 deg
  //    (within HCS01 S-0-0057 = 0.1 deg window, no stall waiting for 0.01 deg).
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(45.0f, 1440.0f, 7200.0f, 7200.0f));
  for (int i = 0; i < 18; ++i) {
    axis.update();
  }
  image.setFeedback(makeHcs01Input102(200000, false, false));
  axis.update();
  for (int i = 0; i < 4; ++i) {
    image.setFeedback(makeHcs01Input102(449400, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  // 2. Sub-window move (89.994 -> 90.000 deg): reached stays 1, must finish
  //    promptly instead of hanging until the 15 s timeout.
  image.setFeedback(makeHcs01Input102(899940, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(90.0f, 30.0f, 1800.0f, 1800.0f));
  for (int i = 0; i < 24; ++i) {
    image.setFeedback(makeHcs01Input102(899940, false, true));
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());

  // 3. Stiction standstill at 0.15 deg short of target (reached=0, standstill=1):
  //    stays busy at 0.60 deg away AND while still creeping at 30 deg/s (0.30 deg/tick),
  //    then settles cleanly once truly stationary inside 0.30 deg for 50 ms.
  auto makeStandstill = [](int32_t puu) {
    Bytes frame;
    eip::ByteWriter w(frame);
    w.u16(0xC80Eu);  // ready=3 AF, in_standstill=1, command_value_reached=0
    w.i32(puu);
    w.i32(0);
    w.u32(0);
    return frame;
  };
  image.setFeedback(makeHcs01Input102(0, false, true));
  TEST_ASSERT_TRUE(axis.moveToDeg(-45.0f, 1000.0f, 3600.0f, 3600.0f));
  for (int i = 0; i < 18; ++i) {
    axis.update();
  }
  // Stationary at -44.40 deg (0.60 deg away): stays busy.
  image.setFeedback(makeStandstill(-444000));
  for (int i = 0; i < 8; ++i) {
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());
  // Passing through [-44.75 .. -45.25] at 0.10 deg/tick (slow bring-up cruise): stays busy.
  for (int i = 0; i < 6; ++i) {
    image.setFeedback(makeStandstill(-447500 - i * 1000));
    axis.update();
  }
  TEST_ASSERT_TRUE(axis.isBusy());
  // Stationary in stiction at -44.85 deg (0.15 deg away): settles after 5 stable ticks.
  image.setFeedback(makeStandstill(-448500));
  for (int i = 0; i < 6; ++i) {
    axis.update();
  }
  TEST_ASSERT_FALSE(axis.isBusy());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_process_image_command_feedback);
  RUN_TEST(test_process_image_online);
  RUN_TEST(test_linear_x_belt_move_command);
  RUN_TEST(test_linear_held_start_waits_for_release);
  RUN_TEST(test_linear_queued_move_waits_for_run_and_edge);
  RUN_TEST(test_linear_offline_aborts_move);
  RUN_TEST(test_linear_second_absolute_preloads_before_start_edge);
  RUN_TEST(test_linear_move_stays_busy_until_target);
  RUN_TEST(test_linear_arrival_requires_stable_ticks);
  RUN_TEST(test_axis_debounces_command_value_reached_jitter);
  RUN_TEST(test_axis_handles_drive_power_loss_while_network_online);
  RUN_TEST(test_linear_soft_home_joint_frame);
  RUN_TEST(test_linear_absolute_keeps_target_on_overshoot_feedback);
  RUN_TEST(test_linear_move_does_not_stop_at_half_travel);
  RUN_TEST(test_linear_z_ballscrew_scaling);
  RUN_TEST(test_linear_move_rejects_drive_limit_stop);
  RUN_TEST(test_linear_feedback_and_alarm);
  RUN_TEST(test_kinetix_a603_warning_decode);
  RUN_TEST(test_kinetix_a014_a015_helpers);
  RUN_TEST(test_kinetix_format_helpers_and_a603);
  RUN_TEST(test_linear_absolute_aborts_busy_on_a015);
  RUN_TEST(test_stop_motion_clears_busy_when_position_stable);
  RUN_TEST(test_stop_motion_stays_busy_while_position_creeps);
  RUN_TEST(test_stop_motion_times_out_while_still_moving);
  RUN_TEST(test_servo_arm_times_out_when_active_stays_clear);
  RUN_TEST(test_home34_times_out_when_homed_stays_clear);
  RUN_TEST(test_move_while_disabled_does_not_arm_motion);
  RUN_TEST(test_abort_image_differs_from_move_only_by_stop_bit);
  RUN_TEST(test_rotary_enable_holds_feedback_and_keeps_halt);
  RUN_TEST(test_rotary_arst_while_disabled_stays_drive_off);
  RUN_TEST(test_rotary_move_and_feedback);
  RUN_TEST(test_hcs01_units_match_drive_scaling);
  RUN_TEST(test_rotary_default_accel_matches_explicit_accel);
  RUN_TEST(test_theta_move_timeout_scales_with_distance);
  RUN_TEST(test_rotary_long_slow_move_outlives_fixed_timeout);
  RUN_TEST(test_theta_ah_times_out_when_ready_stays_below_operation);
  RUN_TEST(test_rotary_offline_aborts_move);
  RUN_TEST(test_rotary_consecutive_move_ignores_stale_in_pos);
  RUN_TEST(test_rotary_settles_within_hcs01_window_and_sub_window_move);
  RUN_TEST(test_rotary_unaligned_home_offsets_and_clamps_to_drive_travel);
  RUN_TEST(test_rotary_aligned_home_is_drive_identity);
  RUN_TEST(test_rotary_unaligned_small_delta_keeps_offset_command);
  RUN_TEST(test_rotary_set_puu_per_deg);
  RUN_TEST(test_rotary_halt_clears_start_bit);
  RUN_TEST(test_rotary_alarm);
  RUN_TEST(test_rotary_thetalim_cannot_expand_past_offset_envelope);
  RUN_TEST(test_rotary_alarm_summary_and_cip_status);
  RUN_TEST(test_axis_log_due_period);
  RUN_TEST(test_scanner_process_image_exchange);
  return UNITY_END();
}
