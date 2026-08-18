// Host tests for ScannerConfig builders and W5500 recover backoff.

#include "unity.h"

#include "EipScannerPolicy.h"

void setUp(void) {}
void tearDown(void) {}

using eip::ChipRecoverDecision;
using eip::ConnectionType;
using eip::KinetixScannerArgs;
using eip::ScannerConfig;
using eip::ThetaScannerArgs;
using eip::chipRecoverNextBackoffMs;
using eip::chipRecoverOnFailure;
using eip::kChipRecoverInitialBackoffMs;
using eip::kChipRecoverMaxBackoffMs;
using eip::kMaxChipRecovers;
using eip::makeKinetixScannerConfig;
using eip::makeThetaScannerConfig;

static void test_hcs01_p2p_config_pins_t_to_o_bug(void) {
  ThetaScannerArgs a;
  a.ip = "192.168.1.23";
  a.rpi_us = 2000;
  a.originator_vendor_id = 1;
  const ScannerConfig cfg = makeThetaScannerConfig(a);

  TEST_ASSERT_EQUAL_STRING("192.168.1.23", cfg.target_ip);
  TEST_ASSERT_EQUAL_UINT32(0xC0A80117u, cfg.target_ip_host);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ScannerConfig::DriveFamily::kHcs01),
                        static_cast<int>(cfg.drive_family));
  TEST_ASSERT_EQUAL_UINT16(0, cfg.config_assembly_instance);
  TEST_ASSERT_EQUAL_UINT16(101, cfg.ot_assembly_instance);
  TEST_ASSERT_EQUAL_UINT16(102, cfg.to_assembly_instance);
  TEST_ASSERT_EQUAL_UINT32(18, static_cast<uint32_t>(cfg.ot_assembly_size));
  TEST_ASSERT_EQUAL_UINT32(14, static_cast<uint32_t>(cfg.to_assembly_size));
  TEST_ASSERT_EQUAL_UINT32(0x10000003u, cfg.ot_connection_id);
  TEST_ASSERT_EQUAL_UINT32(0x20000003u, cfg.to_connection_id);
  TEST_ASSERT_EQUAL_UINT16(0x0003, cfg.connection_serial);
  TEST_ASSERT_TRUE(cfg.include_run_idle_header);
  TEST_ASSERT_FALSE(cfg.include_run_idle_bit_in_net_params);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectionType::kPointToPoint),
                        static_cast<int>(cfg.to_connection_type));
  TEST_ASSERT_EQUAL_UINT32(2000, cfg.ot_rpi_us);
  TEST_ASSERT_EQUAL_UINT32(2000, cfg.to_rpi_us);
}

static void test_kinetix_assemblies_104_154(void) {
  KinetixScannerArgs a;
  a.ip = "192.168.1.20";
  a.connection_serial = 0x0001;
  a.ot_connection_id = 0x10000001;
  a.config_assembly_instance = 191;
  a.rpi_us = 2000;
  a.originator_vendor_id = 1;
  const ScannerConfig cfg = makeKinetixScannerConfig(a);

  TEST_ASSERT_EQUAL_INT(static_cast<int>(ScannerConfig::DriveFamily::kKinetix5100),
                        static_cast<int>(cfg.drive_family));
  TEST_ASSERT_EQUAL_UINT16(191, cfg.config_assembly_instance);
  TEST_ASSERT_EQUAL_UINT16(104, cfg.ot_assembly_instance);
  TEST_ASSERT_EQUAL_UINT16(154, cfg.to_assembly_instance);
  TEST_ASSERT_EQUAL_UINT32(40, static_cast<uint32_t>(cfg.ot_assembly_size));
  TEST_ASSERT_EQUAL_UINT32(52, static_cast<uint32_t>(cfg.to_assembly_size));
  TEST_ASSERT_EQUAL_UINT32(0x20000001u, cfg.to_connection_id);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectionType::kPointToPoint),
                        static_cast<int>(cfg.to_connection_type));
}

static void test_kinetix_multicast_override(void) {
  KinetixScannerArgs a;
  a.ip = "192.168.1.21";
  a.to_point_to_point = false;
  const ScannerConfig cfg = makeKinetixScannerConfig(a);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectionType::kMulticast),
                        static_cast<int>(cfg.to_connection_type));
}

static void test_recover_streak_restarts_after_three(void) {
  unsigned streak = 0;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ChipRecoverDecision::kAttemptRecover),
                        static_cast<int>(chipRecoverOnFailure(streak)));
  TEST_ASSERT_EQUAL_UINT(1, streak);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ChipRecoverDecision::kAttemptRecover),
                        static_cast<int>(chipRecoverOnFailure(streak)));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ChipRecoverDecision::kAttemptRecover),
                        static_cast<int>(chipRecoverOnFailure(streak)));
  TEST_ASSERT_EQUAL_UINT(kMaxChipRecovers, streak);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ChipRecoverDecision::kRestart),
                        static_cast<int>(chipRecoverOnFailure(streak)));
}

static void test_recover_backoff_doubles_then_caps(void) {
  TEST_ASSERT_EQUAL_UINT32(kChipRecoverInitialBackoffMs,
                           chipRecoverNextBackoffMs(5000, true));
  uint32_t b = kChipRecoverInitialBackoffMs;
  b = chipRecoverNextBackoffMs(b, false);
  TEST_ASSERT_EQUAL_UINT32(2000, b);
  b = chipRecoverNextBackoffMs(b, false);
  TEST_ASSERT_EQUAL_UINT32(4000, b);
  b = 16000;
  b = chipRecoverNextBackoffMs(b, false);
  TEST_ASSERT_EQUAL_UINT32(kChipRecoverMaxBackoffMs, b);
  b = chipRecoverNextBackoffMs(b, false);
  TEST_ASSERT_EQUAL_UINT32(kChipRecoverMaxBackoffMs, b);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_hcs01_p2p_config_pins_t_to_o_bug);
  RUN_TEST(test_kinetix_assemblies_104_154);
  RUN_TEST(test_kinetix_multicast_override);
  RUN_TEST(test_recover_streak_restarts_after_three);
  RUN_TEST(test_recover_backoff_doubles_then_caps);
  return UNITY_END();
}
