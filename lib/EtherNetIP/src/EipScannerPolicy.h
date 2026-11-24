/**
 * @file EipScannerPolicy.h
 * @brief Host-testable ScannerConfig builders and W5500 recover backoff.
 */
#pragma once

#include "EipScanner.h"
#include "Hcs01Assembly.h"

namespace eip {

inline constexpr unsigned kMaxChipRecovers = 3;
inline constexpr uint32_t kChipRecoverInitialBackoffMs = 1000;
inline constexpr uint32_t kChipRecoverMaxBackoffMs = 5000;

// CIP ForwardOpen connection timeout multiplier code: timeout = RPI * 4 << code.
// 0 (x4 = 8 ms @ 2 ms RPI) let any scanner stall drop the drive connection;
// 2 (x16 = 32 ms) stays above our 10x RPI stale teardown.
inline constexpr uint8_t kClass1ConnectionTimeoutMultiplier = 2;

// Per-axis reopen (HCS01 theta only, while X/Z stay up). The drive drops its
// side after 16x RPI; after this much silence it is gone and needs a new FO.
inline constexpr uint32_t kHcs01ReopenAfterStaleUs = 500000;  // 500 ms
inline constexpr uint32_t kAxisReopenInitialBackoffMs = 1000;
inline constexpr uint32_t kAxisReopenMaxBackoffMs = 10000;

/// Next retry delay after a failed per-axis reopen (doubling, capped).
inline uint32_t axisReopenNextBackoffMs(uint32_t current_ms) {
    if (current_ms == 0) {
        return kAxisReopenInitialBackoffMs;
    }
    const uint32_t doubled = current_ms * 2u;
    if (doubled < current_ms || doubled > kAxisReopenMaxBackoffMs) {
        return kAxisReopenMaxBackoffMs;
    }
    return doubled;
}

enum class ChipRecoverDecision { kAttemptRecover, kRestart };

inline ChipRecoverDecision chipRecoverOnFailure(unsigned& streak,
                                                unsigned max_recovers = kMaxChipRecovers) {
    ++streak;
    if (streak > max_recovers) {
        return ChipRecoverDecision::kRestart;
    }
    return ChipRecoverDecision::kAttemptRecover;
}

inline uint32_t chipRecoverNextBackoffMs(uint32_t current, bool recover_ok) {
    if (recover_ok) {
        return kChipRecoverInitialBackoffMs;
    }
    const uint32_t doubled = current * 2u;
    if (doubled < current || doubled > kChipRecoverMaxBackoffMs) {
        return kChipRecoverMaxBackoffMs;
    }
    return doubled;
}

struct KinetixScannerArgs {
    const char* ip = nullptr;
    uint16_t connection_serial = 0x0001;
    uint32_t ot_connection_id = 0x10000001;
    uint16_t config_assembly_instance = 191;
    uint32_t rpi_us = 2000;
    uint16_t originator_vendor_id = 0;
    bool to_point_to_point = true;
};

inline ScannerConfig makeKinetixScannerConfig(const KinetixScannerArgs& a) {
    ScannerConfig cfg;
    cfg.target_ip = a.ip;
    cfg.target_ip_host = parseIpv4Host(a.ip);
    cfg.drive_family = ScannerConfig::DriveFamily::kKinetix5100;
    cfg.config_assembly_instance = a.config_assembly_instance;
    cfg.ot_assembly_instance = 104;
    cfg.to_assembly_instance = 154;
    cfg.ot_assembly_size = 40;
    cfg.to_assembly_size = 52;
    cfg.ot_rpi_us = a.rpi_us;
    cfg.to_rpi_us = a.rpi_us;
    cfg.ot_connection_id = a.ot_connection_id;
    cfg.to_connection_id = 0x20000000u | static_cast<uint32_t>(a.connection_serial);
    cfg.connection_serial = a.connection_serial;
    cfg.originator_vendor_id = a.originator_vendor_id;
    cfg.connection_timeout_multiplier = kClass1ConnectionTimeoutMultiplier;
    cfg.include_run_idle_header = true;
    cfg.include_run_idle_bit_in_net_params = false;
    cfg.read_encap_inactivity_timeout = true;
    cfg.to_connection_type = a.to_point_to_point ? ConnectionType::kPointToPoint
                                                 : ConnectionType::kMulticast;
    return cfg;
}

struct ThetaScannerArgs {
    const char* ip = "192.168.1.23";
    uint32_t rpi_us = 2000;
    uint16_t originator_vendor_id = 0;
    bool to_point_to_point = true;
};

inline ScannerConfig makeThetaScannerConfig(const ThetaScannerArgs& a = {}) {
    ScannerConfig cfg;
    cfg.target_ip = a.ip;
    cfg.target_ip_host = parseIpv4Host(a.ip);
    cfg.drive_family = ScannerConfig::DriveFamily::kHcs01;
    cfg.config_assembly_instance = 0;
    cfg.ot_assembly_instance = hcs01::kOutputInstance101;
    cfg.to_assembly_instance = hcs01::kInputInstance102;
    cfg.ot_assembly_size = hcs01::kOutput101Size;
    cfg.to_assembly_size = hcs01::kInput102Size;
    cfg.ot_rpi_us = a.rpi_us;
    cfg.to_rpi_us = a.rpi_us;
    cfg.ot_connection_id = 0x10000003;
    cfg.to_connection_id = 0x20000003;
    cfg.connection_serial = 0x0003;
    cfg.originator_vendor_id = a.originator_vendor_id;
    cfg.connection_timeout_multiplier = kClass1ConnectionTimeoutMultiplier;
    cfg.include_run_idle_header = true;
    cfg.include_run_idle_bit_in_net_params = false;
    cfg.read_encap_inactivity_timeout = true;
    cfg.to_connection_type = a.to_point_to_point ? ConnectionType::kPointToPoint
                                                 : ConnectionType::kMulticast;
    return cfg;
}

}  // namespace eip
