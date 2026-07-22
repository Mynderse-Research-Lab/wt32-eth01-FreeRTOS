// Kinetix 5100 FaultCode / WarningCode display decode (assembly 154).
//
// Drive keypad shows Axxx (minor/warning) and Exxx (major/fault). The Class 1
// words are 16-bit; the low 12 bits are the hex display digits (A603 -> 0x0603).
// Curated from 2198-RD001 codes + UM004D bench notes — not the full catalog.

#ifndef ETHERNET_IP_KINETIX_FAULT_CODES_H
#define ETHERNET_IP_KINETIX_FAULT_CODES_H

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace eip {
namespace k5100 {

constexpr uint16_t kCodeMask12 = 0x0FFFu;
constexpr uint16_t kWarningA603 = 0x0603;  // Invalid I/O command data
constexpr uint16_t kFaultE602 = 0x0602;    // Control connection lost (typical)

struct DriveCodeInfo {
  const char* display;   // e.g. "A603"
  const char* name;      // short title
  const char* hint;      // corrective action (one line)
};

inline uint16_t codeDigits(uint16_t raw) { return static_cast<uint16_t>(raw & kCodeMask12); }

inline void formatDisplayCode(char* out, size_t n, char prefix, uint16_t raw) {
  if (out == nullptr || n == 0) return;
  std::snprintf(out, n, "%c%03X", prefix, codeDigits(raw));
}

// Lookup curated table. prefer_prefix 'A' for warning_code, 'E' for fault_code.
// display_buf receives e.g. "A603" (min 8 bytes).
inline bool lookupDriveCode(uint16_t raw, char prefer_prefix, char* display_buf,
                            size_t display_n, DriveCodeInfo& out) {
  const uint16_t d = codeDigits(raw);
  if (d == 0) return false;

  struct Entry {
    uint16_t digits;
    char prefix;
    const char* name;
    const char* hint;
  };
  static constexpr Entry kTable[] = {
      {0x603, 'A', "Invalid I/O command data",
       "Need Active (ServoOn 0→1 edge); moves use Speed + TravelMode=10 + "
       "StartMotion preload (Position+TM=2 A603s on this bench)"},
      {0x237, 'E', "Indexing mode cannot start",
       "Homing not defined — Position+TravelMode=10 needs a home; use Speed + "
       "TravelMode=10 jog-to-target instead"},
      {0x602, 'E', "Control connection lost",
       "Class 1 timeout — keep O->T within connection timeout; check W5500 link"},
      {0x601, 'E', "Control power lost / STO",
       "Check STO wiring and control power"},
      {0x610, 'A', "Position error excessive",
       "Reduce speed/accel or check mechanics / tuning"},
      {0x709, 'E', "Motor overtemperature", "Allow cool-down; check load"},
      {0x70A, 'E', "Drive overtemperature", "Allow cool-down; check ambient"},
      {0x100, 'E', "Overcurrent", "Check motor wiring and mechanical jam"},
      {0x200, 'E', "Overvoltage", "Check DC bus / regen"},
      {0x300, 'E', "Undervoltage", "Check AC supply"},
  };

  const Entry* preferred = nullptr;
  const Entry* any = nullptr;
  for (const Entry& e : kTable) {
    if (e.digits != d) continue;
    any = &e;
    if (e.prefix == prefer_prefix) {
      preferred = &e;
      break;
    }
  }
  const Entry* hit = preferred ? preferred : any;
  if (hit == nullptr) return false;

  formatDisplayCode(display_buf, display_n, hit->prefix, raw);
  out.display = display_buf;
  out.name = hit->name;
  out.hint = hit->hint;
  return true;
}

// Write human-readable summary into buf. Returns true if any fault/warning text.
inline bool formatDriveTripSummary(char* buf, size_t n, bool fault_bit,
                                   bool warning_bit, bool connection_faulted,
                                   uint16_t fault_code, uint16_t warning_code) {
  if (buf == nullptr || n == 0) return false;
  buf[0] = '\0';
  if (!fault_bit && !warning_bit && !connection_faulted && fault_code == 0 &&
      warning_code == 0) {
    std::snprintf(buf, n, "clear");
    return false;
  }

  char tmp[192];
  size_t used = 0;
  auto append = [&](const char* s) {
    if (s == nullptr || *s == '\0') return;
    const int wrote =
        std::snprintf(tmp + used, sizeof(tmp) - used, "%s%s", used ? "; " : "", s);
    if (wrote > 0) used += static_cast<size_t>(wrote);
  };

  if (connection_faulted) append("ConnectionFaulted");

  if (fault_bit || fault_code != 0) {
    DriveCodeInfo info{};
    char disp[8];
    char code[8];
    formatDisplayCode(code, sizeof(code), 'E', fault_code);
    if (lookupDriveCode(fault_code, 'E', disp, sizeof(disp), info)) {
      char line[128];
      std::snprintf(line, sizeof(line), "FAULT %s %s", info.display, info.name);
      append(line);
    } else {
      char line[64];
      std::snprintf(line, sizeof(line), "FAULT %s (0x%04X)", code, fault_code);
      append(line);
    }
  }

  if (warning_bit || warning_code != 0) {
    DriveCodeInfo info{};
    char disp[8];
    char code[8];
    formatDisplayCode(code, sizeof(code), 'A', warning_code);
    if (lookupDriveCode(warning_code, 'A', disp, sizeof(disp), info)) {
      char line[128];
      std::snprintf(line, sizeof(line), "WARN %s %s", info.display, info.name);
      append(line);
    } else {
      char line[64];
      std::snprintf(line, sizeof(line), "WARN %s (0x%04X)", code, warning_code);
      append(line);
    }
  }

  std::snprintf(buf, n, "%s", tmp);
  return true;
}

inline bool isWarningA603(uint16_t warning_code) {
  return codeDigits(warning_code) == kWarningA603;
}

}  // namespace k5100
}  // namespace eip

#endif  // ETHERNET_IP_KINETIX_FAULT_CODES_H
