#ifndef GANTRY_TEST_CONSOLE_H
#define GANTRY_TEST_CONSOLE_H

#include "Gantry.h"
#include <stddef.h>
#include <stdint.h>
#include "W5500Hal.h"

/**
 * @struct GantryTestConsoleConfig
 * @brief Wiring snapshot passed to the diagnostic console task.
 *
 * Coordinate convention (firmware-wide, as of 2026-05):
 *   X = horizontal traverse, Y = along-belt (no gantry actuator; +Y =
 *   downstream), Z = vertical (+Z = down; 0 = A015 retract). The vertical actuator was
 *   labelled "Y" in pre-2026-05 firmware; the fields below use the new names.
 */
struct GantryTestConsoleConfig {
  Gantry::Gantry *gantry = nullptr;
  int limit_min_pin = -1;
  int limit_max_pin = -1;
  bool use_mcp23s17 = false;
  bool limit_switches_active = false;
  int x_pulse_pin = -1;
  int x_dir_pin = -1;
  int x_enable_pin = -1;
  int x_alarm_pin = -1;
  int x_alarm_reset_pin = -1;
  int z_alarm_pin = -1;
  int z_alarm_reset_pin = -1;
  int x_encoder_a_pin = -1;
  int x_encoder_b_pin = -1;
  int z_pulse_pin = -1;
  int z_encoder_a_pin = -1;
  int z_encoder_b_pin = -1;
  int x_pulse_ledc_channel = -1;
  int z_pulse_ledc_channel = -1;
  int theta_pulse_ledc_channel = -1;
  int theta_pulse_pin = -1;  // theta pulse STEP GPIO
  w5500::W5500Hal* w5500_hal = nullptr; // W5500 SPI driver for EtherNet/IP / COMWS
};

/** Optional reply sink for Ethernet console (tees ESP_LOG during a command). */
using GantryConsoleReplyFn = void (*)(void *ctx, const char *text);

/**
 * Run one console command line (shared by UART and TCP).
 * When reply_fn is non-null, ESP_LOG output for this command is also forwarded
 * (unless a session log sink is already attached).
 */
void gantryConsoleProcessLine(const GantryTestConsoleConfig *cfg, const char *line,
                              GantryConsoleReplyFn reply_fn = nullptr,
                              void *reply_ctx = nullptr);

/**
 * Attach a session-long ESP_LOG sink (e.g. TCP client). All logs are forwarded
 * until gantryConsoleDetachLogSink(). Safe to call from NetConsole task.
 */
void gantryConsoleAttachLogSink(GantryConsoleReplyFn fn, void *ctx);
void gantryConsoleDetachLogSink(void);

void gantryTestConsoleTask(void *param);
void gantryTestPrintHelp();
/** Load the console move profile from the stored parameters. Call once after
 *  AppConfig::init(); a 'cfg' change to the profile reloads it. */
void gantryConsoleLoadStoredProfile();

// How a command word matches the lowercased line. First match wins.
enum class GantryConsoleMatch : uint8_t {
    kExact,   // the whole line
    kPrefix,  // the line starts with the word
    kWord,    // the word, then end of line or a space
    kWordWs,  // the word, then end of line, a space or a tab
};

using GantryConsoleCommandFn =
    void (*)(const GantryTestConsoleConfig* cfg, const char* cmd, const char* cmdLower);

struct GantryConsoleCommand {
    const char* word;
    GantryConsoleMatch match;
    GantryConsoleCommandFn run;
};

// Extra command tables, searched before the console's own table.
void gantryConsoleRegisterCommands(const GantryConsoleCommand* table, size_t n);

// Pick, belt and trace commands. Defined in gantry_console_cmd_pick.cpp.
const GantryConsoleCommand* gantryConsolePickCommands(size_t* count);

// selftest. Defined in basic_tests.cpp, which is built only with
// CONFIG_GANTRY_SELFTEST.
const GantryConsoleCommand* gantryConsoleSelfTestCommands(size_t* count);

#endif  // GANTRY_TEST_CONSOLE_H
