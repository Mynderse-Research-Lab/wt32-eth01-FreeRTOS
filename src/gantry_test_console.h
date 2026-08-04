#ifndef GANTRY_TEST_CONSOLE_H
#define GANTRY_TEST_CONSOLE_H

#include "Gantry.h"
#include <stdint.h>

/**
 * @struct GantryTestConsoleConfig
 * @brief Wiring snapshot passed to the diagnostic console task.
 *
 * Coordinate convention (firmware-wide, as of 2026-05):
 *   X = horizontal traverse, Y = along-belt (no gantry actuator; -Y =
 *   downstream), Z = vertical (+Z = up; belt = 0). The vertical actuator was
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
};

/** Optional reply sink for Ethernet console (tees ESP_LOG during a command). */
using GantryConsoleReplyFn = void (*)(void *ctx, const char *text);

/**
 * Run one console command line (shared by UART and TCP).
 * When reply_fn is non-null, ESP_LOG output for this command is also forwarded.
 */
void gantryConsoleProcessLine(const GantryTestConsoleConfig *cfg, const char *line,
                              GantryConsoleReplyFn reply_fn = nullptr,
                              void *reply_ctx = nullptr);

void gantryTestConsoleTask(void *param);
void gantryTestPrintHelp();

#endif  // GANTRY_TEST_CONSOLE_H
