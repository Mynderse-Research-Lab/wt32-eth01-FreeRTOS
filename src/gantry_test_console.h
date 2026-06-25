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
  uint8_t limit_min_pin = 0;
  uint8_t limit_max_pin = 0;
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

void gantryTestConsoleTask(void *param);
void gantryTestPrintHelp();

#endif  // GANTRY_TEST_CONSOLE_H
