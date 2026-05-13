#ifndef GANTRY_TEST_CONSOLE_H
#define GANTRY_TEST_CONSOLE_H

#include "Gantry.h"
#include <stdint.h>

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
  int y_alarm_pin = -1;
  int y_alarm_reset_pin = -1;
  int x_encoder_a_pin = -1;
  int x_encoder_b_pin = -1;
  int y_pulse_pin = -1;
  int y_encoder_a_pin = -1;
  int y_encoder_b_pin = -1;
  int x_pulse_ledc_channel = -1;
  int y_pulse_ledc_channel = -1;
  int theta_pulse_ledc_channel = -1;
  int theta_pwm_pin = -1;  // theta pulse STEP GPIO (legacy field name)
};

void gantryTestConsoleTask(void *param);
void gantryTestPrintHelp();

#endif  // GANTRY_TEST_CONSOLE_H
