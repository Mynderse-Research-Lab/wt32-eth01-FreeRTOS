#ifndef GANTRY_TEST_CONSOLE_H
#define GANTRY_TEST_CONSOLE_H

#include "Gantry.h"
#include <stdint.h>

struct GantryTestConsoleConfig {
  Gantry::Gantry *gantry;
  uint8_t limit_x_min_pin;
  uint8_t limit_x_max_pin;
  uint8_t limit_y_min_pin;
  uint8_t limit_y_max_pin;
  bool limit_switches_active;
  int x_pulse_pin;
  int x_dir_pin;
  int x_enable_pin;
  int x_alarm_pin;
  int x_alarm_reset_pin;
  int y_alarm_pin;
  int y_alarm_reset_pin;
  int x_encoder_a_pin;
  int x_encoder_b_pin;
  int y_pulse_pin;
  int y_encoder_a_pin;
  int y_encoder_b_pin;
  int x_pulse_ledc_channel;
  int y_pulse_ledc_channel;
  int theta_pwm_pin;
};

void gantryTestConsoleTask(void *param);
void gantryTestPrintHelp();

#endif  // GANTRY_TEST_CONSOLE_H
