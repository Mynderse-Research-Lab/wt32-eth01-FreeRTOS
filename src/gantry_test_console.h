#ifndef GANTRY_TEST_CONSOLE_H
#define GANTRY_TEST_CONSOLE_H

#include "Gantry.h"
#include <stdint.h>

struct GantryTestConsoleConfig {
  Gantry::Gantry *gantry;
  uint8_t limit_min_pin;
  uint8_t limit_max_pin;
};

void gantryTestConsoleTask(void *param);
void gantryTestPrintHelp();

#endif  // GANTRY_TEST_CONSOLE_H
