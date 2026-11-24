/**
 * @file gantry_console_cmd_pick.cpp
 * @brief Gantry test console: pick scheduler, belt and trace bench commands.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

void cmdNetStat(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  char line[200];
  if (!pickSchedulerNetStats(strcmp(cmdLower, "netstat reset") == 0, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdVisMark(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  const char* arg = cmdLower + 7;
  while (*arg == ' ') {
    ++arg;
  }
  char line[160];
  const bool on = strcmp(arg, "on") == 0;
  if ((!on && strcmp(arg, "off") != 0) || !pickSchedulerVisMark(on, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", (on || strcmp(arg, "off") == 0) ? line : "usage: vismark on|off");
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdGripMark(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  // Ahead of "grip", whose prefix test would also take "gripmark".
  const char* arg = cmdLower + 8;
  while (*arg == ' ') {
    ++arg;
  }
  char line[160];
  const bool on = strcmp(arg, "on") == 0;
  if ((!on && strcmp(arg, "off") != 0) || !pickSchedulerGripMark(on, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", (on || strcmp(arg, "off") == 0) ? line : "usage: gripmark on|off");
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdPickSim(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float theta_deg = 0.0f;
  int battery_class = 0;
  unsigned item_id = 0;
  float width_mm = 0.0f;
  float length_mm = 0.0f;
  float height_from_belt_mm = 0.0f;
  const int n = sscanf(cmdLower, "picksim %f %f %f %d %u %f %f %f", &x_mm, &y_mm, &theta_deg,
                       &battery_class, &item_id, &width_mm, &length_mm, &height_from_belt_mm);
  if (n != 8) {
    ESP_LOGE(TAG,
             "Usage: picksim <x> <y> <theta> <class 1-13> <item_id> <width_mm> <length_mm> <height_from_belt_mm>");
    return;
  }
  if (!pickSchedulerInject(x_mm, y_mm, theta_deg, static_cast<uint8_t>(battery_class), item_id,
                           width_mm, length_mm, height_from_belt_mm)) {
    ESP_LOGE(TAG, "ERROR: picksim not queued");
  }
}

void cmdPickPlan(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float theta_deg = 0.0f;
  int battery_class = 0;
  unsigned item_id = 0;
  float width_mm = 0.0f;
  float length_mm = 0.0f;
  float height_from_belt_mm = 0.0f;
  const int n = sscanf(cmdLower, "pickplan %f %f %f %d %u %f %f %f", &x_mm, &y_mm, &theta_deg,
                       &battery_class, &item_id, &width_mm, &length_mm, &height_from_belt_mm);
  if (n != 8) {
    ESP_LOGE(TAG,
             "Usage: pickplan <x> <y> <theta> <class 1-13> <item_id> <width_mm> <length_mm> <height_from_belt_mm>");
    return;
  }
  char plan[512];
  if (!pickSchedulerPlan(x_mm, y_mm, theta_deg, static_cast<uint8_t>(battery_class), item_id,
                         plan, sizeof(plan), width_mm, length_mm, height_from_belt_mm)) {
    ESP_LOGE(TAG, "ERROR: %s", plan);
    return;
  }
  ESP_LOGI(TAG, "pickplan %s", plan);
}

void cmdPickCover(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  int cls = 1;
  if (sscanf(cmdLower, "pickcover %d", &cls) != 1) {
    cls = 1;
  }
  char line[384];
  if (!pickSchedulerCover(static_cast<uint8_t>(cls), line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "pickcover %s", line);
}

void cmdPickCal(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  const char* args = cmdLower + 7;
  while (*args == ' ') {
    ++args;
  }
  char line[384];
  if (!pickSchedulerCalibrate(args, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdBeltSim(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  float mm_s = 0.0f;
  if (sscanf(cmdLower, "beltsim %f", &mm_s) != 1) {
    ESP_LOGE(TAG, "Usage: beltsim <mm_s>  (0 = off)");
    return;
  }
  char line[160];
  if (!pickSchedulerBeltSim(mm_s, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdZTrace(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  const char* args = cmdLower + 6;
  while (*args == ' ') {
    ++args;
  }
  char line[512];
  if (!pickSchedulerTrace(args, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdThTrace(const GantryTestConsoleConfig *cfg, const char *, const char *cmdLower) {
  const char* args = cmdLower + 7;
  while (*args == ' ') {
    ++args;
  }
  char line[768];
  if (!thetaTraceCommand(cfg->gantry, args, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdBeltCal(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
  const char* arg = cmdLower + 7;
  while (*arg == ' ') {
    ++arg;
  }
  char line[384];
  if (!pickSchedulerBeltCal(arg, line, sizeof(line))) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

}  // namespace GantryConsoleImpl
