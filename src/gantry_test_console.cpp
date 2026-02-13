#include "gantry_test_console.h"

#include "basic_tests.h"
#include "freertos/task.h"
#include "gpio_expander.h"
#include "esp_log.h"

#include <ctype.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

extern "C" int getchar(void);

namespace {
static const char *TAG = "GantryConsole";

void printStatus(Gantry::Gantry *gantry) {
  if (gantry == nullptr) {
    ESP_LOGI(TAG, "ERROR: Gantry not initialized");
    return;
  }

  ESP_LOGI(TAG, "=== Gantry Status ===");
  ESP_LOGI(TAG, "X Position: %d pulses", gantry->getXEncoder());
  ESP_LOGI(TAG, "Y Position: %d mm", gantry->getCurrentY());
  ESP_LOGI(TAG, "Theta: %d deg", gantry->getCurrentTheta());
  ESP_LOGI(TAG, "Busy: %s", gantry->isBusy() ? "Yes" : "No");
  ESP_LOGI(TAG, "Alarm: %s", gantry->isAlarmActive() ? "Yes" : "No");

  Gantry::JointConfig current = gantry->getCurrentJointConfig();
  ESP_LOGI(TAG, "Joint Config: x=%.1f y=%.1f theta=%.1f", current.x, current.y, current.theta);

  Gantry::EndEffectorPose pose = gantry->getCurrentEndEffectorPose();
  ESP_LOGI(TAG, "End-Effector: x=%.1f y=%.1f z=%.1f theta=%.1f", pose.x, pose.y, pose.z,
           pose.theta);
}

void printLimits(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGI(TAG, "ERROR: Gantry not initialized");
    return;
  }

  ESP_LOGI(TAG, "=== Limit Switches ===");
  uint8_t limit_min = gpio_expander_read(cfg->limit_min_pin);
  uint8_t limit_max = gpio_expander_read(cfg->limit_max_pin);
  if (cfg->use_mcp23s17) {
    ESP_LOGI(TAG, "X_LS_MIN (MCP23S17 PA%d / Home): %s", cfg->limit_min_pin,
             limit_min == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
    ESP_LOGI(TAG, "X_LS_MAX (MCP23S17 PA%d / End):  %s", cfg->limit_max_pin,
             limit_max == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
  } else {
    ESP_LOGI(TAG, "X_LS_MIN (GPIO %d / Home): %s", cfg->limit_min_pin,
             limit_min == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
    ESP_LOGI(TAG, "X_LS_MAX (GPIO %d / End):  %s", cfg->limit_max_pin,
             limit_max == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
  }
}

void printActivePins(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr) {
    ESP_LOGI(TAG, "ERROR: Pin configuration not available");
    return;
  }

  ESP_LOGI(TAG, "=== Active Pin Configuration ===");
  ESP_LOGI(TAG, "Mode: %s", cfg->use_mcp23s17 ? "MCP23S17 IO Expander" : "Direct WT32 GPIO (temporary)");

  ESP_LOGI(TAG, "X Pulse      : %d", cfg->x_pulse_pin);
  ESP_LOGI(TAG, "X Dir        : %d", cfg->x_dir_pin);
  ESP_LOGI(TAG, "X Enable     : %d", cfg->x_enable_pin);
  ESP_LOGI(TAG, "X Alarm In   : %d", cfg->x_alarm_pin);
  if (cfg->x_alarm_reset_pin >= 0) {
    ESP_LOGI(TAG, "X Alarm Reset: %d", cfg->x_alarm_reset_pin);
  } else {
    ESP_LOGI(TAG, "X Alarm Reset: disabled");
  }
  ESP_LOGI(TAG, "X Encoder A  : %d", cfg->x_encoder_a_pin);
  ESP_LOGI(TAG, "X Encoder B  : %d", cfg->x_encoder_b_pin);
  ESP_LOGI(TAG, "Theta PWM    : %d", cfg->theta_pwm_pin);

  if (cfg->use_mcp23s17) {
    ESP_LOGI(TAG, "X Min Limit  : MCP P%d", cfg->limit_min_pin);
    ESP_LOGI(TAG, "X Max Limit  : MCP P%d", cfg->limit_max_pin);
  } else {
    ESP_LOGI(TAG, "X Min Limit  : GPIO %d", cfg->limit_min_pin);
    ESP_LOGI(TAG, "X Max Limit  : GPIO %d", cfg->limit_max_pin);
  }

  ESP_LOGI(TAG, "========================================");
}

void processCommand(const GantryTestConsoleConfig *cfg, const char *cmd) {
  if (cfg == nullptr || cmd == nullptr || strlen(cmd) == 0) {
    return;
  }

  char cmdLower[256];
  strncpy(cmdLower, cmd, sizeof(cmdLower) - 1);
  cmdLower[sizeof(cmdLower) - 1] = '\0';
  for (int i = 0; cmdLower[i]; i++) {
    cmdLower[i] = static_cast<char>(tolower(cmdLower[i]));
  }

  if (cfg->gantry == nullptr && strcmp(cmdLower, "help") != 0) {
    ESP_LOGI(TAG, "ERROR: Gantry not initialized");
    return;
  }

  if (strcmp(cmdLower, "help") == 0 || strcmp(cmdLower, "?") == 0) {
    gantryTestPrintHelp();
  } else if (strcmp(cmdLower, "status") == 0) {
    printStatus(cfg->gantry);
  } else if (strcmp(cmdLower, "limits") == 0) {
    printLimits(cfg);
  } else if (strcmp(cmdLower, "pins") == 0) {
    printActivePins(cfg);
  } else if (strcmp(cmdLower, "enable") == 0) {
    cfg->gantry->enable();
    ESP_LOGI(TAG, "OK Motors enabled");
  } else if (strcmp(cmdLower, "disable") == 0) {
    cfg->gantry->disable();
    ESP_LOGI(TAG, "OK Motors disabled");
  } else if (strcmp(cmdLower, "home") == 0) {
    ESP_LOGI(TAG, "Starting homing sequence...");
    cfg->gantry->home();
    while (cfg->gantry->isBusy()) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "OK Homing complete");
  } else if (strcmp(cmdLower, "calibrate") == 0) {
    ESP_LOGI(TAG, "Starting calibration...");
    int len = cfg->gantry->calibrate();
    if (len > 0) {
      ESP_LOGI(TAG, "OK Calibrated length: %d mm", len);
    } else {
      ESP_LOGI(TAG, "ERROR: Calibration failed");
    }
  } else if (strncmp(cmdLower, "move", 4) == 0) {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    int parsed = sscanf(cmd, "move %f %f %f", &x, &y, &theta);
    if (parsed < 3) {
      ESP_LOGI(TAG, "ERROR: Usage: move <x_mm> <y_mm> <theta_deg>");
      return;
    }

    Gantry::JointConfig target;
    target.x = x;
    target.y = y;
    target.theta = theta;

    ESP_LOGI(TAG, "Moving to: x=%.1f y=%.1f theta=%.1f", x, y, theta);
    Gantry::GantryError result = cfg->gantry->moveTo(target, 50, 30);
    if (result == Gantry::GantryError::OK) {
      ESP_LOGI(TAG, "OK Move command accepted");
      while (cfg->gantry->isBusy()) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      ESP_LOGI(TAG, "OK Move complete");
    } else {
      ESP_LOGE(TAG, "ERROR: Move failed: %d", (int)result);
    }
  } else if (strncmp(cmdLower, "grip", 4) == 0) {
    int value = 0;
    int parsed = sscanf(cmd, "grip %d", &value);
    if (parsed < 1) {
      ESP_LOGI(TAG, "ERROR: Usage: grip <0|1>");
      return;
    }
    cfg->gantry->grip(value != 0);
    ESP_LOGI(TAG, "OK Gripper %s", value ? "closed" : "opened");
  } else if (strcmp(cmdLower, "stop") == 0) {
    cfg->gantry->disable();
    cfg->gantry->enable();
    ESP_LOGI(TAG, "OK Stop requested");
  } else if (strcmp(cmdLower, "alarmreset") == 0 || strcmp(cmdLower, "arst") == 0) {
    if (cfg->gantry->clearAlarm()) {
      ESP_LOGI(TAG, "OK Alarm reset pulse sent");
    } else {
      ESP_LOGE(TAG, "ERROR: Alarm reset failed (ARST pin may be disabled)");
    }
  } else if (strcmp(cmdLower, "selftest") == 0) {
    BasicTestSummary result = runBasicTests();
    ESP_LOGI(TAG, "Selftest complete: passed=%d failed=%d", result.passed, result.failed);
  } else {
    ESP_LOGE(TAG, "ERROR: Unknown command: %s", cmd);
    ESP_LOGI(TAG, "Type 'help' for available commands");
  }
}
}  // namespace

void gantryTestPrintHelp() {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Gantry Library Example - Commands:");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  help                 - show this help");
  ESP_LOGI(TAG, "  status               - print gantry status");
  ESP_LOGI(TAG, "  limits               - read limit switches");
  ESP_LOGI(TAG, "  pins                 - print active pin configuration");
  ESP_LOGI(TAG, "  enable               - enable motors");
  ESP_LOGI(TAG, "  disable              - disable motors");
  ESP_LOGI(TAG, "  home                 - home X-axis");
  ESP_LOGI(TAG, "  calibrate            - calibrate X-axis length");
  ESP_LOGI(TAG, "  move <x> <y> <t>     - move to position (mm, mm, deg)");
  ESP_LOGI(TAG, "  grip <0|1>           - control gripper (0=open, 1=close)");
  ESP_LOGI(TAG, "  stop                 - stop all motion");
  ESP_LOGI(TAG, "  alarmreset | arst    - pulse alarm reset output (ARST)");
  ESP_LOGI(TAG, "  selftest             - run basic math/config tests");
  ESP_LOGI(TAG, "");
}

void gantryTestConsoleTask(void *param) {
  auto *cfg = static_cast<GantryTestConsoleConfig *>(param);

  char inputLine[256];
  size_t inputIndex = 0;

  ESP_LOGI(TAG, "Serial task started");
  ESP_LOGI(TAG, "Type 'help' for commands");

  while (1) {
    int c = getchar();
    if (c >= 0) {
      if (c == '\n' || c == '\r' || c == ';') {
        if (inputIndex > 0) {
          inputLine[inputIndex] = '\0';
          ESP_LOGI(TAG, "[RX] %s", inputLine);
          processCommand(cfg, inputLine);
          inputIndex = 0;
        }
      } else if (c >= 32 && c <= 126 && inputIndex < sizeof(inputLine) - 1) {
        inputLine[inputIndex++] = static_cast<char>(c);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
