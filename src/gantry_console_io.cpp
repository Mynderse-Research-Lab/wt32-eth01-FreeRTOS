/**
 * @file gantry_console_io.cpp
 * @brief Gantry test console: MCP23S17, field I/O and GPIO bench commands.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

#if MCP_DEBUG_CMDS
void runMcpPinModeCommand(const char *cmd) {
  int pin = -1;
  char mode[16] = {0};
  if (sscanf(cmd, "mcp_pin_mode %d %15s", &pin, mode) < 2) {
    ESP_LOGE(TAG, "Usage: mcp_pin_mode <0..15> <inpu|in|out0|out1>");
    return;
  }
  if (pin < 0 || pin > 15) {
    ESP_LOGE(TAG, "Pin must be in range 0..15 for MCP23S17");
    return;
  }

  for (int i = 0; mode[i]; ++i) {
    mode[i] = static_cast<char>(tolower(mode[i]));
  }

  const uint8_t pin_u8 = static_cast<uint8_t>(pin);
  auto logImmediateMcpState = [pin]() {
    mcp23s17_handle_t h = gpio_expander_get_mcp_handle();
    if (h == nullptr) {
      ESP_LOGE(TAG, "MCP handle not available for immediate readback");
      return;
    }

    const bool isPortA = (pin < 8);
    const int bit = pin & 0x07;
    const uint8_t reg_iodir = isPortA ? 0x00 : 0x01;
    const uint8_t reg_gppu = isPortA ? 0x0C : 0x0D;
    const uint8_t reg_gpio = isPortA ? 0x12 : 0x13;
    const uint8_t reg_olat = isPortA ? 0x14 : 0x15;
    uint8_t iodir = 0, gppu = 0, olat = 0, gpio = 0;

    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_iodir, &iodir));
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gppu, &gppu));
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_olat, &olat));
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gpio, &gpio));

    ESP_LOGI(TAG,
             "MCP immediate pin %d: dir=%d(1=input) pullup=%d olat=%d gpio=%d [IODIR=0x%02X GPPU=0x%02X OLAT=0x%02X GPIO=0x%02X]",
             pin, (iodir >> bit) & 0x1, (gppu >> bit) & 0x1, (olat >> bit) & 0x1,
             (gpio >> bit) & 0x1, iodir, gppu, olat, gpio);
  };

  if (strcmp(mode, "inpu") == 0 || strcmp(mode, "in") == 0) {
    ESP_ERROR_CHECK(gpio_expander_set_direction(pin_u8, false));
    ESP_ERROR_CHECK(gpio_expander_set_pullup(pin_u8, true));
    vTaskDelay(pdMS_TO_TICKS(2));
    const int level = gpio_expander_read(pin_u8);
    ESP_LOGI(TAG, "MCP pin %d -> INPUT_PULLUP, level=%d", pin, level);
    logImmediateMcpState();
    return;
  }
  if (strcmp(mode, "out0") == 0) {
    ESP_ERROR_CHECK(gpio_expander_set_direction(pin_u8, true));
    ESP_ERROR_CHECK(gpio_expander_write(pin_u8, 0));
    vTaskDelay(pdMS_TO_TICKS(2));
    const int level = gpio_expander_read(pin_u8);
    ESP_LOGI(TAG, "MCP pin %d -> OUTPUT LOW, level=%d", pin, level);
    logImmediateMcpState();
    return;
  }
  if (strcmp(mode, "out1") == 0) {
    ESP_ERROR_CHECK(gpio_expander_set_direction(pin_u8, true));
    ESP_ERROR_CHECK(gpio_expander_write(pin_u8, 1));
    vTaskDelay(pdMS_TO_TICKS(2));
    const int level = gpio_expander_read(pin_u8);
    ESP_LOGI(TAG, "MCP pin %d -> OUTPUT HIGH, level=%d", pin, level);
    logImmediateMcpState();
    return;
  }

  ESP_LOGE(TAG, "Invalid mode '%s'. Use inpu|in|out0|out1", mode);
}

void runMcpDumpCommand(const char *cmd) {
  char portChar = 0;
  if (sscanf(cmd, "mcp_dump %c", &portChar) < 1) {
    ESP_LOGE(TAG, "Usage: mcp_dump <a|b>");
    return;
  }

  const char port = static_cast<char>(tolower(static_cast<unsigned char>(portChar)));
  if (port != 'a' && port != 'b') {
    ESP_LOGE(TAG, "Port must be 'a' or 'b'");
    return;
  }

  mcp23s17_handle_t h = gpio_expander_get_mcp_handle();
  if (h == nullptr) {
    ESP_LOGE(TAG, "MCP handle not available");
    return;
  }

  uint8_t iocon = 0, iodir = 0, gppu = 0, olat = 0, gpio = 0;
  const uint8_t reg_iodir = (port == 'a') ? 0x00 : 0x01;
  const uint8_t reg_gppu = (port == 'a') ? 0x0C : 0x0D;
  const uint8_t reg_gpio = (port == 'a') ? 0x12 : 0x13;
  const uint8_t reg_olat = (port == 'a') ? 0x14 : 0x15;

  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, 0x0A, &iocon));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_iodir, &iodir));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gppu, &gppu));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_olat, &olat));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gpio, &gpio));

  ESP_LOGI(TAG, "MCP dump port %c: IOCON=0x%02X IODIR%c=0x%02X GPPU%c=0x%02X OLAT%c=0x%02X GPIO%c=0x%02X",
           (char)toupper((unsigned char)port), iocon,
           (char)toupper((unsigned char)port), iodir,
           (char)toupper((unsigned char)port), gppu,
           (char)toupper((unsigned char)port), olat,
           (char)toupper((unsigned char)port), gpio);

  if (port == 'b') {
    const int pb4_dir = (iodir >> 4) & 0x1;
    const int pb4_pull = (gppu >> 4) & 0x1;
    const int pb4_olat = (olat >> 4) & 0x1;
    const int pb4_gpio = (gpio >> 4) & 0x1;
    ESP_LOGI(TAG, "PB4 bits: dir=%d(1=input) pullup=%d olat=%d gpio=%d",
             pb4_dir, pb4_pull, pb4_olat, pb4_gpio);
  }
}

void runMcpRegCommand(const char *cmd) {
  char action[8] = {0};
  char regStr[16] = {0};
  char valueStr[16] = {0};
  const int parsed = sscanf(cmd, "mcp_reg %7s %15s %15s", action, regStr, valueStr);
  if (parsed < 2) {
    ESP_LOGE(TAG, "Usage: mcp_reg <r|w> <reg> [value]");
    return;
  }

  for (int i = 0; action[i]; ++i) {
    action[i] = static_cast<char>(tolower(static_cast<unsigned char>(action[i])));
  }

  mcp23s17_handle_t h = gpio_expander_get_mcp_handle();
  if (h == nullptr) {
    ESP_LOGE(TAG, "MCP handle not available");
    return;
  }

  char *endPtr = nullptr;
  const unsigned long regUl = strtoul(regStr, &endPtr, 0);
  if (endPtr == regStr || *endPtr != '\0' || regUl > 0x1F) {
    ESP_LOGE(TAG, "Invalid reg '%s' (expected 0x00..0x1F)", regStr);
    return;
  }
  const uint8_t reg = static_cast<uint8_t>(regUl);

  if (strcmp(action, "r") == 0) {
    uint8_t value = 0;
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg, &value));
    ESP_LOGI(TAG, "MCP reg[0x%02X] = 0x%02X", reg, value);
    return;
  }

  if (strcmp(action, "w") == 0) {
    if (parsed < 3) {
      ESP_LOGE(TAG, "Usage: mcp_reg w <reg> <value>");
      return;
    }
    endPtr = nullptr;
    const unsigned long valueUl = strtoul(valueStr, &endPtr, 0);
    if (endPtr == valueStr || *endPtr != '\0' || valueUl > 0xFF) {
      ESP_LOGE(TAG, "Invalid value '%s' (expected 0x00..0xFF)", valueStr);
      return;
    }
    const uint8_t value = static_cast<uint8_t>(valueUl);
    ESP_ERROR_CHECK(mcp23s17_debug_write_register(h, reg, value));

    uint8_t verify = 0;
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg, &verify));
    ESP_LOGI(TAG, "MCP reg[0x%02X] <= 0x%02X, readback=0x%02X", reg, value, verify);
    return;
  }

  ESP_LOGE(TAG, "Invalid action '%s'. Use r or w", action);
}

void runFieldDoutCommand(const char *cmd) {
  int ch = -1;
  int level = -1;
  if (sscanf(cmd, "field_dout %d %d", &ch, &level) < 2) {
    ESP_LOGE(TAG, "Usage: field_dout <0..3> <0|1>");
    return;
  }
  if (ch < 0 || ch >= FIELD_24V_DOUT_COUNT || (level != 0 && level != 1)) {
    ESP_LOGE(TAG, "Usage: field_dout <0..3> <0|1>");
    return;
  }
  if (field_dout_set(static_cast<unsigned>(ch), level != 0) != ESP_OK) {
    ESP_LOGE(TAG, "field_dout failed (MCP not ready?)");
    return;
  }
  ESP_LOGI(TAG, "FIELD_DOUT%d = %d (MCP PA%d)", ch, level, MCP_FIELD_DOUT0 + ch);
}

void runFieldDinCommand(const char *cmd) {
  (void)cmd;
  if (gpio_expander_get_mcp_handle() == nullptr) {
    ESP_LOGE(TAG, "MCP not initialized");
    return;
  }
  for (unsigned i = 0; i < FIELD_24V_DIN_COUNT; ++i) {
    ESP_LOGI(TAG, "FIELD_DIN%d = %d (MCP PA%d)", i, field_din_get(i) ? 1 : 0,
             MCP_FIELD_DIN0 + static_cast<int>(i));
  }
  ESP_LOGI(TAG, "ENC KO/PUSH/B/A = %d %d %d %d  BLK=%d DC=%d RES=%d  W5500_RST(PB7)=%d",
           gpio_expander_read(MCP_UI_ENC_KO), gpio_expander_read(MCP_UI_ENC_PUSH),
           gpio_expander_read(MCP_UI_ENC_B), gpio_expander_read(MCP_UI_ENC_A),
           gpio_expander_read(MCP_TFT_BLK), gpio_expander_read(MCP_TFT_DC),
           gpio_expander_read(MCP_TFT_RES), gpio_expander_read(MCP_W5500_RST));
}
#endif  // MCP_DEBUG_CMDS

void runGpioDriveCommand(const char *cmd) {
  int gpio = -1;
  int value = -1;
  if (sscanf(cmd, "gpio_drive %d %d", &gpio, &value) < 2) {
    ESP_LOGE(TAG, "Usage: gpio_drive <gpio_num> <0|1>");
    return;
  }
  if (gpio < 0 || gpio >= 40 || (value != 0 && value != 1)) {
    ESP_LOGE(TAG, "Invalid args. gpio=0..39, value=0|1");
    return;
  }

  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << gpio);
  io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio, value));
  vTaskDelay(pdMS_TO_TICKS(2));
  int level = gpio_get_level((gpio_num_t)gpio);
  ESP_LOGI(TAG, "GPIO %d driven to %d, readback=%d", gpio, value, level);
}

}  // namespace GantryConsoleImpl
