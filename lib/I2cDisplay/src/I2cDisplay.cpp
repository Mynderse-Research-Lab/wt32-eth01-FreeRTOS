#include "I2cDisplay.h"

#include "esp_log.h"

namespace display {
namespace {

constexpr const char* TAG = "I2cDisplay";

}  // namespace

bool I2cDisplay::begin(const I2cDisplayConfig& cfg) {
  if (cfg.sda_gpio < 0 || cfg.scl_gpio < 0) {
    ESP_LOGE(TAG, "begin: invalid SDA/SCL (%d/%d)", cfg.sda_gpio, cfg.scl_gpio);
    ready_ = false;
    return false;
  }

  cfg_ = cfg;
  ready_ = true;

  // TODO: i2c_master bus + device add; probe 0x3C/0x3D; SSD1306 init sequence.
  ESP_LOGW(TAG,
           "stub begin OK (SDA=%d SCL=%d addr=0x%02X @ %lu Hz) — panel I/O not "
           "implemented",
           cfg_.sda_gpio, cfg_.scl_gpio, cfg_.i2c_addr,
           static_cast<unsigned long>(cfg_.scl_hz));
  return true;
}

void I2cDisplay::clear() {
  if (!ready_) return;
  // TODO: clear framebuffer + refresh.
}

void I2cDisplay::printLine(int row, const char* text) {
  if (!ready_ || text == nullptr) return;
  (void)row;
  // TODO: render text to row and refresh.
  ESP_LOGD(TAG, "stub printLine(%d): %s", row, text);
}

}  // namespace display
