#ifndef I2C_DISPLAY_H
#define I2C_DISPLAY_H

#include <cstdint>
#include <cstddef>

/**
 * @file I2cDisplay.h
 * @brief Stub driver for an I2C operator display (SSD1306/SH1106-class).
 *
 * Pins: SDA=GPIO4, SCL=GPIO14 (see gantry_app_constants.h). Full panel
 * bring-up (bus init + panel probe + framebuffer) is not implemented yet;
 * begin() records config and returns true so app wiring can land early.
 */

namespace display {

struct I2cDisplayConfig {
  int sda_gpio = -1;
  int scl_gpio = -1;
  uint8_t i2c_addr = 0x3C;   // common SSD1306 default
  uint32_t scl_hz = 400000;  // Fast-mode
  int i2c_port = 0;          // I2C_NUM_0 when implemented
};

class I2cDisplay {
 public:
  I2cDisplay() = default;

  /** Record config; stub does not touch the I2C peripheral yet. */
  bool begin(const I2cDisplayConfig& cfg);

  bool isReady() const { return ready_; }

  /** Stub no-ops — real driver will clear the panel. */
  void clear();

  /**
   * Stub no-op — real driver will draw UTF-8/ASCII on a text row.
   * @param row 0-based text row (panel-dependent).
   */
  void printLine(int row, const char* text);

  const I2cDisplayConfig& config() const { return cfg_; }

 private:
  I2cDisplayConfig cfg_{};
  bool ready_ = false;
};

}  // namespace display

#endif  // I2C_DISPLAY_H
