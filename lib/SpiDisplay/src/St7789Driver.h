#ifndef ST7789_DRIVER_H
#define ST7789_DRIVER_H

#include "SpiDisplay.h"
#include <cstdint>
#include <cstddef>

namespace display {

enum class Orientation : uint8_t {
    LANDSCAPE = 1,      // 320 x 240 (Default)
    PORTRAIT = 0,       // 240 x 320
    LANDSCAPE_INV = 3,  // 320 x 240 Inverted
    PORTRAIT_INV = 2    // 240 x 320 Inverted
};

// Common RGB565 colors
constexpr uint16_t COLOR_BLACK       = 0x0000;
constexpr uint16_t COLOR_NAVY        = 0x000F;
constexpr uint16_t COLOR_DARKGREEN   = 0x03E0;
constexpr uint16_t COLOR_DARKCYAN    = 0x03EF;
constexpr uint16_t COLOR_MAROON      = 0x7800;
constexpr uint16_t COLOR_PURPLE      = 0x780F;
constexpr uint16_t COLOR_OLIVE       = 0x7BE0;
constexpr uint16_t COLOR_LIGHTGREY   = 0xC618;
constexpr uint16_t COLOR_DARKGREY    = 0x7BEF;
constexpr uint16_t COLOR_BLUE        = 0x001F;
constexpr uint16_t COLOR_GREEN       = 0x07E0;
constexpr uint16_t COLOR_CYAN        = 0x07FF;
constexpr uint16_t COLOR_RED         = 0xF800;
constexpr uint16_t COLOR_MAGENTA     = 0xF81F;
constexpr uint16_t COLOR_YELLOW      = 0xFFE0;
constexpr uint16_t COLOR_WHITE       = 0xFFFF;
constexpr uint16_t COLOR_ORANGE      = 0xFD20;
constexpr uint16_t COLOR_GREENYELLOW = 0xAFE5;
constexpr uint16_t COLOR_HEADER_BG   = 0x18E3; // Dark slate blue
constexpr uint16_t COLOR_PANEL_BG    = 0x0841; // Very dark grey/blue
constexpr uint16_t COLOR_BORDER      = 0x39E7; // Subtle border

class St7789Driver {
public:
    St7789Driver();
    ~St7789Driver();

    bool init(const SpiDisplayConfig& cfg);
    bool isReady() const { return ready_; }

    void setOrientation(Orientation orient);
    Orientation getOrientation() const { return orientation_; }

    uint16_t getWidth() const { return width_; }
    uint16_t getHeight() const { return height_; }

    void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void fillScreen(uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    void drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    void drawChar(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg);
    void drawString(int16_t x, int16_t y, const char* str, uint16_t fg, uint16_t bg);
    void drawStringCentered(int16_t y, const char* str, uint16_t fg, uint16_t bg);

private:
    void writeCommand(uint8_t cmd);
    void writeDataByte(uint8_t data);
    void writeDataBytes(const uint8_t* data, size_t len);

    void assertCs();
    void deassertCs();

    SpiDisplayConfig cfg_{};
    spi_device_handle_t spi_dev_{nullptr};
    bool ready_{false};
    Orientation orientation_{Orientation::LANDSCAPE};
    uint16_t width_{320};
    uint16_t height_{240};
    uint16_t x_offset_{0};
    uint16_t y_offset_{0};
};

} // namespace display

#endif // ST7789_DRIVER_H
