#include "St7789Driver.h"
#include "TftFont.h"
#include "Spi3Bus.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstring>
#include <algorithm>

namespace display {
namespace {
const char* TAG = "St7789";

// ST7789 commands
constexpr uint8_t ST7789_SWRESET = 0x01;
constexpr uint8_t ST7789_SLPOUT  = 0x11;
constexpr uint8_t ST7789_NORON   = 0x13;
constexpr uint8_t ST7789_INVON   = 0x21;
constexpr uint8_t ST7789_DISPON  = 0x29;
constexpr uint8_t ST7789_CASET   = 0x2A;
constexpr uint8_t ST7789_RASET   = 0x2B;
constexpr uint8_t ST7789_RAMWR   = 0x2C;
constexpr uint8_t ST7789_MADCTL  = 0x36;
constexpr uint8_t ST7789_COLMOD  = 0x3A;

// MADCTL bit definitions
constexpr uint8_t MADCTL_MY  = 0x80;
constexpr uint8_t MADCTL_MX  = 0x40;
constexpr uint8_t MADCTL_MV  = 0x20;
constexpr uint8_t MADCTL_ML  = 0x10;
constexpr uint8_t MADCTL_BGR = 0x08;
constexpr uint8_t MADCTL_RGB = 0x00;
} // namespace

St7789Driver::St7789Driver() = default;
St7789Driver::~St7789Driver() {
    if (spi_dev_) {
        spi_bus_remove_device(spi_dev_);
        spi_dev_ = nullptr;
    }
}

void St7789Driver::assertCs() {
    if (cfg_.esp_cs_pin >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(cfg_.esp_cs_pin), 0);
    } else if (cfg_.mcp && cfg_.mcp_cs_pin >= 0) {
        mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_cs_pin), 0);
    }
}

void St7789Driver::deassertCs() {
    if (cfg_.esp_cs_pin >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(cfg_.esp_cs_pin), 1);
    } else if (cfg_.mcp && cfg_.mcp_cs_pin >= 0) {
        mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_cs_pin), 1);
    }
}

void St7789Driver::writeCommand(uint8_t cmd) {
    if (!spi_dev_ || cfg_.mcp == nullptr) return;
    (void)spi3::withTft([this, cmd]() -> esp_err_t {
        // DC = 0 (command)
        mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_dc_pin), 0);
        assertCs();

        spi_transaction_t t = {};
        t.length = 8;
        t.tx_buffer = &cmd;
        esp_err_t e = spi_device_polling_transmit(spi_dev_, &t);

        deassertCs();
        return e;
    });
}

void St7789Driver::writeDataByte(uint8_t data) {
    writeDataBytes(&data, 1);
}

void St7789Driver::writeDataBytes(const uint8_t* data, size_t len) {
    if (!spi_dev_ || cfg_.mcp == nullptr || !data || len == 0) return;
    (void)spi3::withTft([this, data, len]() -> esp_err_t {
        // DC = 1 (data)
        mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_dc_pin), 1);
        assertCs();

        spi_transaction_t t = {};
        t.length = len * 8;
        t.tx_buffer = data;
        esp_err_t e = spi_device_polling_transmit(spi_dev_, &t);

        deassertCs();
        return e;
    });
}

bool St7789Driver::init(const SpiDisplayConfig& cfg) {
    cfg_ = cfg;
    ready_ = false;

    if (!spi3::isReady() || cfg_.mcp == nullptr || (cfg_.mcp_cs_pin < 0 && cfg_.esp_cs_pin < 0)) {
        ESP_LOGE(TAG, "Cannot init ST7789: SPI3 or CS pin not configured");
        return false;
    }

    if (cfg_.esp_cs_pin >= 0) {
        gpio_set_direction(static_cast<gpio_num_t>(cfg_.esp_cs_pin), GPIO_MODE_OUTPUT);
        gpio_set_level(static_cast<gpio_num_t>(cfg_.esp_cs_pin), 1);
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = cfg_.clock_hz > 0 ? cfg_.clock_hz : 20000000;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = -1; // Software CS via MCP23S17 or direct GPIO
    dev_cfg.queue_size = 1;
    dev_cfg.flags = SPI_DEVICE_NO_DUMMY;

    esp_err_t err = spi_bus_add_device(spi3::host(), &dev_cfg, &spi_dev_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return false;
    }

    // Hardware reset pulse via MCP23S17 RES pin
    if (cfg_.mcp_res_pin >= 0) {
        (void)spi3::withMcp([this]() -> esp_err_t {
            mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_res_pin), 0);
            return ESP_OK;
        });
        vTaskDelay(pdMS_TO_TICKS(20));
        (void)spi3::withMcp([this]() -> esp_err_t {
            mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_res_pin), 1);
            return ESP_OK;
        });
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    // ST7789 initialization sequence
    writeCommand(ST7789_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));

    writeCommand(ST7789_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));

    // 16-bit RGB565 pixel format
    writeCommand(ST7789_COLMOD);
    writeDataByte(0x55);
    vTaskDelay(pdMS_TO_TICKS(10));

    setOrientation(Orientation::LANDSCAPE);

    writeCommand(ST7789_INVON);
    vTaskDelay(pdMS_TO_TICKS(10));

    writeCommand(ST7789_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));

    writeCommand(ST7789_DISPON);
    vTaskDelay(pdMS_TO_TICKS(100));

    ready_ = true;
    fillScreen(COLOR_BLACK);

    ESP_LOGI(TAG, "ST7789 320x240 display initialized successfully");
    return true;
}

void St7789Driver::setOrientation(Orientation orient) {
    orientation_ = orient;
    uint8_t madctl = 0;

    switch (orient) {
        case Orientation::PORTRAIT:
            madctl = MADCTL_RGB;
            width_ = 240;
            height_ = 320;
            x_offset_ = 0;
            y_offset_ = 0;
            break;
        case Orientation::LANDSCAPE:
            madctl = MADCTL_MV | MADCTL_MX | MADCTL_RGB;
            width_ = 320;
            height_ = 240;
            x_offset_ = 0;
            y_offset_ = 0;
            break;
        case Orientation::PORTRAIT_INV:
            madctl = MADCTL_MY | MADCTL_MX | MADCTL_RGB;
            width_ = 240;
            height_ = 320;
            x_offset_ = 0;
            y_offset_ = 0;
            break;
        case Orientation::LANDSCAPE_INV:
            madctl = MADCTL_MV | MADCTL_MY | MADCTL_RGB;
            width_ = 320;
            height_ = 240;
            x_offset_ = 0;
            y_offset_ = 0;
            break;
    }

    writeCommand(ST7789_MADCTL);
    writeDataByte(madctl);
}

void St7789Driver::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint16_t xa0 = x0 + x_offset_;
    uint16_t xa1 = x1 + x_offset_;
    uint16_t ya0 = y0 + y_offset_;
    uint16_t ya1 = y1 + y_offset_;

    uint8_t x_buf[4] = {
        static_cast<uint8_t>(xa0 >> 8), static_cast<uint8_t>(xa0 & 0xFF),
        static_cast<uint8_t>(xa1 >> 8), static_cast<uint8_t>(xa1 & 0xFF)
    };
    uint8_t y_buf[4] = {
        static_cast<uint8_t>(ya0 >> 8), static_cast<uint8_t>(ya0 & 0xFF),
        static_cast<uint8_t>(ya1 >> 8), static_cast<uint8_t>(ya1 & 0xFF)
    };

    writeCommand(ST7789_CASET);
    writeDataBytes(x_buf, 4);

    writeCommand(ST7789_RASET);
    writeDataBytes(y_buf, 4);

    writeCommand(ST7789_RAMWR);
}

void St7789Driver::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (!ready_ || w <= 0 || h <= 0) return;
    if (x >= width_ || y >= height_) return;

    int16_t x1 = x + w - 1;
    int16_t y1 = y + h - 1;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x1 >= width_) x1 = width_ - 1;
    if (y1 >= height_) y1 = height_ - 1;

    uint32_t total_pixels = static_cast<uint32_t>(x1 - x + 1) * static_cast<uint32_t>(y1 - y + 1);
    setAddrWindow(static_cast<uint16_t>(x), static_cast<uint16_t>(y),
                  static_cast<uint16_t>(x1), static_cast<uint16_t>(y1));

    // Buffer 512 pixels (1024 bytes) of the color in big-endian RGB565
    constexpr size_t BATCH_PIXELS = 512;
    uint8_t buf[BATCH_PIXELS * 2];
    uint8_t hi = static_cast<uint8_t>(color >> 8);
    uint8_t lo = static_cast<uint8_t>(color & 0xFF);
    for (size_t i = 0; i < BATCH_PIXELS; ++i) {
        buf[i * 2] = hi;
        buf[i * 2 + 1] = lo;
    }

    (void)spi3::withTft([this, &buf, total_pixels]() -> esp_err_t {
        // Assert DC = 1 (data), assert CS
        mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_dc_pin), 1);
        assertCs();

        uint32_t remaining = total_pixels;
        while (remaining > 0) {
            uint32_t to_send = std::min(remaining, static_cast<uint32_t>(BATCH_PIXELS));
            spi_transaction_t t = {};
            t.length = to_send * 16;
            t.tx_buffer = buf;
            esp_err_t e = spi_device_polling_transmit(spi_dev_, &t);
            if (e != ESP_OK) {
                deassertCs();
                return e;
            }
            remaining -= to_send;
        }

        deassertCs();
        return ESP_OK;
    });
}

void St7789Driver::fillScreen(uint16_t color) {
    fillRect(0, 0, width_, height_, color);
}

void St7789Driver::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
    fillRect(x, y, 1, 1, color);
}

void St7789Driver::drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    fillRect(x, y, w, 1, color);
}

void St7789Driver::drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    fillRect(x, y, 1, h, color);
}

void St7789Driver::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    drawHLine(x, y, w, color);
    drawHLine(x, y + h - 1, w, color);
    drawVLine(x, y, h, color);
    drawVLine(x + w - 1, y, h, color);
}

void St7789Driver::drawChar(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg) {
    if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
        c = ' ';
    }
    const uint8_t* glyph = font8x16[c - FONT_FIRST_CHAR];

    if (x < 0 || x + FONT_WIDTH > width_ || y < 0 || y + FONT_HEIGHT > height_) {
        // Clipped fallback pixel-by-pixel
        for (uint8_t row = 0; row < FONT_HEIGHT; ++row) {
            uint8_t line = glyph[row];
            for (uint8_t col = 0; col < FONT_WIDTH; ++col) {
                if (line & (0x80 >> col)) {
                    drawPixel(x + col, y + row, fg);
                } else if (bg != fg) {
                    drawPixel(x + col, y + row, bg);
                }
            }
        }
        return;
    }

    setAddrWindow(x, y, x + FONT_WIDTH - 1, y + FONT_HEIGHT - 1);

    // 8x16 = 128 pixels (256 bytes)
    uint8_t buf[FONT_WIDTH * FONT_HEIGHT * 2];
    uint8_t fg_hi = static_cast<uint8_t>(fg >> 8);
    uint8_t fg_lo = static_cast<uint8_t>(fg & 0xFF);
    uint8_t bg_hi = static_cast<uint8_t>(bg >> 8);
    uint8_t bg_lo = static_cast<uint8_t>(bg & 0xFF);

    size_t idx = 0;
    for (uint8_t row = 0; row < FONT_HEIGHT; ++row) {
        uint8_t line = glyph[row];
        for (uint8_t col = 0; col < FONT_WIDTH; ++col) {
            bool on = (line & (0x80 >> col)) != 0;
            buf[idx++] = on ? fg_hi : bg_hi;
            buf[idx++] = on ? fg_lo : bg_lo;
        }
    }

    (void)spi3::withTft([this, &buf]() -> esp_err_t {
        mcp23s17_write_pin(cfg_.mcp, static_cast<mcp23s17_pin_t>(cfg_.mcp_dc_pin), 1);
        assertCs();

        spi_transaction_t t = {};
        t.length = sizeof(buf) * 8;
        t.tx_buffer = buf;
        esp_err_t e = spi_device_polling_transmit(spi_dev_, &t);

        deassertCs();
        return e;
    });
}

void St7789Driver::drawString(int16_t x, int16_t y, const char* str, uint16_t fg, uint16_t bg) {
    if (!str) return;
    int16_t cur_x = x;
    while (*str) {
        if (*str == '\n') {
            cur_x = x;
            y += FONT_HEIGHT;
        } else {
            drawChar(cur_x, y, *str, fg, bg);
            cur_x += FONT_WIDTH;
        }
        str++;
    }
}

void St7789Driver::drawStringCentered(int16_t y, const char* str, uint16_t fg, uint16_t bg) {
    if (!str) return;
    size_t len = strlen(str);
    int16_t total_w = static_cast<int16_t>(len * FONT_WIDTH);
    int16_t x = (width_ - total_w) / 2;
    if (x < 0) x = 0;
    drawString(x, y, str, fg, bg);
}

} // namespace display
