#ifndef SPI_DISPLAY_H
#define SPI_DISPLAY_H

#include "MCP23S17.h"
#include "driver/spi_master.h"

#include <cstdint>

/**
 * @file SpiDisplay.h
 * @brief Minimal SPI TFT stub on shared SPI3 (MCP CS/DC/RES/BLK).
 *
 * TFT CS is an MCP GPIO (software CS). ESP SPI device uses spics_io_num=-1.
 */

namespace display {

struct SpiDisplayConfig {
    mcp23s17_handle_t mcp = nullptr;
    int mcp_cs_pin = -1;
    int mcp_dc_pin = -1;
    int mcp_res_pin = -1;
    int mcp_blk_pin = -1;
    uint32_t clock_hz = 20000000;
};

class SpiDisplay {
 public:
    SpiDisplay() = default;

    bool begin(const SpiDisplayConfig& cfg);
    bool isReady() const { return ready_; }

    bool refreshStub();

    const SpiDisplayConfig& config() const { return cfg_; }

 private:
    SpiDisplayConfig cfg_{};
    spi_device_handle_t tft_dev_ = nullptr;
    bool ready_ = false;
};

}  // namespace display

#endif  // SPI_DISPLAY_H
