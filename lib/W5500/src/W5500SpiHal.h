// W5500SpiHal — concrete W5500Hal implementation wrapping the W5500 SPI class.
//
// Provides register read/write access through the existing W5500 hardware
// driver. Friend of W5500 to access private readReg/writeReg methods.
//
// NOTE: Declared at global scope (not in w5500 namespace) because W5500.h
// wraps its class in extern "C" which prohibits namespace references in
// friend declarations.

#ifndef W5500_SPI_HAL_H
#define W5500_SPI_HAL_H

#include "W5500Hal.h"

class W5500;  // forward

// Global scope — W5500 is also global (inside extern "C" block in W5500.h)
class W5500SpiHal : public w5500::W5500Hal {
public:
    explicit W5500SpiHal(W5500& w5500) : w5500_(w5500) {}

    uint8_t  readReg(uint8_t blockSelect, uint16_t addr) override;
    void     writeReg(uint8_t blockSelect, uint16_t addr, uint8_t value) override;
    uint16_t readReg16(uint8_t blockSelect, uint16_t addr) override;
    void     writeReg16(uint8_t blockSelect, uint16_t addr, uint16_t value) override;
    void     readBuf(uint8_t blockSelect, uint16_t addr, uint8_t* buf,
                     uint16_t len) override;
    void     writeBuf(uint8_t blockSelect, uint16_t addr, const uint8_t* buf,
                      uint16_t len) override;

private:
    W5500& w5500_;
};

#endif  // W5500_SPI_HAL_H
