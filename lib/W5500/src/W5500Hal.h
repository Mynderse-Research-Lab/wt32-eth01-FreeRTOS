// W5500 Hardware Abstraction Layer for host testing.
//
// Pure virtual interface for SPI register read/write. Real firmware uses
// W5500SpiHal (wraps W5500::readReg/writeReg). Host tests use W5500MockHal
// (scripted register values to verify socket state machine transitions).

#ifndef W5500_HAL_H
#define W5500_HAL_H

#include <cstdint>
#include <cstddef>

namespace w5500 {

class W5500Hal {
public:
    virtual ~W5500Hal() = default;

    virtual uint8_t  readReg(uint8_t blockSelect, uint16_t addr) = 0;
    virtual void     writeReg(uint8_t blockSelect, uint16_t addr, uint8_t value) = 0;
    virtual uint16_t readReg16(uint8_t blockSelect, uint16_t addr) = 0;
    virtual void     writeReg16(uint8_t blockSelect, uint16_t addr, uint16_t value) = 0;

    // Burst access to socket TX/RX buffer blocks (VDM auto-increment).
    virtual void readBuf(uint8_t blockSelect, uint16_t addr, uint8_t* buf,
                         uint16_t len) = 0;
    virtual void writeBuf(uint8_t blockSelect, uint16_t addr, const uint8_t* buf,
                          uint16_t len) = 0;
};

}  // namespace w5500

#endif  // W5500_HAL_H
