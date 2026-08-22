// W5500SpiHal — delegates to W5500 private register methods.

#include "W5500SpiHal.h"
#include "W5500.h"

uint8_t W5500SpiHal::readReg(uint8_t blockSelect, uint16_t addr) {
    return w5500_.readReg(blockSelect, addr);
}

void W5500SpiHal::writeReg(uint8_t blockSelect, uint16_t addr, uint8_t value) {
    w5500_.writeReg(blockSelect, addr, value);
}

uint16_t W5500SpiHal::readReg16(uint8_t blockSelect, uint16_t addr) {
    return w5500_.readReg16(blockSelect, addr);
}

void W5500SpiHal::writeReg16(uint8_t blockSelect, uint16_t addr, uint16_t value) {
    w5500_.writeReg16(blockSelect, addr, value);
}

void W5500SpiHal::readBuf(uint8_t blockSelect, uint16_t addr, uint8_t* buf,
                          uint16_t len) {
    w5500_.readBuf(blockSelect, addr, buf, len);
}

void W5500SpiHal::writeBuf(uint8_t blockSelect, uint16_t addr,
                           const uint8_t* buf, uint16_t len) {
    w5500_.writeBuf(blockSelect, addr, buf, len);
}
