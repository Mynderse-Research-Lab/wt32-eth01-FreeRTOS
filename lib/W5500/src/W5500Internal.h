/**
 * @file W5500Internal.h
 * @brief Log tag and the frame sizes shared by the W5500 driver files.
 *
 * Private to this library. The SPI scratch buffers stay in W5500.cpp,
 * with the register and buffer path.
 */
#pragma once

static const char* TAG = "W5500";

static constexpr uint16_t kMaxFrameSize = 1536;
static constexpr uint16_t kHeaderSize   = 3;
// Without DMA the ESP32 SPI data buffer holds 64 bytes: header + 61 payload.
static constexpr uint16_t kNoDmaMaxTransfer = 64;
static constexpr uint16_t kNoDmaChunkPayload = kNoDmaMaxTransfer - kHeaderSize;
