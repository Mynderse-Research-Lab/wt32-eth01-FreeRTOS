# W5500 reference

Public headers under `lib/W5500/src/`: `W5500.h`, `W5500Hal.h`, `W5500SpiHal.h`, `W5500Socket.h`, `W5500BusMutex.h`, `W5500DeferredLog.h`, `W5500Diag.h`, `W5500Poll.h`.

## API

```cpp
class W5500 {                     // W5500.h
public:
    bool init(const W5500Config& cfg);
    bool recover();
    void deinit();
    bool isLinkUp();
    uint8_t getVersion();
    void getMacAddress(uint8_t mac[6]);
    bool acquireBus();
    void releaseBus();
    bool openMacRaw(uint8_t sock = 0);
    void closeSocket(uint8_t sock = 0);
    int sendFrame(const uint8_t* data, uint16_t len);
    int recvFrame(uint8_t* buf, uint16_t bufSize);
};
```

`W5500Config` holds the SPI host, the CS, INT, RST, MOSI, MISO and SCLK GPIOs, the SPI clock in Hz, an optional reset callback (`rst_set_level`, `rst_ctx`) and the DMA choice. The socket layer is free functions in `W5500Socket.h` (namespace `w5500`) on a `W5500Hal`: `socketOpen`, `socketClose`, `socketConnect`, `socketListen`, `socketBind`, `socketSend`, `socketSendNoWait`, `socketRecv`, `socketSendTo`, `socketRecvFrom`, `socketRecvFromNonBlocking`, `socketIsConnected`, `socketIsOpen`, `socketRxAvailable`.

## Chip

`W5500::init`, `recover`, `deinit`, `isLinkUp`. `recover` takes `spiBusMutex`, closes every socket, pulses the reset, and calls `configureAfterReset` (`W5500Init.cpp:271–293`).

MAC written to SHAR is `02:00:00:00:00:01` (`W5500Init.cpp:221–224`). The comment in `W5500.h` that says the MAC is factory-burned does not match this write.

IP, subnet and gateway are parsed from `CONFIG_EIP_W5500_IP`, `CONFIG_EIP_W5500_SUBNET` and `CONFIG_EIP_W5500_GATEWAY` when the scanner is enabled (`W5500Init.cpp:229–243`). No AppConfig key is read in this component. G-B5.

Socket buffers, both RX and TX, in KB (`W5500Init.cpp:255–258`): 4, 2, 2, 2, 1, 1, 1, 1. The sum is 14 KB of the 16 KB on the chip.

If VERSIONR is not `0x04`, `configureAfterReset` retries the SPI clock at 5 MHz, then 2 MHz, then 1 MHz, and keeps the first rate that reads `0x04` (`W5500Init.cpp:176–212`). `recover` uses that same function. G-B18.

## UDP length

The datagram length is the 16-bit big-endian field at header bytes 6 and 7 (`W5500Socket.cpp:257–259` and `310–311`). The size compared with the bytes available is a `uint32_t`:

```c
const uint32_t packetBytes = kUdpHeaderSize + static_cast<uint32_t>(payloadLen);
```

`kUdpHeaderSize` is 8 (`W5500SocketInternal.h:50`). A length of `0xFFF8` or more does not wrap that sum (`W5500Socket.cpp:260–261` and `312–314`). G-B15 records the earlier 16-bit sum; it was closed on 2026-11-27.

`readUdpDatagramUnlocked` peeks at most 8 + 256 bytes (`kUdpFastBurstMax`). `readUdpBatchUnlocked` refuses a buffer longer than `0xFFFF` and treats a length that does not fit the burst as malformed when the burst was the whole RX.

## Timeouts

From `W5500Poll.h`:

| Constant | Value |
|---|---|
| `kUdpSendOkTimeoutUs` | 2,000 µs |
| `kUdpDestChangeSendOkTimeoutUs` | 10,000 µs |
| `kTcpSendOkTimeoutUs` | 50,000 µs |
| `kUdpRecvCmdTimeoutUs` | 2,000 µs |
| `kBusyPollYieldAboveUs` | 50,000 µs. Longer waits call `taskYIELD` |

## Bus mutex

`w5500::spiBusMutex()` is one `std::mutex` for socket-layer SPI. `BusMutex` records the longest hold and the longest wait. Do not call a logging function that can block while the mutex is held. G-B16. `diagMutex` is a separate `std::mutex`. G-B17.

## Deferred log

`W5500DeferredLog`: 16 lines of 160 bytes (`W5500DeferredLog.h:18–19`). The EtherNet/IP log task drains it. The scanner must not print from inside the exchange.

## Console

None in this component. `gaptrace` and `eiptiming` print snapshots this driver captured; the commands live in `src/gantry_console_commands.cpp`.

## AppConfig

None read here. `bootInitMotion` applies the stored keys `eip_spi_khz`, `eip_spi_dma` and `eip_txwr_split` to `W5500Config` before `init`.

## Errors

`init` returns false when the SPI bus or device cannot be set up, or when the chip does not read VERSIONR `0x04` at any tried clock (`W5500Init.cpp:76–138, 216`). `recover` returns false before `init` and when VERSIONR fails again (`W5500Init.cpp:274–289`). The socket calls return a negative value on failure, and the `bool` ones false.

## Usage

```cpp
// src/main_init_motion.cpp
static W5500 w5500;
W5500Config cfg = {};
cfg.spi_host = W5500_SPI_HOST;
cfg.cs_gpio = W5500_CS_GPIO;
cfg.rst_gpio = W5500_RST_GPIO;   // -1: reset through MCP PB7
cfg.sclk_hz = W5500_SCLK_HZ;
if (w5500.init(cfg)) {
    // hand the chip to eip::startScannerTask
}
```

## Documents

Class 1 use of these sockets is `LOW_LEVEL_GANTRY_CONTROL.md`. Open items: G-B5, G-B16, G-B17, G-B18 in `docs/OPEN_ITEMS.md`; G-B15 is closed.
