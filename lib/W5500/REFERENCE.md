*[README](../../README.md) › [Knowledge transfer](../../docs/KNOWLEDGE_TRANSFER.md) › [Gantry firmware](../../GANTRY_FIRMWARE.md) › [Libraries](../README.md) › [W5500](README.md) › Reference*

# W5500 reference

**Read this when** you need an exact fact: a signature, a unit, a timeout, a key or a return value.

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

## Known issues

Found in the library review of 2026-11-26 (at `1caa98c`) and not tracked as open items. Not re-checked since; confirm against the code before acting.

| # | Category | Location | Finding | Evidence |
|---|---|---|---|---|
| W3 | bug, not in production | `W5500.cpp:198-291` | MACRAW writes the TX buffer and reads the RX buffer at address 0 without using `Sn_TX_WR`/`Sn_RX_RD`. `recvFrame` underflows on a length below 2. `sendFrame` and `waitForCommand` truncate `esp_timer_get_time()` to 32 bits (every wait expires at once after 71.6 min). Used only by `test/w5500_loopback`. Whether the chip requires the pointer offset is not checked against the datasheet | Confirmed (code) |
| W7 | risk | `W5500.cpp:22`, `W5500Init.cpp:90` | With DMA on, `max_transfer_sz` is 1,539 while bursts go up to 2,048 bytes. A TCP receive of 1,537–2,048 bytes would exceed the bus limit. Only matters with `eip_spi_dma=1` | Confirmed (code); IDF behaviour Suggested |
| W9 | bug | `W5500SocketTcp.cpp:415-421` | TCP `socketRecv` returns -1 on CLOSE_WAIT before reading `RX_RSR`, so a reply followed by FIN is lost | Confirmed |
| W11 | docs | `W5500.h:164`, `W5500Socket.h:8-10, 32, 55-57, 91-95, 110-111` | Stale or wrong comments: "factory-burned MAC" (the code writes 02:00:00:00:00:01), "yield" polling, the "all functions lock" claim, the UDP ephemeral port, the -1 return, and "oversized datagrams stay queued" (they are truncated and consumed, unflagged) | Confirmed |
| W12 | cleanup | `W5500.h:7-9`, `W5500Init.cpp:136-138`, `W5500.cpp:311`, `W5500SocketTcp.cpp:208` | `extern "C"` around C++; literal 0x002E instead of `REG_PHYCFGR`; `socketClose` indexes without `sock < 8`; a failed `init` leaks the device | Confirmed |

**Dead code:** `socketListen`, `socketBind`, `socketIsOpen`, `socketStatus`, `nextEphemeralPort`, `MR_WOL`/`MR_PB`, and the MACRAW path (used only by `test/w5500_loopback`).

**Not covered by host tests:** the 16-bit wrap (W1, W2); MISO stuck high on the receive paths; TCP `socketRecv`/`socketConnect`/`socketClose`; the 61-byte no-DMA chunking; all of `W5500.cpp`. `test_integration_macraw_send` encodes the MACRAW addressing at address 0.

## SPI mode: VDM, not FDM

The W5500 SPI frame is Address + Control + Data. Control bits `OM[1:0]` select:

| Mode | OM | Data phase | SCSn |
|------|-----|------------|------|
| **VDM** (firmware today) | `00` | Arbitrary N bytes | **Must toggle per frame** (edges delimit start/end) |
| FDM | `01` / `10` / `11` | Fixed **1 / 2 / 4** bytes only | May stay low |

**CIP Class 1 sizes are not FDM sizes.** FDM’s 1/2/4-byte limit is the **SPI data phase to the chip**, not the UDP/CIP payload on the wire. Assemblies and CPF still ship at full size; FDM only **chunks** the SPI copy into many tiny transactions.

| Traffic | Size | Fits in one FDM frame? |
|---------|------|------------------------|
| Kinetix O→T assy 104 | 40 B | No |
| Kinetix T→O assy 154 | 52 B | No |
| Class 1 O→T CPF UDP (~104 + seq + run/idle) | ~64 B | No |
| Class 1 T→O CPF UDP (~154) | ~72 B | No |
| HCS01 cmd / actual | 18 / 14 B | No |
| Explicit CIP / Forward Open (TCP) | tens–hundreds B | No |
| W5500 register R/W | 1–2 B (MAC 6, IP 4) | Yes (per SPI frame) |

For buffer copies the largest useful FDM length is **4 bytes** (`OM=11`). Register writes still need **OM=1 or 2** — using a 4-byte data phase for a 1-byte register would overwrite adjacent registers.

**Hardwiring SCSn low (sole SPI2 device) is invalid under VDM.** Symptom seen on bench: `VERSIONR=0xFF` (MISO idle-high / frames not delimited). FDM rewrite could free GPIO15, but:

- Each ~64–80 B UDP payload becomes **⌈N/4⌉** SPI frames (~1.7× bit traffic plus far more `spi_device_transmit` calls).
- Dual X+Z `exchangeOnce` SPI call count rises from ~O(10) to ~O(70+); driver overhead dominates bit time at 20 MHz.
- Class 1 does **not** become more deterministic — more SPI calls increase FreeRTOS/IRQ jitter exposure and shrink RPI margin.

| Setup | Practical RPI floor if FDM-4 were used |
|-------|----------------------------------------|
| Single axis | **≥ ~2 ms** (1 ms / 500 µs gate likely fails) |
| Dual X+Z | **≥ ~4–5 ms** with SPI3 contention |
| + Theta / TCP reconnect | **≥ ~5–10 ms** during FO / explicit |

**Decision:** stay on **VDM + CS GPIO15**. Class 1 UDP SENDOK/CR waits must **busy-spin** (no `taskYIELD` on short polls) — at FreeRTOS 1000 Hz a yield costs ~1 ms and dual-axis O→T was measured at ~3 ms from yields alone. SPI uses **`spi_device_polling_transmit`** with the bus acquired for the whole exchange (ISR path was ~40–60 µs per register). O→T uses dest caching + burst DIPR/DPORT; exchange is **produce-then-drain** (no blocking wait for T→O phase). Target: exchange p99 **under RPI** (default **2 ms** with `CONFIG_EIP_X_RPI_US=2000`).

## Documents

Class 1 use of these sockets is the [Class 1 lifecycle](../EtherNetIP/REFERENCE.md#class-1-lifecycle). Open items: G-B5, G-B16, G-B17, G-B18 in `docs/OPEN_ITEMS.md`; G-B15 is closed.

## Next

- **Up:** [W5500 README](README.md), [Libraries](../README.md).
