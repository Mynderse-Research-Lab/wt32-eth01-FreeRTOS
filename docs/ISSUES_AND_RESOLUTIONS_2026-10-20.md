# Issues and Resolutions — Class 1 dropout campaign (2026-10-20 / 21)

Single summary of every issue found while eliminating the EtherNet/IP Class 1
connection drops on the WT32-ETH01 gantry, with the evidence and the resolution
for each. Policy throughout: **eliminate the drop at its root cause, never
mitigate it** (no auto-recover / resume). Detailed history stays in
`docs/DEV_TRACKER.md` (rows "Class 1 dropouts under fast Z motion" and
"MCP23S17").

Status key: **Fixed** (root cause removed, validated on hardware) ·
**Fixed, partial validation** · **Open** · **Tooling** (test-setup problem, not product).

Evidence labels: *log* = gantry console/boot log, *Saleae* = Logic Pro 16
capture (Logic 2 automation), *code* = source reference. Statements not backed
by a primary source are marked **Assumption**.

---

## Summary table

| # | Issue | Root cause | Resolution | Status |
|---|-------|-----------|------------|--------|
| 1 | Class 1 drops (X E602 / HCS01 F4009) during picks | Class 1 task stalled 20–43 ms by synchronous logging | Buffered UART log output (`cfg log_uart_async`, default 1) | Fixed |
| 2 | Residual stalls after #1 | `tcpip_thread` / `emac_rx` unpinned, preempting core 1 | Pinned to core 0 | Fixed |
| 3 | Stalls caused by the Class 1 loop's own logging | Deferred-log flush + diagnostics printed from the RT loop | `EipLog` task (core 0) prints; loop only records | Fixed |
| 4 | W5500 write corruption (TX_WR MISMATCH, buffer size CHANGED, register replay) | ESP32 SPI master: short TXDATA transfer after a DMA transfer re-sent stale DMA bytes | W5500 SPI2 runs without DMA (`eip_spi_dma` default 0) | Fixed |
| 5 | MCP23S17 "latch-up" after every software reset / OTA | Firmware address auto-detect picked a non-existent address | Use configured `MCP23S17_HW_ADDR`; no scan | Fixed |
| 6 | MCP lock failures (`SPI mutex lock failed`) | ~3000 MCP writes/s of TFT D/C toggling crowding SPI3 | Text drawn 8 chars per address window (~5x less MCP traffic) + failure-branch logging | Fixed, partial validation |
| 7 | Low Class 1 timing headroom (exchange p99 ≥ 2 ms) | No-DMA SPI2 at 10 MHz | SPI2 clock 20 MHz (`cfg eip_spi_khz 20000`) | Fixed |
| 8 | Earlier link-level false drops | Single LNK=0 read, PB7 rewrite, self-reset chip | Fixes A, C, D (2026-10-20) | Fixed |
| 9 | ethlink console command panicked the ESP | Explicit message during Class 1 without bus mutex | `spiBusMutex()` in `W5500::acquireBus/releaseBus` | Fixed |
| 10 | Boot-time Class 1 teardown | X still latched E602 from previous session at boot | Not a new fault; clean boot shows reliability 0 | Explained |
| 11 | θ (HCS01) T->O stale ~50 ms after every boot | Unknown | Per-axis reopen recovers it; no root cause yet | **Open** |
| 12 | OTA aborts / Ethernet TX wedges after OTA | Unknown; task_wdt shows `gantry_ota_srv` hogging core 0 during writes | Reset recovers; OTA retry succeeds | **Open** |
| 13 | Diagnostics perturbing the system (observer effect) | Logging in RT loop; FreeRTOS run-time stats + per-cycle snapshots | Probes removed / redesigned | Fixed (lesson) |
| 14 | Saleae / Logic 2 problems | Floating ground leads, CS probe unplugged, USB ReadTimeout, Logic 2 crash | Per-channel grounds, reconnect, relaunch | Tooling |
| 15 | Dev-page console stuck "busy" after `cfg reboot` | Supervisor waits for a reply that never comes | Send `stop`, then retry | Tooling |

---

## 1. Class 1 drops — task stalled by synchronous logging

**Symptom.** Drops every 1–9 min of picking: X latches E602 "Control connection
lost", HCS01 F4009; teardown + W5500 hard reset. All three drops on 2026-10-20
evening occurred at the same pick step (bin drop → path to X=320 + θ START).

**Evidence.**
- *log*: stale ages jumped 22.9 → 40.9 ms between consecutive checks; θ first
  reported at 30 ms (should be ~20 ms) → the loop itself was not running.
- *log* (loop-gap probe): `Class 1 loop gap 43622 us: exchange 1448, log flush 20,
  rest 42154`; W5500 bus mutex never held > 3.3 ms by another task.
- *code*: UART0 had no driver installed (`CONSOLE_UART_ENABLE` off,
  `CONFIG_ESP_CONSOLE_UART=y`), so every `ESP_LOG` busy-waited at 115200 baud
  (~22 ms per 250-char line) while holding stdout; the failing pick step logs
  ~1.5 kB in 200 ms.

**Resolution.** `src/main.cpp`: install the UART0 driver with a 16 KB TX ring and
`uart_vfs_dev_use_driver()` (`cfg log_uart_async`, default 1).

## 2. Network tasks preempting the Class 1 core

**Evidence.** After #1, gaps up to 26 ms remained; the scanner once held the
bus mutex for 11.3 ms (only possible if preempted mid-exchange). *sdkconfig*:
`CONFIG_LWIP_TCPIP_TASK_AFFINITY_NO_AFFINITY=y`; `emac_rx` created with
`tskNO_AFFINITY` (ESP-IDF `esp_eth_mac_esp.c`, flag `ETH_MAC_FLAG_PIN_TO_CORE`
not set).

**Resolution.** `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y` (sdkconfig.defaults);
`mac_config.flags |= ETH_MAC_FLAG_PIN_TO_CORE` in `EthernetLink.cpp` (created
from app_main → core 0). Validated: 24 min / 165+ picks, reliability counters 0.

## 3. Logging from the Class 1 loop itself

**Evidence.** 182 of 191 residual `rest` gaps followed one of the diagnostic
report lines within 130 ms (the loop's own `ESP_LOG` → stdout + LAN console
lwIP `send()`).

**Resolution.** `EipLog` task (core 0, prio 2) drains the deferred Class 1 event
ring and prints diagnostics; the loop only records (`EipScannerTask.cpp`).
Validated: 30 min / 263 picks, `soft_miss=0 sendok_fail=0 chip_recover=0
reconnect=0`.

## 4. W5500 SPI2 write corruption — ESP32 SPI DMA defect

**Symptom.** Snapshots showed `sw_wr` MISMATCH (TX_WR write lost), TX_WR high
byte landing in `Sn_TXBUF_SIZE` (`buf=…(CHANGED)`), overwritten socket register
blocks, and RX header "register replay". Splitting the TX_WR write
(`eip_txwr_split`) did not stop it.

**Evidence (Saleae, DMA on, stock TX_WR write, 2026-10-21 ~23:53).** Z socket 5
send on MOSI: `00 01 A8` CR/IR read ✓, `00 02 AC 10` IR clear ✓, 64-byte frame
at 0x5E80 ✓, `00 24 AC 5E C0` TX_WR ✓ — then the SEND command, a 4-byte TXDATA
transfer that must be `00 01 AC 20`, went out as **`00 24 AC 5E`** (first 4 bytes
of the previous DMA buffer). SEND never reached the chip; deferred SENDOK
timed out; drop. The W5500 executed exactly what it received. With DMA
re-enabled (ground bond in place) drops returned within 37–55 s.

**Resolution.** W5500 bus without DMA (`spi_bus_initialize(..., SPI_DMA_DISABLED)`,
bursts > 61 bytes split into separate VDM frames, `W5500.cpp`); `eip_spi_dma`
default changed to 0 (`AppConfigData.h`, `AppConfig.cpp`); `eip_txwr_split`
back to 0. Validated (10 MHz, picking): 30 min, reliability 0, 18,275 clean
read-backs.

**Assumption.** That this matches a documented ESP32 SPI erratum or ESP-IDF
driver issue has **not** been checked against Espressif's errata / issue
tracker; the conclusion rests on the bus capture only. To confirm: search the
ESP32 Series SoC Errata and ESP-IDF issues for "SPI DMA" + "TXDATA" stale data.

## 5. MCP23S17 dead after every software reset — address auto-detect bug

**Symptom.** After OTA / `cfg reboot` all MCP registers read 0x00 (or 0xFF);
gripper, inputs, W5500 RSTn (PB7) and TFT D/C dead; only a 3.3 V power cycle
recovered it (0/9 software resets survived).

**Wrong explanations ruled out (recorded so they are not revisited).**
- GPIO2 boot-strap theory — *Saleae*: MCP CS stays **high** through reset and
  ROM boot.
- Carrier ground bond — the Saleae grey leads "fixed" it (12/12) only because
  they changed where floating MISO settled. Not the cause.

**Evidence.** *log*: `MCP23S17 detected at address 0x21 (A2..A0 = 1)` with
A2..A0 strapped to GND; *Saleae*: every MCP frame used opcode `42/43`, reads
returned `00`. *code*: `mcp23s17_init` scanned addresses 0..7 and took the first
reading IODIRA = 0xFF (power-on default). After a software reset the chip keeps
IODIRA = 0xF0, so address 0 was skipped and a floating-MISO read of 0xFF at an
empty address won.

**Resolution.** `lib/MCP23S17/src/MCP23S17.cpp`: use `config->device_address`
(`MCP23S17_HW_ADDR` = 0); log the IODIRA value only. Proven by OTA-flashing the
fix onto a "dead" MCP with no power cycle → came up at 0x20
(`IODIRA=0xF0 (kept through software reset)`). OTAs and `cfg reboot` no longer
need a power cycle.

## 6. SPI3 contention — MCP lock failures

**Evidence.** *Saleae* 20 s: 60,929 MCP frames, 59,614 of them `40 15` OLATB
writes alternating `D0/F0` (TFT D/C pin PB5 through the MCP); TFT CS ~6000
transitions/s. *code*: `St7789Driver::drawString` opened one address window
(CASET/RASET/RAMWR = 6 D/C flips) per character. No stale-DMA bytes on SPI3
(all frames 24 bits, opcodes only 40/41).

**Resolution.** `drawString` renders up to 8 characters per window
(`drawTextChunk`); `mcp23s17_lock` failures now log the branch
(Class 1 idle wait timeout / mutex timeout / Class 1 re-entered on every retry).
Measured: MCP frames −78%, OLATB writes 5.2x fewer, GPIOB input reads +51%.
**Partial validation**: no lock failure seen on the new build so far, but not yet
over a long loaded run.

## 7. Class 1 timing headroom

**Evidence (eiptiming).** 10 MHz no-DMA: exchange p99 2066 µs (NO-GO), ~35 %
pace overruns. 16 MHz: p99 1309–1348 µs, ~10–20 % overruns. 20 MHz under real
picking (74–82 % of samples in motion): p50/p99 1056/1355 µs, ~18 % overruns,
11,522 read-backs 0 bad, reliability 0.

**Resolution.** `cfg eip_spi_khz 20000` (NVS). Note: the exchange is now
overhead-bound (per-transaction driver/CS cost), not bit-bound — 16→20 MHz
saved little. Further headroom would need fewer SPI transactions per cycle.

**Assumption.** MISO timing margin at 20 MHz on GPIO-matrix pins
(`input_delay_ns` not set) was judged only empirically (0 bad read-backs); the
W5500 output-valid time was not checked against the W5500 datasheet.

## 8. Earlier link / chip fixes (2026-10-20, before root cause)

- **A** — MCP PB7 (W5500 RSTn) released (input + pull-up) except during a
  commanded reset.
- **C** — link-down requires LNK=0 on 3 consecutive reads (single LNK=0 reads
  logged "not a link loss").
- **D** — a W5500 that lost MAC/IP (self-reset) gets its config restored via the
  existing chip recover; motion stays held.
- **B** (VDM address phase) was reverted the same day (chip never came up).

## 9. ethlink panic

A console explicit message during Class 1 hit `spi_device_release_bus` assert.
Fixed by taking `spiBusMutex()` inside `W5500::acquireBus/releaseBus`.

## 10. Boot-time Class 1 teardown

*bootlog*: X stale ~20 ms after cyclic start while X was clearing an E602
latched from the previous session. A reboot with no latched fault showed
`reliability soft_miss=0 … reconnect=0`. Explained, not a new fault. The new
`bootlog` console command (`src/boot_log_ring.cpp`, first 12 KB of logs) was
added to see such start-up events.

## 11. θ (HCS01) stale right after every boot — OPEN

Every boot logs `axis2 HCS01 T->O stale — X/Z stay up (age≈20 ms)` ~50 ms after
`Multi-axis Class 1 running`, then `axis2 silent … per-axis reopen` → reopen OK.
X/Z unaffected. Root cause not investigated.

## 12. OTA failures and Ethernet TX wedge — OPEN

**Evidence.**
- OTA results: `ERR WRITE_FAILED`, `ERR TIMEOUT`,
  `OTA socket recv error … errno 11` / `received 640064 / 815344 bytes`.
- *UART (Saleae)*: `task_wdt … IDLE0 (CPU 0) … CPU 0: gantry_ota_srv` during
  image writes; Class 1 drops and W5500 recovers repeatedly during the write
  (expected while flash ops stall the CPU).
- Twice after a failed / interrupted OTA the gantry's Ethernet **transmit** path
  wedged: no ping, ports closed, supervisor offline — while CellNet RX still
  arrived (vision frames logged) and core 1 (Class 1, PickScheduler) ran.
  Recovered only by a reset.

**Workaround in use (not a fix).** Reset, wait ≥ 1 min after boot, retry OTA.
**Next steps.** Review `gantry_ota.cpp` (task priority/core, recv timeout,
yielding during `esp_ota_write`), and EMAC TX handling around flash operations.
**Assumption.** The link between flash writes and the TX wedge is inferred from
timing only.

## 13. Observer effect — diagnostics that caused the problem

- Logging from the RT loop (see #3).
- FreeRTOS run-time stats + `uxTaskGetSystemState()` every 2 ms cycle produced
  27–48 ms pace stalls and a full drop within ~4 min → reverted
  (`CONFIG_FREERTOS_USE_TRACE_FACILITY` / `GENERATE_RUN_TIME_STATS` off).
- Safe pattern kept: record into RAM in the loop (timed bus mutex, loop-gap
  record under a spinlock), print from `EipLog`.

## 14. Saleae / Logic 2 tooling

- Floating readings (60 Hz hum, million edges/s): ground not referenced → clip
  each channel's grey lead to carrier GND at the chip.
- Grey harness leads are ground — never clip one onto a signal.
- CS probe unplugged once (decoded SPI2 by SCLK gaps instead, `spidecode_gap.py`).
- `DeviceError: ReadTimeout` on long rolling captures; Logic 2 crashed once →
  relaunch with `Logic.exe --automation`; 50 MS/s and 60 s segments are lighter.
- Logic 2 rejects a capture while its own live view is recording.
- ch7 (UART TX) lead appeared to drop off GPIO1 during the last OTA capture
  (line flat low for 66 s across a reboot).

## 15. Dev-page console "busy"

After `cfg reboot` the supervisor console stays "busy" waiting for a reply.
Send `stop`, then retry (note: `stop` also disables the motors).

---

## Current configuration (as of 2026-10-21 ~03:10)

| Setting | Value |
|---------|-------|
| `eip_spi_dma` | 0 (firmware default) |
| `eip_txwr_split` | 0 |
| `eip_spi_khz` | 20000 |
| `log_uart_async` | 1 (default) |
| `disp_tft_en` | 1 (TFT stays enabled) |
| `eip_scope_gpio` | -1 (off) |
| `pk_th_spd` | 270 deg/s |

## Open follow-ups

1. θ boot-time stale (#11).
2. OTA abort / Ethernet TX wedge (#12).
3. Long loaded run on the latest build (SPI3 batching, MCP fix, new speeds).
4. Remove leftover experiment code once settled: `eip_txwr_split`, `eip_scope_gpio`,
   destination check, loop-gap / bus-mutex probes (or keep as permanent diagnostics).
5. Remove the Saleae; the grey-lead "ground bond" is not required for the MCP.
6. Verify the assumptions marked above against primary sources (ESP32 errata,
   W5500 datasheet timing).
