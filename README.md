# WT32-ETH01 Gantry Controller

ESP-IDF firmware for the SCHUNK multi-axis gantry pick-and-place motion controller on WT32-ETH01.
Part of the distributed battery sorting cell:
- **Motion Bus (W5500 SPI):** Hard real-time EtherNet/IP Class 1 originator ($2\text{ ms}$ RPI) for Kinetix X/Z and Rexroth Theta drives.
- **Closed Cell Bus (LAN8720 RMII):** High-speed peer-to-peer messaging (CellNet Layer-2 `0x88B5` / Zenoh-DDS) connecting Ubuntu Vision, Conveyor WT32, Gantry, and Raspberry Pi Gateway.
- **Diagnostics:** UART0 console + secure TCP line console on port `2323`.

**Canonical docs:** see [`docs/INDEX.md`](docs/INDEX.md).

| Doc | Role |
|-----|------|
| [Expected electro-mechanical assembly](docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Mechanics / drives / networks |
| [HV/LV schematics](docs/HV_LV_SCHEMATICS.md) | Electrical design basis (preliminary) |
| [Low-level gantry control](docs/LOW_LEVEL_GANTRY_CONTROL.md) | Firmware / EIP / console |
| [CellNet Layer-2 Communication Guide](docs/CELL_NET_L2_COMMUNICATION_GUIDE.md) | OSI Layer-2 real-time protocol & deployment manual |
| [Pick Scheduler Kinematics](docs/PICK_SCHEDULER_KINEMATICS.md) | Authoritative coordinate frames & timing math |
| [Firmware Review & Optimizations](docs/FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md) | Quality, integrity, and 8-stage verification test cycle |
| [Development Roadmap](docs/DEVELOPMENT_ROADMAP.md) | Architecture phases & Zenoh-DDS migration |

## Console

- **UART0** — flash and pre-link logs (USB–TTL).
- **TCP** on LAN8720 — after plant link is up, gantry listens at
  **`192.168.1.100:2323`**. Password / remember TTL editable under menuconfig
  **TCP gantry console (LAN8720)** (default password `LTU_1932`, 600 s / 4 IPs).

```powershell
ncat 192.168.1.100 2323
# enter password if prompted
# PuTTY: Raw → 192.168.1.100 port 2323
```

```bash
nc 192.168.1.100 2323
```

Details: [LOW_LEVEL §9](docs/LOW_LEVEL_GANTRY_CONTROL.md).

## Build & flash (ESP-IDF)

```powershell
cd idf
idf.py set-target esp32   # first time only
idf.py build
idf.py -p COMx flash monitor
```

Host tests (required before merge):

```powershell
cmake -S test/host -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

See [`CONTRIBUTING.md`](CONTRIBUTING.md) and [`AGENTS.md`](AGENTS.md).

## Entering programming mode (WT32-ETH01)

You will need a USB–TTL adapter (e.g. Waveshare USB to RS232/RS485/TTL).

1. Power: 5 V to pin 12; GND to board ground and pin 11.
2. UART: adapter RX0 ↔ WT32 TX0; adapter TX0 ↔ WT32 RX0.
3. Boot: jumper pin 24 (IO0) to pin 23 (GND).
4. Reset: briefly ground pin 1 (EN), then remove the IO0 jumper.
5. Flash, then ground EN again to run normally.

![Programming wiring](<Screenshot from 2025-06-28 18-05-10-1.png>)

## CI

GitHub Actions (`.github/workflows/firmware-ci.yml`) runs host tests and the
ESP-IDF build on push / PR.
