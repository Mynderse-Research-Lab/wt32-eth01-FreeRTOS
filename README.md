# WT32-ETH01 Gantry Controller

ESP-IDF firmware for a SCHUNK gantry pick-and-place controller on WT32-ETH01:
EtherNet/IP originator (W5500) for Kinetix X/Z, MQTT over LAN8720, and a serial
console for bring-up.

**Canonical docs:** see [`docs/INDEX.md`](docs/INDEX.md).

| Doc | Role |
|-----|------|
| [Expected electro-mechanical assembly](docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Mechanics / drives / networks |
| [HV/LV schematics](docs/HV_LV_SCHEMATICS.md) | Electrical design basis (preliminary) |
| [Low-level gantry control](docs/LOW_LEVEL_GANTRY_CONTROL.md) | Firmware / EIP / console |

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
