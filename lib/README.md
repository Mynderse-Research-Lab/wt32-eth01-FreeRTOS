*[README](../README.md) › [Knowledge transfer](../docs/KNOWLEDGE_TRANSFER.md) › [Gantry firmware](../GANTRY_FIRMWARE.md) › Libraries*

# Libraries

**Read this when** you are looking for the library that does a job, before opening its README.

Each folder is an ESP-IDF component: a `CMakeLists.txt`, the code in `src/`, a `README.md` (what it does, where it sits, who uses it, tests, open items) and a `REFERENCE.md` (API, timing, keys, errors, console commands, usage). The firmware finds them through `EXTRA_COMPONENT_DIRS` in `idf/CMakeLists.txt`, and `idf/main/CMakeLists.txt` lists them in `REQUIRES`.

| Layer | Library | What it does |
|---|---|---|
| Motion | [Gantry](Gantry/README.md) | The motion API: paths, homing, gripper; Kinetix 5100 X/Z and HCS01 theta axes |
| Links and protocols | [EtherNetIP](EtherNetIP/README.md) | EtherNet/IP and CIP originator: codecs, Class 1 scanner tasks, explicit messages |
|  | [CellNetL2](CellNetL2/README.md) | Cell messages on raw Ethernet (EtherType 0x88B5), transports, the OTA receiver |
|  | [EthernetLink](EthernetLink/README.md) | LAN8720 bring-up, static IP or DHCP, link events |
| Drivers | [W5500](W5500/README.md) | W5500 on SPI2: init, recovery, hardware sockets, the bus mutex |
|  | [MCP23S17](MCP23S17/README.md) | MCP23S17 SPI GPIO expander driver |
|  | [Spi3Bus](Spi3Bus/README.md) | Owns SPI3 (MCP23S17 and TFT) and the Class 1 critical section |
| Services | [AppConfig](AppConfig/README.md) | Stored runtime parameters: one NVS set, the schema, `cfg` and TFT access |
|  | [GpioExpander](GpioExpander/README.md) | Field I/O, gripper valve, encoder, TFT lines and W5500 reset on the MCP23S17 |
|  | [SpiDisplay](SpiDisplay/README.md) | ST7789 TFT and its menu |
|  | [GantryOta](GantryOta/README.md) | Firmware updates into the passive OTA slot, over TCP or CellNet |
|  | [ThetaTrace](ThetaTrace/README.md) | 1 kHz theta move trace and its summary (`thtrace`) |
|  | [BootLogRing](BootLogRing/README.md) | Keeps the first log output after reset for `bootlog` |

## Next

- **Down:** open a library's README from the table.
- **Up:** [Gantry firmware](../GANTRY_FIRMWARE.md), how the libraries work together.
