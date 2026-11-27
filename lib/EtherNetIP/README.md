*[README](../../README.md) › [Knowledge transfer](../../docs/KNOWLEDGE_TRANSFER.md) › [Gantry firmware](../../GANTRY_FIRMWARE.md) › [Libraries](../README.md) › EtherNetIP*

# EtherNetIP

**Read this when** you need to know what EtherNetIP does, where it sits and who uses it.

EtherNet/IP and CIP originator, in three layers:

- Encoders and decoders with no OS calls: encapsulation, CPF, Message Router, Forward Open and Close, Kinetix 5100 and HCS01 assemblies.
- Session and Class 1 connection logic, host-tested with fake sockets: `EipSession`, `EipIoConnection`, `EipScanner`, `EipMultiScanner`.
- The W5500 socket adapter, the scanner tasks and the diagnostics. Those files need ESP-IDF.

Production motion uses `EipMultiScanner` and `multiAxisTask`. `EipScanner` and `singleAxisTask` are the single-axis path.

## Maturity

Runs on the gantry. G-A14, G-B5, G-B8, G-B9, G-B16, G-B17 and G-B19 are open.

## Dependencies

`W5500`, `Spi3Bus`, and ESP-IDF `esp_driver_gpio`, `esp_netif`, `esp_timer`, `log`, `freertos`, `lwip` (the HCS01 COMWS client). The component `CMakeLists.txt` is the full list.

## Consumers

`bootInitMotion` builds the process images and calls `eip::startScannerTask`. `Gantry` is the only motion caller. The console commands below call into this component.

## Build and test

Registered from `idf/CMakeLists.txt`. Host tests cover encoding, assemblies, the session fakes, Class 1 timing and the multi-scanner demux (`test_eip_encoding`, `test_eip_transport`, `test_eip_axis`, `test_hcs01_assembly`, `test_cip_ethernet_link`, and the other `test_eip_*` / `test_hcs01_*` / `test_k5100_*` targets). `multiAxisTask` itself is not a host test.

## Open items

G-A14 (the Kinetix speed unit), G-B5, G-B8, G-B9, G-B16, G-B17, G-B19.

## Next

- **Down:** [EtherNetIP reference](REFERENCE.md), the facts: API, timing, keys, errors.
- **Up:** [Libraries](../README.md).
