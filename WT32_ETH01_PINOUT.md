# WT32-ETH01 Pinout Reference

## Overview

The **WT32-ETH01** is a Wireless-Tag ESP32 module with built-in Ethernet (LAN8720A PHY).
Many ESP32 GPIOs are **internally wired to the Ethernet PHY** and are **not available** for user IO.

This document lists every pin exposed on the board headers and their constraints.

---

## Header Pin Map

Derived from the official WT32-ETH01 pinout diagram.

### Left header (top to bottom)

| Header Label | GPIO | Direction | Notes |
|-------------|------|-----------|-------|
| EN          | —    | —         | Chip enable (active high, active to reset) |
| GND         | —    | —         | Ground |
| 3V3         | —    | —         | 3.3 V power supply |
| EN          | —    | —         | Enable (active high) |
| CFG         | 32   | I/O       | CFG configuration pin |
| 485_EN      | 33   | I/O       | Enable pin for RS485 |
| RXD         | 5    | I/O       | Serial port transparent transmission (RXD) |
| TXD         | 17   | I/O       | Serial port transparent transmission (TXD) |
| GND         | —    | —         | Ground |
| 3V3         | —    | —         | 3.3 V power supply |
| GND         | —    | —         | Ground |
| 5V          | —    | —         | 5 V power supply |
| LINK        | —    | —         | Network connection indicator light status |

### Right header (top to bottom)

| Header Label | GPIO | Direction | Notes |
|-------------|------|-----------|-------|
| TXD         | 0    | I/O       | TX (reserved for debugging/burning) |
| RXD         | 3    | Input     | RX (reserved for debugging/burning) |
| IO0         | 0    | I/O       | Reserved for debugging/burning (boot mode) |
| GND         | —    | —         | Ground |
| IO39        | 39   | **Input only** | — |
| IO36        | 36   | **Input only** | — |
| IO15        | 15   | I/O       | General purpose |
| IO14        | 14   | I/O       | General purpose |
| IO12        | 12   | I/O       | General purpose |
| IO35        | 35   | **Input only** | — |
| IO4         | 4    | I/O       | General purpose |
| IO2         | 2    | I/O       | General purpose |
| GND         | —    | —         | Ground |

---

## Available user GPIOs

These are the GPIOs exposed on the WT32-ETH01 headers that can be used for application IO:

| GPIO | Direction | Restrictions |
|------|-----------|-------------|
| 2    | I/O       | Directly exposed; also controls boot mode (leave floating or HIGH during boot) |
| 4    | I/O       | General purpose |
| 5    | I/O       | Directly exposed (also labelled RXD for transparent serial) |
| 12   | I/O       | General purpose; **must be LOW during boot** (strapping pin) |
| 14   | I/O       | General purpose |
| 15   | I/O       | General purpose; **must be HIGH during boot** (strapping pin, JTAG) |
| 17   | I/O       | Directly exposed (labelled TXD for transparent serial) |
| 32   | I/O       | Labelled CFG |
| 33   | I/O       | Labelled 485_EN |
| 35   | **Input only** | No internal pull-up/pull-down available |
| 36   | **Input only** | No internal pull-up/pull-down available |
| 39   | **Input only** | No internal pull-up/pull-down available |

**Total user IOs:** 12 (9 bidirectional + 3 input-only)

---

## GPIOs reserved by Ethernet PHY (NOT available)

These GPIOs are internally wired to the LAN8720A Ethernet PHY inside the module and **must not be used for other purposes** when Ethernet is active:

| GPIO | Ethernet Function |
|------|-------------------|
| 16   | ETH PHY power enable (`ETH_PHY_POWER`) |
| 17   | EMAC clock output (`ETH_CLK_MODE=ETH_CLOCK_GPIO17_OUT`) |
| 18   | EMAC MDC (`ETH_MDC_PIN`) |
| 19   | EMAC RMII TXD0 |
| 21   | EMAC RMII TX_EN |
| 22   | EMAC RMII TXD1 |
| 23   | EMAC MDIO (`ETH_MDIO_PIN`) |
| 25   | EMAC RMII RXD0 |
| 26   | EMAC RMII RXD1 |
| 27   | EMAC RMII CRS_DV |

> **Warning:** GPIO 17 is exposed on the header (labelled TXD) but is also used as the Ethernet clock output. If Ethernet is enabled, GPIO 17 is **not available** for general-purpose use.

---

## GPIOs not exposed on headers

These ESP32 GPIOs exist on the chip but are **not routed to any header pin** on the WT32-ETH01:

| GPIO | Reason |
|------|--------|
| 1    | Internal UART TX (used for debug/programming via TXD header) |
| 6–11 | Connected to internal SPI flash |
| 13   | Not routed to header |
| 16   | Ethernet PHY power (internal) |
| 18   | Ethernet MDC (internal) |
| 19   | Ethernet TXD0 (internal) |
| 20   | Not present on ESP32 |
| 21   | Ethernet TX_EN (internal) |
| 22   | Ethernet TXD1 (internal) |
| 23   | Ethernet MDIO (internal) |
| 24   | Not present on ESP32 |
| 25   | Ethernet RXD0 (internal) |
| 26   | Ethernet RXD1 (internal) |
| 27   | Ethernet CRS_DV (internal) |
| 28–31 | Not present on ESP32 |
| 34   | Not routed to header (input-only on ESP32) |
| 37–38 | Not routed to header |

---

## Boot strapping pins

| GPIO | Required state at boot | Effect |
|------|----------------------|--------|
| 0    | HIGH → normal boot; LOW → download mode | Directly exposed; use with care |
| 2    | LOW or floating → normal boot | Directly exposed |
| 12   | **LOW** → 3.3 V flash; HIGH → 1.8 V flash (usually wrong) | Must be LOW at boot |
| 15   | HIGH → normal boot; LOW → silences boot log | Directly exposed |

---

## Design notes for expansion cards

When designing expansion hardware (e.g. SPI-based MCP23S17 IO expander), keep in mind:

- **SPI bus must use available GPIOs only:** GPIO 18, 19, 23 are NOT available (Ethernet PHY). Use GPIOs from the "Available user GPIOs" table above.
- **LEDC (PWM) outputs** work on any output-capable GPIO (2, 4, 5, 12, 14, 15, 32, 33).
- **PCNT (encoder)** works on any GPIO (including input-only 35, 36, 39).
- **GPIO 17** is available for general use **only if Ethernet is disabled**.
- **Input-only pins** (35, 36, 39) cannot drive outputs and have no internal pull resistors.
