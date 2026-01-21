## WT32-ETH01 MQTT Client Node Setup
This repository provides a basic setup for an MQTT client node designed to communicate over TCP using the WT32-ETH01 module.

## Entering Programming Mode on the WT32-ETH01
Putting the WT32-ETH01 into programming mode can be a little tricky the first time, but once you've done it, it's straightforward.

## Required Hardware
You will need a TTL device programmer. For this project, I am using the Waveshare USB to RS232 / RS485 / TTL converter.

## Wiring the WT32-ETH01 for Programming
Power: Connect 5V from your power supply or programmer to pin 12 of the WT32-ETH01.
GND: Connect GND to the breakout board’s ground rail.

## UART Connections:

Connect RX0 from the programmer to TX0 on the WT32-ETH01.

Connect TX0 from the programmer to RX0 on the WT32-ETH01.

## Extra Ground Pin:
Connect GND from the breakout board to pin 11 on the WT32-ETH01.

## Boot Mode Jumper:
Use a jumper wire to connect pin 24 (IO0) to pin 23 (GND).

## Enter Programming Mode
Briefly touch a jumper wire between the common GND and pin 1 (EN) to reset the module.

Remove the jumper wire between pin 24 (IO0) and pin 23 (GND).

Your WT32-ETH01 is now in programming mode and ready for flashing firmware.

## Return to Normal Operation
To return the WT32-ETH01 to normal operation mode, simply:

Touch pin 1 (EN) with ground again to reset the module.

![alt text](<Screenshot from 2025-06-28 18-05-10-1.png>)

## ESP-IDF Usage
This project includes an ESP-IDF build that reuses the existing Arduino-style `src/main.cpp`.

### Prerequisites
- ESP-IDF installed and `idf.py` available in your shell.
- ESP-IDF component manager enabled (default).

### Build and Flash
From an ESP-IDF shell:

```
cd D:\Projects\wt32-eth01-base\idf
idf.py set-target esp32
idf.py build
idf.py flash -p COM3
idf.py monitor -p COM3
```

### Notes
- The ESP-IDF project lives in `idf/`.
- `idf/main/app_main.cpp` bridges into your existing `setup()` / `loop()` in `src/main.cpp`.
- Arduino core v3 is pulled via `idf/main/idf_component.yml`.