# EthernetLink reference

Public header: `lib/EthernetLink/src/EthernetLink.h`, namespace `Network`.

## API

```c
struct EthernetBootTimes {
    int64_t start_us;       // EthernetLink::start() entered, or -1
    int64_t eth_started_us; // esp_eth_start() returned, or -1
    int64_t link_up_us;     // first PHY link-up, or -1
    int64_t got_ip_us;      // first IP address, or -1
};
EthernetBootTimes ethernetBootTimes();

static bool EthernetLink::enablePhyOscillator();
bool start();
bool waitForUp(uint32_t timeout_ms);
bool isUp() const;
esp_eth_handle_t getEthHandle() const;
esp_netif_t* getNetif() const;
```

Times are `esp_timer` microseconds since boot. `-1` means that milestone has not happened.

`enablePhyOscillator` drives GPIO16 high and, when it had to configure the pin, waits `ETH_PHY_OSC_SETTLE_MS` (1,000 ms) (`EthernetLink.cpp:29–32, 98`). A pin that is already high is left high and does not wait. `start` does not use GPIO16 as the PHY reset pin.

`start` installs the ESP32 EMAC with `EMAC_CLK_EXT_IN` on GPIO0 (`EthernetLink.cpp:247–248`) and sets `ETH_MAC_FLAG_PIN_TO_CORE` so `emac_rx` stays on the core that calls `start` (`EthernetLink.cpp:233–235`). `app_main` calls `bootStartNetwork`, which calls `start`, on core 0. `enablePhyOscillator` is called earlier, from `bootInitIo`.

## Link state

`isUp()` returns `link_up_`.

On `ETHERNET_EVENT_DISCONNECTED` the handler clears `phy_link_connected_`. It clears `link_up_` only when `ETH_USE_STATIC_IP` is 0 (`EthernetLink.cpp:366–370`). `idf/sdkconfig.defaults` sets `CONFIG_ETH_USE_STATIC_IP=y`, so a cable pull leaves `isUp()` true. G-E7.

`waitForUp` polls until `link_up_` or `timeout_ms`. The boot call uses `ETH_IP_WAIT_TIMEOUT_MS`, 1,500 ms unless menuconfig overrides it (`include/ethernet_app_config.h:42–45`). Services in `bootStartNetwork` are not started when this wait fails. G-E7.

## Address

With static IP (the default), `configureNetwork` stops DHCP and writes `ETH_STATIC_IP`, `ETH_STATIC_GATEWAY` and `ETH_STATIC_NETMASK` from `include/ethernet_app_config.h`. The defaults file sets the IP to `10.42.0.100`. The stored AppConfig key `eth_ip` has no reader. G-F2.

## Task and timing

This component starts no task. Event handlers run on the default event loop. `emac_rx` is priority 15 on the core that called `start`.

## AppConfig

None inside this component. The compile-time address macros above are the values that are applied.

## Errors

`enablePhyOscillator` returns false when the GPIO config or the drive-high fails. `start` returns false when netif, MAC, PHY or driver install fails. `waitForUp` returns the value of `isUp()` at the deadline.

## Console

`boottime` (exact match) prints `ethernetBootTimes()` in milliseconds. The command is `cmdBootTime` in `src/gantry_console_commands.cpp`.

## Usage

```cpp
Network::EthernetLink::enablePhyOscillator();   // bootInitIo
static Network::EthernetLink ethernetLink;       // app_main
if (ethernetLink.start() && ethernetLink.waitForUp(ETH_IP_WAIT_TIMEOUT_MS)) {
    // bootStartNetwork: TCP console, OTA server, CellNet
}
```

## Documents

Wiring and the plant subnet are in `LOW_LEVEL_GANTRY_CONTROL.md`. Open items: G-E7 and G-F2 in `docs/OPEN_ITEMS.md`.
