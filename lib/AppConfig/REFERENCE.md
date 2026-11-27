# AppConfig reference

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
[AppConfig](README.md) › Reference*

**Read this when** you need an exact fact: a signature, a unit, a timeout, a key
or a return value.

Public headers: `lib/AppConfig/src/AppConfig.h`, `AppConfigData.h`,
`ConfigSchema.h`, `ConfigSet.h`, `ConfigStorage.h`, `NvsStorage.h`,
`MemoryStorage.h`. Namespace `Config`.

## API

```c
static AppConfig& instance();
bool init(std::unique_ptr<IConfigStorage> storage = nullptr);
void loadDefaults();
bool load();
void commitPending();
bool saveSet();
bool requestSave();
void setSaveHandler(std::function<bool()> handler);
bool factoryReset();
ConfigSetInfo activeSet() const;
int pendingChanges() const;
const AppConfigData& data() const;

bool getParamInt(const char* key, int32_t& out_val) const;
bool setParamInt(const char* key, int32_t val, bool& out_reboot_required);
bool getParamFloat(const char* key, float& out_val) const;
bool setParamFloat(const char* key, float val, bool& out_reboot_required);
bool getParamBool(const char* key, bool& out_val) const;
bool setParamBool(const char* key, bool val, bool& out_reboot_required);
bool getParamString(const char* key, char* out_buf, size_t max_len) const;
bool setParamString(const char* key, const char* val, bool& out_reboot_required);

bool isParamVisible(const char* key) const;
bool formatParamValue(const char* key, char* out_buf, size_t max_len) const;
bool applyParamText(const char* key, const char* text, bool& out_reboot_required);
bool unlockDeveloperMode(const char* password);
void lockDeveloperMode();
bool changeDeveloperPassword(const char* old_pw, const char* new_pw);
```

`init` with a null storage builds `NvsStorage`. `factoryReset` reloads
compile-time defaults in RAM and does not write NVS. `requestSave` calls the
handler set by `setSaveHandler` when one is installed.

## Set format

One NVS key, `cfg_set` (`AppConfig.cpp:16`). There is no second slot.

`ConfigSetHeader` (`ConfigSet.h:21–31`) is the front of the blob:

| Field | Meaning |
|---|---|
| `magic` | `kConfigSetMagic` = `0x47434647` |
| `seq` | count of writes |
| `unix_s` | newest change, 0 if the clock was not set |
| `payload_hash` | `configHash` of the payload (FNV-1a, `ConfigSet.h:33–39`) |
| `payload_len`, `param_count` | bytes and record count after the header |
| `format` | `kConfigSetFormat` = 1 |

`kConfigSetMaxBytes` is 2,048 (`ConfigSet.h:19`). A record key is
`configKeyHash` of the name. `dev_pw` is read and written with the set
(`AppConfigParamIo.cpp:212, 370`) and is not a `ConfigSchema` row.

## Boot and save

`load` (`AppConfig.cpp:98–138`) always starts from `loadDefaults`. `readBlob`
results:

-   `kOk` and a hash that matches: the payload is applied.
-   `kOk` and a hash or length that does not match, or `kError`: `set_damaged_`
    is set and the function returns without `writeSet`. G-F7.
-   a missing key is not damaged. The following `writeSet` stores the defaults
    plus any leftover per-key values.

`setParamFloat` rejects a value outside the schema range. Both comparisons are
false for a NaN, so a NaN is stored (`AppConfigAccess.cpp:227`,
`AppConfigText.cpp:162–168`). G-F8.

`formatParamValue` prints a float with `%.2f` (`AppConfigText.cpp:92`).

## Threading

`data()` returns the live struct with no lock (`AppConfig.h:93`).
`pendingChanges` and the change lists take `change_mutex_`. Several tasks call
`data()` while `factoryReset`, from the console or the TFT menu, rewrites the
struct in place. G-F9.

## Console

`cfg` is handled in `src/console/system/gantry_console_cfg.cpp`, not in this
component. The help line is:

`cfg <key> [value] | cfg unlock <pw> | cfg lock | cfg passwd <old> <new> | cfg save | cfg export | cfg time | cfg factory | cfg reboot [discard]`

## Errors

The getters and setters return false for an unknown key or a key of another
type, and the setters also for a value outside the schema range
(`AppConfigAccess.cpp:89–91, 225–227`); `out_reboot_required` is then not
written. `saveSet` returns false without a storage backend
(`AppConfig.cpp:186`). `requestSave` returns false when no save handler is
installed (`AppConfig.cpp:203`). `unlockDeveloperMode` and
`changeDeveloperPassword` return false for a null or wrong password.

## Usage

```c
// boot (src/main/main_init_io.cpp)
if (nvs_flash_init() == ESP_OK) {
    Config::AppConfig::instance().init();
}
// any task
const Config::AppConfigData& cfg = Config::AppConfig::instance().data();
// console: change one value; `cfg save` writes the set
bool reboot = false;
Config::AppConfig::instance().setParamInt("spd_mm_s", 500, reboot);
```

## Known issues

Found in the library review of 2026-11-26 (at `1caa98c`) and not tracked as open
items. Not re-checked since; confirm against the code before acting.

| # | Category | Location | Finding | Evidence |
|---|---|---|---|---|
| C4 | rule | `AppConfigDefaults*.cpp`, `AppConfigData.h` | Three copies of the defaults. The host fallbacks disagree with `sdkconfig.defaults`; for example X speed 3333.3 vs 500, theta accel 1800 vs 18000, LAN IP 192.168.1.100 vs 10.42.0.100. About 30 defaults exist only in C++, against the rule that `sdkconfig.defaults` is the only compile-time default source | Confirmed |
| C6 | docs | `AppConfigText.cpp:92` | `formatParamValue` prints floats with `%.2f`, so `cfg <key>` and the TFT show 0.005 as 0.01; the export uses `%.7g` | Confirmed |
| C7 | risk | `ConfigSet.h:180-181` | The reader stops at the first unknown record tag, so a newer format without a version bump would make older firmware drop the rest of the set and rewrite it | Suggested |
| C8 | risk | `AppConfig.h:93-98` | `data_` is read by several tasks without a lock; `factoryReset()` or `load()` rewrite the whole struct while readers run | Suggested |
| C9 | cleanup | `AppConfigParamIo.cpp:149-150, 307-308`; `CMakeLists.txt:16` | Two empty `for` loops; unused `mutableData()`; an unneeded `${APP_ROOT}/include` export; two keys for the W5500 clock (`w5500_spi_hz` unread, `eip_spi_khz` read) | Confirmed |

**Not covered by host tests:**
-   an NVS read error other than "not found" (C1)
-   NaN and infinity (C3)
-   a malformed payload given straight to the reader
-   hash collisions including `dev_pw` and the old renamed keys
-   set size with maximum-length strings

## Documents

Key names, units and ranges are `ConfigSchema.cpp`. Open items: G-F1, G-F2,
G-F8, G-F9 and G-B5 in `docs/OPEN_ITEMS.md`; G-F7 is closed.

## See also

-   **Up:** [AppConfig README](README.md), [Libraries](/lib/README.md).
