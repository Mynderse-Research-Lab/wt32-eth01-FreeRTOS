# AppConfig reference

Public headers: `lib/AppConfig/src/AppConfig.h`, `AppConfigData.h`, `ConfigSchema.h`, `ConfigSet.h`, `ConfigStorage.h`, `NvsStorage.h`, `MemoryStorage.h`. Namespace `Config`.

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

`init` with a null storage builds `NvsStorage`. `factoryReset` reloads compile-time defaults in RAM and does not write NVS. `requestSave` calls the handler set by `setSaveHandler` when one is installed.

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

`kConfigSetMaxBytes` is 2,048 (`ConfigSet.h:19`). A record key is `configKeyHash` of the name. `dev_pw` is read and written with the set (`AppConfigParamIo.cpp:212, 370`) and is not a `ConfigSchema` row.

## Boot and save

`load` (`AppConfig.cpp:98–138`) always starts from `loadDefaults`. `readBlob` results:

- `kOk` and a hash that matches: the payload is applied.
- `kOk` and a hash or length that does not match, or `kError`: `set_damaged_` is set and the function returns without `writeSet`. G-F7.
- a missing key is not damaged. The following `writeSet` stores the defaults plus any leftover per-key values.

`setParamFloat` rejects a value outside the schema range. Both comparisons are false for a NaN, so a NaN is stored (`AppConfigAccess.cpp:227`, `AppConfigText.cpp:162–168`). G-F8.

`formatParamValue` prints a float with `%.2f` (`AppConfigText.cpp:92`).

## Threading

`data()` returns the live struct with no lock (`AppConfig.h:93`). `pendingChanges` and the change lists take `change_mutex_`. Several tasks call `data()` while `load` or `factoryReset` can replace the struct. The row for that sharing is not a separate open item; G-F7 is the boot path.

## Console

`cfg` is handled in `src/gantry_console_cfg.cpp`, not in this component. The help line is:

`cfg <key> [value] | cfg unlock <pw> | cfg lock | cfg passwd <old> <new> | cfg save | cfg export | cfg time | cfg factory | cfg reboot [discard]`

## Documents

Key names, units and ranges are `ConfigSchema.cpp`. Open items: G-F1, G-F2, G-F7, G-F8 and G-B5 in `docs/OPEN_ITEMS.md`.
