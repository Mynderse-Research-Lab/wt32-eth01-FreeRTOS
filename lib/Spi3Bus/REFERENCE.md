# Spi3Bus reference

Public header: `lib/Spi3Bus/src/Spi3Bus.h`. The nesting counter is `lib/Spi3Bus/src/Spi3BusCritical.h`.

## API

```c
bool init();
void deinit();
bool isReady();
spi_host_device_t host();
SemaphoreHandle_t mutex();

void class1CriticalEnter();
void class1CriticalExit();
bool class1CriticalActive();
bool waitClass1Idle(uint32_t timeout_ms);

esp_err_t withMcp(const std::function<esp_err_t()>& fn);
esp_err_t withTft(const std::function<esp_err_t()>& fn);
```

The same critical-section calls exist as `spi3_class1_critical_enter`, `spi3_class1_critical_exit`, `spi3_class1_critical_active` and `spi3_wait_class1_idle` for C callers.

`init` is idempotent: a second call returns true. `withMcp` and `withTft` return `ESP_ERR_INVALID_STATE` when the bus is not ready. They return `ESP_ERR_TIMEOUT` when the Class 1 wait or the mutex wait fails (`Spi3Bus.cpp:79–92`). Otherwise they return the callback's `esp_err_t`.

## Locks

Three locks can sit on one SPI3 transfer:

1. The Class 1 critical flag. `withMcp` and `withTft` wait until it is clear before taking the bus mutex. A transfer that has already started is not aborted. G-D3.
2. The Spi3Bus mutex (`mutex()`). `withMcp` and `withTft` take it.
3. The MCP23S17 mutex. The MCP driver takes it on its own. `withMcp` does not. G-D6.

A TFT data/command change takes the Spi3Bus mutex and then the MCP mutex. This tree does not take them in the other order. The ESP-IDF SPI device lock inside `spi_device_polling_transmit` is a fourth wait, and these timeouts do not bound it.

## Timing

From `Spi3Bus.cpp:28–31`:

| Constant | Value |
|---|---|
| `kClass1WaitMs` | 8 ms, shared by up to `kClass1RetryAttempts` (4) tries |
| `kMutexTimeoutMs` | 50 ms |

`waitClass1Idle` blocks on an event-group bit that `class1CriticalExit` sets. It does not poll.

Clocks, from `include/gantry_app_constants.h:45–46`: MCP 1 MHz (`SPI3_MCP_CLOCK_HZ`), TFT 20 MHz (`SPI3_TFT_CLOCK_HZ`).

## AppConfig

None. Pins and clocks are compile-time.

## Errors

`init` returns false if the mutex cannot be created or the SPI bus install fails. `withMcp` and `withTft` return the codes above. The component starts no task.

## Console

None.

## Usage

```c
spi3::init();
spi3_class1_critical_enter();
// Class 1 SPI2 exchange
spi3_class1_critical_exit();
```

## Documents

Pin names are the block at `include/gantry_app_constants.h:40–71`. Open items: G-D3, G-D6 in `docs/OPEN_ITEMS.md`.
