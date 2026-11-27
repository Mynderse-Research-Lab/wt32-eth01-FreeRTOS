# GpioExpander reference

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
[GpioExpander](README.md) › Reference*

**Read this when** you need an exact fact: a signature, a unit, a timeout, a key
or a return value.

Public header: `lib/GpioExpander/src/gpio_expander.h`.

## API

```c
bool gpio_expander_init(const mcp23s17_config_t* mcp_config);
void gpio_expander_deinit(void);
esp_err_t gpio_expander_set_direction(int pin, bool is_output);
esp_err_t gpio_expander_set_pullup(int pin, bool enable);
esp_err_t gpio_expander_write(int pin, uint8_t level);
uint8_t gpio_expander_read(int pin);
mcp23s17_handle_t gpio_expander_get_mcp_handle(void);
uint8_t gpio_expander_read_port_b(void);
esp_err_t gpio_expander_read_latch(int pin, uint8_t* level);
esp_err_t gpio_expander_verify(uint8_t* iocon, uint8_t* iodira, uint8_t* iodirb);
esp_err_t gpio_expander_configure_field_and_ui(void);
esp_err_t field_dout_set(unsigned channel, bool level);
bool field_din_get(unsigned channel);
void gpio_expander_w5500_rst_set_level(void* ctx, int level);
```

`pin` is an MCP23S17 pin number, 0 through 15. `level` is 0 or 1. `channel` is a
field channel starting at 0. The channel counts and the `MCP_*` pin names are in
`include/gantry_app_constants.h`. The direction and initial level of each group
are the comments in `gpio_expander_configure_field_and_ui`.

`gpio_expander_init` returns false when `mcp_config` is null or `mcp23s17_init`
fails. `gpio_expander_read` returns 0 when the chip is not initialised or the
pin is not an MCP pin. `gpio_expander_read_port_b` returns `0xFF` when the chip
is not initialised. `field_din_get` returns false in that case, and otherwise
true when the pin reads non-zero.

`field_dout_set` returns `ESP_ERR_INVALID_ARG` when the channel is out of range
or the chip is not initialised. Direction, pull-up, write, and latch calls
return `ESP_ERR_INVALID_ARG` for a non-MCP pin (and a null latch pointer). They
return `ESP_ERR_INVALID_STATE` when the chip is not initialised. Other
`esp_err_t` values come from MCP23S17.

`gpio_expander_w5500_rst_set_level` drives PB7 low when `level` is 0 and
releases it to the pull-up when `level` is non-zero. `ctx` is unused. Failures
from the chip calls are ignored.

`gpio_expander_verify` forwards to `mcp23s17_verify_config`. The header states
`ESP_ERR_INVALID_RESPONSE` when the read-back does not match.

## Task and timing

No task. Calls run on the caller and take the MCP23S17 lock inside that driver.
This wrapper adds no timeout of its own.

## AppConfig

None.

## Console

This component registers no command. The `mcp_*` console commands in
`src/gantry_console_io.cpp` call these functions.

## Usage

```c
if (!gpio_expander_init(&mcp_config)) {
    return;
}
if (gpio_expander_configure_field_and_ui() != ESP_OK) {
    return;
}
field_dout_set(0, true);
```

## Documents

The chip driver is `lib/MCP23S17`. G-D3 and G-D6 in `docs/OPEN_ITEMS.md` are
about that driver and Spi3Bus.

## See also

-   **Up:** [GpioExpander README](README.md), [Libraries](/lib/README.md).
