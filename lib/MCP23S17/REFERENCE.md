# MCP23S17 reference

Public header: `lib/MCP23S17/src/MCP23S17.h`.

## API

```c
mcp23s17_handle_t mcp23s17_init(const mcp23s17_config_t* config);
void mcp23s17_deinit(mcp23s17_handle_t handle);

esp_err_t mcp23s17_set_pin_direction(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool is_output);
esp_err_t mcp23s17_set_pin_pullup(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool enable);
esp_err_t mcp23s17_write_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin, uint8_t level);
uint8_t mcp23s17_read_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin);

esp_err_t mcp23s17_read_latch(mcp23s17_handle_t handle, mcp23s17_port_t port, uint8_t* value);
esp_err_t mcp23s17_verify_config(mcp23s17_handle_t handle, uint8_t* iocon,
                                 uint8_t* iodira, uint8_t* iodirb);

esp_err_t mcp23s17_write_port(mcp23s17_handle_t handle, mcp23s17_port_t port, uint8_t value);
uint8_t mcp23s17_read_port(mcp23s17_handle_t handle, mcp23s17_port_t port);

esp_err_t mcp23s17_set_pin_interrupt(mcp23s17_handle_t handle, mcp23s17_pin_t pin,
                                      bool enable, bool trigger_on_rising);
esp_err_t mcp23s17_debug_read_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value);
esp_err_t mcp23s17_debug_write_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t value);
```

Pins are 0–15: 0–7 port A, 8–15 port B. `mcp23s17_init` returns a handle, or null on failure.

`mcp23s17_set_pin_direction`, `mcp23s17_set_pin_pullup` and `mcp23s17_write_pin` return `ESP_ERR_INVALID_ARG` for a null handle or a pin above 15, and `ESP_ERR_INVALID_STATE` when the lock times out. A successful SPI write returns `ESP_OK`. The shadow byte is updated before that write's result is known (`MCP23S17.cpp:151–157` for direction). G-D6.

`mcp23s17_read_pin` returns 0 when the handle is null, the pin is above 15, or the register read fails (`MCP23S17.cpp:234–244`). `mcp23s17_read_port` returns 0 on a null handle or a failed read (`MCP23S17.cpp:270–279`). G-D6.

`mcp23s17_set_pin_interrupt` sets INTCON so the pin is compared with DEFVAL, then writes that DEFVAL from `trigger_on_rising` (`MCP23S17.cpp:318–342`). The header comment calls the flag a rising or falling edge.

The header comment says the default clock is 10 MHz (`MCP23S17.h:51`). A `clock_speed_hz` of 0 is stored as 1 MHz (`MCP23S17.cpp:52`). The gantry passes `SPI3_MCP_CLOCK_HZ`, which is 1,000,000.

## Lock

`mcp23s17_lock` (`MCP23S17Bus.cpp:15–18, 39`) tries up to four times. Each try waits up to 8 ms for Class 1 to be idle, then up to 8 ms for the handle mutex. The mutex is then held across `spi_device_polling_transmit`. That IDF call waits for the SPI bus lock with no timeout of ours. G-D3.

The handle mutex and the Spi3Bus mutex are different locks. G-D6. See `lib/Spi3Bus/REFERENCE.md`.

## Cache

IODIR, GPPU and OLAT are cached on the handle and written back as whole port bytes. `mcp23s17_debug_write_register` writes the chip without updating those caches, so a debug write of IODIR or OLAT makes `mcp23s17_verify_config` disagree with the chip.

## AppConfig

None.

## Console

`mcp_pin_mode`, `mcp_dump`, `mcp_reg`, `field_dout` and `field_din` are in `src/gantry_console_commands.cpp` under `MCP_DEBUG_CMDS`. That macro defaults to 1 (`src/gantry_console_internal.h:51–52`), so the current firmware includes them.

## Usage

```c
spi3::init();
mcp23s17_config_t config = {};
config.spi_host = spi3::host();
config.cs_pin = (gpio_num_t)SPI3_CS_MCP_GPIO;
config.device_address = MCP23S17_HW_ADDR;
config.clock_speed_hz = SPI3_MCP_CLOCK_HZ;
config.skip_bus_init = true;
mcp23s17_handle_t handle = mcp23s17_init(&config);
```

Field and UI setup goes through `gpio_expander_init` in `lib/GpioExpander`.

## Documents

The pin map is `include/gantry_app_constants.h:40–71`. Open items: G-D3, G-D6 in `docs/OPEN_ITEMS.md`.
