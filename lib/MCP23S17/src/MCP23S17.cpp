/**
 * @file MCP23S17.cpp
 * @brief MCP23S17 SPI GPIO Expander Implementation
 */

#include "MCP23S17.h"
#include <string.h>

// MCP23S17 Register addresses
#define MCP23S17_IODIRA   0x00  // I/O Direction Port A
#define MCP23S17_IODIRB   0x01  // I/O Direction Port B
#define MCP23S17_IPOLA    0x02  // Input Polarity Port A
#define MCP23S17_IPOLB    0x03  // Input Polarity Port B
#define MCP23S17_GPINTENA 0x04  // Interrupt Enable Port A
#define MCP23S17_GPINTENB 0x05  // Interrupt Enable Port B
#define MCP23S17_DEFVALA  0x06  // Default Compare Port A
#define MCP23S17_DEFVALB  0x07  // Default Compare Port B
#define MCP23S17_INTCONA  0x08  // Interrupt Control Port A
#define MCP23S17_INTCONB  0x09  // Interrupt Control Port B
#define MCP23S17_IOCON    0x0A  // I/O Configuration (shared)
#define MCP23S17_GPPUA    0x0C  // Pull-up Resistor Port A
#define MCP23S17_GPPUB    0x0D  // Pull-up Resistor Port B
#define MCP23S17_INTFA    0x0E  // Interrupt Flag Port A
#define MCP23S17_INTFB    0x0F  // Interrupt Flag Port B
#define MCP23S17_INTCAPA  0x10  // Interrupt Capture Port A
#define MCP23S17_INTCAPB  0x11  // Interrupt Capture Port B
#define MCP23S17_GPIOA    0x12  // GPIO Port A
#define MCP23S17_GPIOB    0x13  // GPIO Port B
#define MCP23S17_OLATA    0x14  // Output Latch Port A
#define MCP23S17_OLATB    0x15  // Output Latch Port B

// MCP23S17 SPI opcode
#define MCP23S17_WRITE_OPCODE 0x40
#define MCP23S17_READ_OPCODE  0x41

struct mcp23s17_handle {
    spi_device_handle_t spi_device;
    uint8_t device_address;
    uint8_t port_a_dir;      // Cached direction register
    uint8_t port_b_dir;
    uint8_t port_a_output;   // Cached output latch
    uint8_t port_b_output;
};

static const char *TAG = "MCP23S17";

static esp_err_t mcp23s17_write_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t value) {
    uint8_t tx_data[3];
    tx_data[0] = MCP23S17_WRITE_OPCODE | (handle->device_address << 1);
    tx_data[1] = reg;
    tx_data[2] = value;

    spi_transaction_t trans = {};
    trans.length = 24;  // 3 bytes * 8 bits
    trans.tx_buffer = tx_data;
    trans.rx_buffer = NULL;

    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write register 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t mcp23s17_read_register(mcp23s17_handle_t handle, uint8_t reg, uint8_t* value) {
    uint8_t tx_data[3];
    uint8_t rx_data[3];
    
    tx_data[0] = MCP23S17_READ_OPCODE | (handle->device_address << 1);
    tx_data[1] = reg;
    tx_data[2] = 0;  // Dummy byte

    spi_transaction_t trans = {};
    trans.length = 24;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    esp_err_t ret = spi_device_transmit(handle->spi_device, &trans);
    if (ret == ESP_OK) {
        *value = rx_data[2];
    } else {
        ESP_LOGE(TAG, "Read register 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

mcp23s17_handle_t mcp23s17_init(const mcp23s17_config_t* config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }

    // Allocate handle
    mcp23s17_handle_t handle = (mcp23s17_handle_t)malloc(sizeof(struct mcp23s17_handle));
    if (handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return NULL;
    }
    memset(handle, 0, sizeof(struct mcp23s17_handle));
    handle->device_address = config->device_address & 0x07;  // Lower 3 bits only

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = config->miso_pin;
    bus_cfg.mosi_io_num = config->mosi_pin;
    bus_cfg.sclk_io_num = config->sclk_pin;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 32;

    esp_err_t ret = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        free(handle);
        return NULL;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = config->clock_speed_hz;
    dev_cfg.mode = 0;  // SPI mode 0
    dev_cfg.spics_io_num = config->cs_pin;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = 0;
    dev_cfg.pre_cb = NULL;

    ret = spi_bus_add_device(config->spi_host, &dev_cfg, &handle->spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(config->spi_host);
        free(handle);
        return NULL;
    }

    // Configure CS pin
    gpio_set_direction(config->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(config->cs_pin, 1);  // CS high (inactive)

    // Initialize IOCON register (sequential mode, hardware addressing)
    ret = mcp23s17_write_register(handle, MCP23S17_IOCON, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IOCON init failed");
        mcp23s17_deinit(handle);
        return NULL;
    }

    // Initialize all pins as inputs with pull-ups disabled
    handle->port_a_dir = 0xFF;
    handle->port_b_dir = 0xFF;
    mcp23s17_write_register(handle, MCP23S17_IODIRA, 0xFF);
    mcp23s17_write_register(handle, MCP23S17_IODIRB, 0xFF);
    mcp23s17_write_register(handle, MCP23S17_GPPUA, 0x00);
    mcp23s17_write_register(handle, MCP23S17_GPPUB, 0x00);

    handle->port_a_output = 0x00;
    handle->port_b_output = 0x00;

    ESP_LOGI(TAG, "MCP23S17 initialized (address: 0x%02X)", 0x20 | handle->device_address);
    return handle;
}

void mcp23s17_deinit(mcp23s17_handle_t handle) {
    if (handle == NULL) return;

    if (handle->spi_device) {
        spi_bus_remove_device(handle->spi_device);
    }
    spi_bus_free(SPI2_HOST);  // Assuming SPI2_HOST, adjust if needed
    free(handle);
}

esp_err_t mcp23s17_set_pin_direction(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool is_output) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t bit = pin & 0x07;
    uint8_t reg = (pin < 8) ? MCP23S17_IODIRA : MCP23S17_IODIRB;
    uint8_t* dir_cache = (pin < 8) ? &handle->port_a_dir : &handle->port_b_dir;

    if (is_output) {
        *dir_cache &= ~(1 << bit);
    } else {
        *dir_cache |= (1 << bit);
    }

    return mcp23s17_write_register(handle, reg, *dir_cache);
}

esp_err_t mcp23s17_set_pin_pullup(mcp23s17_handle_t handle, mcp23s17_pin_t pin, bool enable) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t bit = pin & 0x07;
    uint8_t reg = (pin < 8) ? MCP23S17_GPPUA : MCP23S17_GPPUB;
    uint8_t current;
    
    esp_err_t ret = mcp23s17_read_register(handle, reg, &current);
    if (ret != ESP_OK) return ret;

    if (enable) {
        current |= (1 << bit);
    } else {
        current &= ~(1 << bit);
    }

    return mcp23s17_write_register(handle, reg, current);
}

esp_err_t mcp23s17_write_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin, uint8_t level) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t bit = pin & 0x07;
    uint8_t reg = (pin < 8) ? MCP23S17_OLATA : MCP23S17_OLATB;
    uint8_t* output_cache = (pin < 8) ? &handle->port_a_output : &handle->port_b_output;

    if (level) {
        *output_cache |= (1 << bit);
    } else {
        *output_cache &= ~(1 << bit);
    }

    return mcp23s17_write_register(handle, reg, *output_cache);
}

uint8_t mcp23s17_read_pin(mcp23s17_handle_t handle, mcp23s17_pin_t pin) {
    if (handle == NULL || pin > 15) {
        return 0;
    }

    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t value;
    
    if (mcp23s17_read_register(handle, reg, &value) != ESP_OK) {
        return 0;
    }

    uint8_t bit = pin & 0x07;
    return (value >> bit) & 0x01;
}

esp_err_t mcp23s17_write_port(mcp23s17_handle_t handle, mcp23s17_port_t port, uint8_t value) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_OLATA : MCP23S17_OLATB;
    if (port == MCP23S17_PORT_A) {
        handle->port_a_output = value;
    } else {
        handle->port_b_output = value;
    }

    return mcp23s17_write_register(handle, reg, value);
}

uint8_t mcp23s17_read_port(mcp23s17_handle_t handle, mcp23s17_port_t port) {
    if (handle == NULL) {
        return 0;
    }

    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t value;
    
    if (mcp23s17_read_register(handle, reg, &value) != ESP_OK) {
        return 0;
    }

    return value;
}

esp_err_t mcp23s17_set_pin_interrupt(mcp23s17_handle_t handle, mcp23s17_pin_t pin, 
                                      bool enable, bool trigger_on_rising) {
    if (handle == NULL || pin > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t bit = pin & 0x07;
    uint8_t inten_reg = (pin < 8) ? MCP23S17_GPINTENA : MCP23S17_GPINTENB;
    uint8_t intcon_reg = (pin < 8) ? MCP23S17_INTCONA : MCP23S17_INTCONB;
    uint8_t defval_reg = (pin < 8) ? MCP23S17_DEFVALA : MCP23S17_DEFVALB;

    // Enable/disable interrupt
    uint8_t inten_val;
    mcp23s17_read_register(handle, inten_reg, &inten_val);
    if (enable) {
        inten_val |= (1 << bit);
    } else {
        inten_val &= ~(1 << bit);
    }
    mcp23s17_write_register(handle, inten_reg, inten_val);

    // Configure interrupt trigger
    uint8_t intcon_val;
    mcp23s17_read_register(handle, intcon_reg, &intcon_val);
    intcon_val |= (1 << bit);  // Compare to DEFVAL
    mcp23s17_write_register(handle, intcon_reg, intcon_val);

    // Set default value for comparison
    uint8_t defval_val;
    mcp23s17_read_register(handle, defval_reg, &defval_val);
    if (trigger_on_rising) {
        defval_val &= ~(1 << bit);  // Trigger when pin goes HIGH (was LOW)
    } else {
        defval_val |= (1 << bit);   // Trigger when pin goes LOW (was HIGH)
    }
    mcp23s17_write_register(handle, defval_reg, defval_val);

    return ESP_OK;
}
