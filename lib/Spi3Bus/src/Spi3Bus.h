#ifndef SPI3_BUS_H
#define SPI3_BUS_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <functional>

/**
 * @file Spi3Bus.h
 * @brief Shared SPI3 for MCP23S17 (ESP CS) + TFT (MCP software CS).
 *
 * Class 1 (SPI2 / EIP scanner) has priority: SPI3 waits while a Class 1
 * exchange critical section is active.
 */

namespace spi3 {

bool init();
void deinit();
bool isReady();

spi_host_device_t host();
SemaphoreHandle_t mutex();

void class1CriticalEnter();
void class1CriticalExit();
bool class1CriticalActive();

esp_err_t withMcp(const std::function<esp_err_t()>& fn);
esp_err_t withTft(const std::function<esp_err_t()>& fn);

}  // namespace spi3

#ifdef __cplusplus
extern "C" {
#endif
void spi3_class1_critical_enter(void);
void spi3_class1_critical_exit(void);
bool spi3_class1_critical_active(void);
#ifdef __cplusplus
}
#endif

#endif  // SPI3_BUS_H
