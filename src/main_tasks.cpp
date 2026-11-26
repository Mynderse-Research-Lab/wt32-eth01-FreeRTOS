/**
 * @file main_tasks.cpp
 * @brief The application's periodic tasks: the 100 Hz Gantry update and the
 *        TFT UI (encoder input at 100 Hz, display redraw at 15 Hz).
 */

#include "main_internal.h"

#include "AppConfig.h"
#include "GantryControlPeriod.h"
#include "Spi3Bus.h"
#include "SpiDisplay.h"
#include "UiManager.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gantry_app_constants.h"
#include "gpio_expander.h"

#include <cstring>

// ---------------------------------------------------------------------------
// Gantry periodic update task (100 Hz on Core 1)
// ---------------------------------------------------------------------------
void gantryUpdateTask(void* param) {
    auto* cfg = static_cast<UpdateTaskConfig*>(param);
    if (cfg == nullptr || cfg->gantry == nullptr) {
        ESP_LOGE(TAG, "Invalid Gantry update task config");
        vTaskDelete(nullptr);
        return;
    }

    const TickType_t updateInterval = pdMS_TO_TICKS(GANTRY_UPDATE_PERIOD_MS);
    Gantry::setControlPeriodUs(static_cast<uint32_t>(GANTRY_UPDATE_PERIOD_MS) * 1000u);
    ESP_LOGI(TAG, "Gantry update task started (%d ms)", GANTRY_UPDATE_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        cfg->gantry->update();
        vTaskDelayUntil(&lastWakeTime, updateInterval);
    }
}

// ---------------------------------------------------------------------------
// TFT UI: encoder polled ~100 Hz; display redraw 15 Hz (Core 0)
// ---------------------------------------------------------------------------
void tftUiTask(void* param) {
    auto* cfg = static_cast<UiTaskConfig*>(param);
    display::DashboardTelemetry telem = {};
    // Quadrature needs >> detent rate: 15 Hz display undersamples the knob.
    const TickType_t inputInterval = pdMS_TO_TICKS(10);       // 100 Hz
    const TickType_t drawInterval = pdMS_TO_TICKS(1000 / 15);  // 15 Hz
    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t lastDrawTime = lastWakeTime - drawInterval;  // draw on first pass

    ESP_LOGI(TAG, "TFT UI task started (input 100 Hz, draw 15 Hz)");

    while (1) {
        // Skip SPI3 while Class 1 critical — brief miss OK at 100 Hz input rate.
        if (!spi3_class1_critical_active()) {
            uint8_t port_b = gpio_expander_read_port_b();
            const TickType_t now = xTaskGetTickCount();
            const bool draw_due = (now - lastDrawTime) >= drawInterval;

            if (draw_due) {
                if (cfg && cfg->gantry) {
                    auto joints = cfg->gantry->getCurrentJointConfig();
                    telem.x_mm = joints.x;
                    telem.z_mm = joints.z;
                    telem.theta_deg = joints.theta;
                    telem.x_homed = !cfg->gantry->isBusy();
                    telem.z_homed = !cfg->gantry->isBusy();
                    telem.theta_homed = cfg->gantry->isThetaDriveOriginAligned();
                    telem.gripper_open = !cfg->gantry->isGripperActive();
                    telem.uptime_s =
                        static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
                    telem.dev_unlocked =
                        Config::AppConfig::instance().isDeveloperModeUnlocked();
                    telem.motion_state = cfg->gantry->isBusy() ? "BUSY" : "IDLE";
                }

                if (cfg && cfg->eth) {
                    telem.lan_link = cfg->eth->isUp();
                    esp_netif_ip_info_t ip_info;
                    if (cfg->eth->getNetif() &&
                        esp_netif_get_ip_info(cfg->eth->getNetif(), &ip_info) ==
                            ESP_OK) {
                        esp_ip4addr_ntoa(&ip_info.ip, telem.lan_ip,
                                         sizeof(telem.lan_ip));
                    } else {
                        std::strncpy(
                            telem.lan_ip,
                            Config::AppConfig::instance().data().eth_static_ip,
                            sizeof(telem.lan_ip));
                    }
                }

                display::UiManager::instance().update(port_b, telem);
                lastDrawTime = now;
            } else {
                display::UiManager::instance().pollInput(port_b);
            }
        }

        vTaskDelayUntil(&lastWakeTime, inputInterval);
    }
}
