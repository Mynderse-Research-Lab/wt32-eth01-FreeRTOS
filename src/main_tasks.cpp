/**
 * @file main_tasks.cpp
 * @brief The application's periodic tasks (the 100 Hz Gantry update and the
 *        TFT UI) and the boot stage that starts every application task.
 */

#include "main_internal.h"

#include "AppConfig.h"
#include "CellNetL2.h"
#include "GantryControlPeriod.h"
#include "Spi3Bus.h"
#include "SpiDisplay.h"
#include "UiManager.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "ethernet_app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gantry_app_constants.h"
#include "gantry_test_console.h"
#include "gpio_expander.h"
#include "pick_scheduler.h"

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

GantryTestConsoleConfig* bootStartTasks(Gantry::Gantry& gantry, w5500::W5500Hal* w5500_hal,
                                        Network::EthernetLink& ethernetLink,
                                        CellNet::CellNetL2Node& l2Node) {
    BaseType_t result;
    static PickSchedulerTaskConfig pickCfg = { &gantry, &l2Node };
    result = xTaskCreatePinnedToCore(
        pickSchedulerTask, "PickScheduler",
        PICK_SCHEDULER_TASK_STACK, &pickCfg,
        PICK_SCHEDULER_TASK_PRIORITY, nullptr, PICK_SCHEDULER_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create PickScheduler task");
        gantry.disable();
        return nullptr;
    }
    static UpdateTaskConfig updateCfg = { &gantry };
    result = xTaskCreatePinnedToCore(
        gantryUpdateTask, "GantryUpdate",
        GANTRY_UPDATE_TASK_STACK, &updateCfg,
        GANTRY_UPDATE_TASK_PRIORITY, nullptr, GANTRY_UPDATE_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Gantry Update task!");
    }

    static UiTaskConfig uiCfg = { &gantry, &ethernetLink };
    if (Config::AppConfig::instance().data().display_tft_enable != 0) {
        result = xTaskCreatePinnedToCore(
            tftUiTask, "TftUiTask",
            TFT_UI_TASK_STACK, &uiCfg,
            TFT_UI_TASK_PRIORITY, nullptr, TFT_UI_TASK_CORE);
        if (result != pdPASS) {
            ESP_LOGW(TAG, "Failed to create TftUiTask");
        }
    }

    static GantryTestConsoleConfig consoleCfg = {};
    consoleCfg.gantry                 = &gantry;
    consoleCfg.limit_min_pin          = -1;   // Endstops drive-managed (EIP)
    consoleCfg.limit_max_pin          = -1;
    consoleCfg.use_mcp23s17           = (gpio_expander_get_mcp_handle() != nullptr);
    consoleCfg.w5500_hal              = w5500_hal;

#if CONSOLE_UART_ENABLE
    result = xTaskCreatePinnedToCore(
        gantryTestConsoleTask, "SerialCmd",
        CONSOLE_TASK_STACK, &consoleCfg,
        CONSOLE_TASK_PRIORITY, nullptr, CONSOLE_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Serial task!");
    }
#else
    ESP_LOGI(TAG, "UART console disabled — GPIO1/3 free; use TCP :%d",
             CONSOLE_TCP_PORT);
#endif
    return &consoleCfg;
}
