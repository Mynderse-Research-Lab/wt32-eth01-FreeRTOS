#include "UiManager.h"
#include "AppConfig.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include <cstdio>

namespace display {

static const char* TAG = "UiManager";

UiManager& UiManager::instance() {
    static UiManager s_instance;
    return s_instance;
}

UiManager::UiManager()
    : dashboard_(driver_),
      menu_(driver_),
      editor_(driver_),
      reboot_prompt_(driver_) {}

bool UiManager::init(const SpiDisplayConfig& cfg) {
    mcp_ = cfg.mcp;
    mcp_blk_pin_ = cfg.mcp_blk_pin;
    esp_blk_pin_ = cfg.esp_blk_pin;

    if (esp_blk_pin_ >= 0) {
        ledc_timer_config_t ledc_timer = {};
        ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num        = LEDC_TIMER_0;
        ledc_timer.duty_resolution  = LEDC_TIMER_8_BIT;
        ledc_timer.freq_hz          = 5000;
        ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel = {};
        ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
        ledc_channel.channel        = LEDC_CHANNEL_0;
        ledc_channel.timer_sel      = LEDC_TIMER_0;
        ledc_channel.intr_type      = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num       = esp_blk_pin_;
        ledc_channel.duty           = 255;
        ledc_channel.hpoint         = 0;
        ledc_channel_config(&ledc_channel);
    }

    if (!driver_.init(cfg)) {
        ESP_LOGE(TAG, "ST7789 driver initialization failed");
        return false;
    }

    // Apply orientation from persistent configuration
    const auto& app_cfg = Config::AppConfig::instance().data();
    if (app_cfg.display_orientation == 1) {
        driver_.setOrientation(Orientation::PORTRAIT);
    } else {
        driver_.setOrientation(Orientation::LANDSCAPE);
    }
    prev_orient_ = app_cfg.display_orientation;

    menu_.init();
    state_ = UiScreenState::DASHBOARD;
    initialized_ = true;
    return true;
}

void UiManager::applyBacklight(uint32_t uptime_s) {
    const auto& app_cfg = Config::AppConfig::instance().data();
    int32_t target_brightness = app_cfg.display_brightness; // 0..100 %
    if (app_cfg.display_backlight_timeout_s > 0 &&
        (uptime_s - last_activity_s_) > static_cast<uint32_t>(app_cfg.display_backlight_timeout_s)) {
        target_brightness = 10;
    }

    if (esp_blk_pin_ >= 0) {
        uint32_t duty = (target_brightness * 255) / 100;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else if (mcp_ && mcp_blk_pin_ >= 0) {
        // 10-step software PWM (carrier tracks input poll rate when used)
        pwm_cycle_ = (pwm_cycle_ + 1) % 10;
        int active_threshold = (target_brightness + 5) / 10; // 0..10
        uint8_t blk_val = (pwm_cycle_ < active_threshold) ? 1 : 0;
        mcp23s17_write_pin(mcp_, static_cast<mcp23s17_pin_t>(mcp_blk_pin_), blk_val);
    }
}

void UiManager::dispatchEvent(UiEvent evt, const DashboardTelemetry& telem,
                              bool refresh_dashboard_telem) {
    switch (state_) {
        case UiScreenState::DASHBOARD: {
            if (evt == UiEvent::PUSH_CLICK) {
                state_ = UiScreenState::MENU;
                menu_.reset();
                menu_.render();
            } else if (refresh_dashboard_telem) {
                dashboard_.updateTelemetry(telem);
            }
            break;
        }

        case UiScreenState::MENU: {
            if (evt == UiEvent::BACK_CLICK) {
                state_ = UiScreenState::DASHBOARD;
                dashboard_.renderFull(telem);
                break;
            }

            std::string param_to_edit;
            MenuItemType action = MenuItemType::SUBMENU;
            bool triggered = menu_.handleEvent(evt, param_to_edit, action);

            if (triggered) {
                if (action == MenuItemType::PARAM) {
                    state_ = UiScreenState::EDITOR;
                    editor_.startParamEdit(param_to_edit);
                    editor_.render();
                } else if (action == MenuItemType::ACTION_UNLOCK_DEV) {
                    state_ = UiScreenState::EDITOR;
                    editor_.startPasswordUnlock();
                    editor_.render();
                } else if (action == MenuItemType::ACTION_CHANGE_DEV_PW) {
                    state_ = UiScreenState::EDITOR;
                    editor_.startPasswordChange();
                    editor_.render();
                } else if (action == MenuItemType::ACTION_FACTORY_RESET) {
                    Config::AppConfig::instance().factoryReset();
                    state_ = UiScreenState::REBOOT_PROMPT;
                    reboot_prompt_.show();
                } else if (action == MenuItemType::ACTION_ABOUT) {
                    state_ = UiScreenState::ABOUT;
                    driver_.fillScreen(COLOR_BLACK);
                    driver_.fillRect(0, 0, driver_.getWidth(), 22, COLOR_HEADER_BG);
                    driver_.drawStringCentered(3, "ABOUT GANTRY CONTROLLER", COLOR_WHITE, COLOR_HEADER_BG);

                    driver_.drawString(15, 45, "Firmware: WT32-ETH01 Gantry", COLOR_YELLOW, COLOR_BLACK);
                    driver_.drawString(15, 70, "Target:   ESP32-WROVER / Rev 3", COLOR_WHITE, COLOR_BLACK);
                    driver_.drawString(15, 95, "Version:  v1.0.0 (Production)", COLOR_WHITE, COLOR_BLACK);
                    driver_.drawString(15, 120,"Build:    " __DATE__ " " __TIME__, COLOR_LIGHTGREY, COLOR_BLACK);
                    driver_.drawString(15, 145,"Class 1:  EtherNet/IP Originator", COLOR_CYAN, COLOR_BLACK);
                    driver_.drawString(15, 170,"Display:  ST7789 320x240 (SPI3)", COLOR_CYAN, COLOR_BLACK);

                    driver_.fillRect(0, 218, driver_.getWidth(), 22, COLOR_PANEL_BG);
                    driver_.drawStringCentered(221, "PRESS ANY KEY TO RETURN", COLOR_YELLOW, COLOR_PANEL_BG);
                }
            }
            break;
        }

        case UiScreenState::EDITOR: {
            bool reboot_req = false;
            EditorResult res = editor_.handleEvent(evt, reboot_req);
            if (res == EditorResult::CONFIRMED) {
                if (reboot_req) {
                    state_ = UiScreenState::REBOOT_PROMPT;
                    reboot_prompt_.show();
                } else {
                    state_ = UiScreenState::MENU;
                    menu_.invalidate();
                    menu_.render();
                }
            } else if (res == EditorResult::CANCELLED) {
                state_ = UiScreenState::MENU;
                menu_.invalidate();
                menu_.render();
            }
            break;
        }

        case UiScreenState::REBOOT_PROMPT: {
            bool closed = reboot_prompt_.handleEvent(evt);
            if (closed) {
                state_ = UiScreenState::MENU;
                menu_.invalidate();
                menu_.render();
            }
            break;
        }

        case UiScreenState::ABOUT: {
            if (evt == UiEvent::PUSH_CLICK || evt == UiEvent::BACK_CLICK) {
                state_ = UiScreenState::MENU;
                menu_.invalidate();
                menu_.render();
            }
            break;
        }
    }
}

void UiManager::pollInput(uint8_t port_b_val) {
    if (!initialized_) return;

    UiEvent evt = input_.update(port_b_val);
    if (evt != UiEvent::NONE) {
        last_activity_s_ = last_telem_.uptime_s;
    }
    // No dashboard telemetry SPI on the fast path.
    dispatchEvent(evt, last_telem_, /*refresh_dashboard_telem=*/false);
}

void UiManager::update(uint8_t port_b_val, const DashboardTelemetry& telem) {
    if (!initialized_) return;

    last_telem_ = telem;

    const auto& app_cfg = Config::AppConfig::instance().data();
    if (prev_orient_ != app_cfg.display_orientation) {
        prev_orient_ = app_cfg.display_orientation;
        if (app_cfg.display_orientation == 1) {
            driver_.setOrientation(Orientation::PORTRAIT);
        } else {
            driver_.setOrientation(Orientation::LANDSCAPE);
        }
        if (state_ == UiScreenState::DASHBOARD) {
            dashboard_.renderFull(telem);
        } else if (state_ == UiScreenState::MENU) {
            menu_.invalidate();
            menu_.render();
        } else if (state_ == UiScreenState::EDITOR) {
            editor_.forceFullRender();
            editor_.render();
        }
    }

    UiEvent evt = input_.update(port_b_val);
    if (evt != UiEvent::NONE) {
        last_activity_s_ = telem.uptime_s;
    }

    applyBacklight(telem.uptime_s);
    dispatchEvent(evt, telem, /*refresh_dashboard_telem=*/true);
}

} // namespace display
