#include "UiEditor.h"
#include "AppConfig.h"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <cmath>

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace display {

static const char* CHAR_SET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ_.-";
static const size_t CHAR_SET_LEN = 39;

UiEditor::UiEditor(St7789Driver& driver)
    : driver_(driver) {}

void UiEditor::startParamEdit(const std::string& param_key) {
    force_full_render_ = true;
    param_key_ = param_key;
    mode_ = EditorMode::PARAM_VALUE;
    schema_ = Config::findParamSchema(param_key.c_str());
    if (!schema_) return;

    if (schema_->type == Config::ParamType::INT) {
        Config::AppConfig::instance().getParamInt(schema_->key, int_val_);
    } else if (schema_->type == Config::ParamType::FLOAT) {
        Config::AppConfig::instance().getParamFloat(schema_->key, float_val_);
    } else if (schema_->type == Config::ParamType::BOOL) {
        Config::AppConfig::instance().getParamBool(schema_->key, bool_val_);
    } else if (schema_->type == Config::ParamType::CHOICE) {
        Config::AppConfig::instance().getParamInt(schema_->key, choice_val_);
    } else if (schema_->type == Config::ParamType::STRING) {
        char buf[32] = "";
        Config::AppConfig::instance().getParamString(schema_->key, buf, sizeof(buf));
        str_val_ = buf;
        str_cursor_ = 0;
    }
}

void UiEditor::startPasswordUnlock() {
    force_full_render_ = true;
    mode_ = EditorMode::PASSWORD_UNLOCK;
    schema_ = nullptr;
    str_val_ = "DEV_2026";
    str_cursor_ = 0;
}

void UiEditor::startPasswordChange() {
    force_full_render_ = true;
    mode_ = EditorMode::PASSWORD_CHANGE_OLD;
    schema_ = nullptr;
    str_val_ = "DEV_2026";
    old_pw_buf_ = "";
    str_cursor_ = 0;
}

void UiEditor::renderBox(const char* title) {
    int16_t bx = 20;
    int16_t by = 40;
    int16_t bw = driver_.getWidth() - 40;
    int16_t bh = 160;

    driver_.fillRect(bx, by, bw, bh, COLOR_PANEL_BG);
    driver_.drawRect(bx, by, bw, bh, COLOR_BORDER);
    driver_.drawRect(bx + 1, by + 1, bw - 2, bh - 2, COLOR_BORDER);

    // Title bar
    driver_.fillRect(bx + 2, by + 2, bw - 4, 24, COLOR_HEADER_BG);
    driver_.drawStringCentered(by + 6, title, COLOR_YELLOW, COLOR_HEADER_BG);
}

void UiEditor::renderControls() {
    int16_t by = 40 + 160 - 24;
    driver_.drawStringCentered(by, "PUSH:CONFIRM | KO:CANCEL", COLOR_LIGHTGREY, COLOR_PANEL_BG);
}

void UiEditor::render() {
    if (mode_ == EditorMode::PARAM_VALUE) {
        if (!schema_) return;

        if (force_full_render_) {
            char title[48];
            std::snprintf(title, sizeof(title), "EDIT: %s", schema_->label);
            renderBox(title);
            renderControls();
        }

        // Clear only the value area (y=85 to 110)
        driver_.fillRect(21, 85, 278, 26, COLOR_PANEL_BG);

        char buf[48];
        if (schema_->type == Config::ParamType::INT) {
            std::snprintf(buf, sizeof(buf), "[ %ld %s ]", (long)int_val_, schema_->units);
            driver_.drawStringCentered(90, buf, COLOR_WHITE, COLOR_PANEL_BG);

            if (force_full_render_) {
                std::snprintf(buf, sizeof(buf), "Range: %ld .. %ld", (long)schema_->min_val, (long)schema_->max_val);
                driver_.drawStringCentered(120, buf, COLOR_CYAN, COLOR_PANEL_BG);
            }
        } else if (schema_->type == Config::ParamType::FLOAT) {
            std::snprintf(buf, sizeof(buf), "[ %.2f %s ]", float_val_, schema_->units);
            driver_.drawStringCentered(90, buf, COLOR_WHITE, COLOR_PANEL_BG);

            if (force_full_render_) {
                std::snprintf(buf, sizeof(buf), "Range: %.1f .. %.1f", schema_->min_val, schema_->max_val);
                driver_.drawStringCentered(120, buf, COLOR_CYAN, COLOR_PANEL_BG);
            }
        } else if (schema_->type == Config::ParamType::BOOL) {
            std::snprintf(buf, sizeof(buf), "[ %s ]", bool_val_ ? "ENABLED" : "DISABLED");
            driver_.drawStringCentered(90, buf, bool_val_ ? COLOR_GREEN : COLOR_RED, COLOR_PANEL_BG);
            
            if (force_full_render_) driver_.drawStringCentered(120, "Rotate to Toggle", COLOR_CYAN, COLOR_PANEL_BG);
        } else if (schema_->type == Config::ParamType::CHOICE) {
            const char* opt = (choice_val_ >= 0 && choice_val_ < (int)schema_->choices.size())
                              ? schema_->choices[choice_val_] : "?";
            std::snprintf(buf, sizeof(buf), "[ %s ]", opt);
            driver_.drawStringCentered(90, buf, COLOR_WHITE, COLOR_PANEL_BG);
            
            if (force_full_render_) driver_.drawStringCentered(120, "Rotate to Select Option", COLOR_CYAN, COLOR_PANEL_BG);
        } else if (schema_->type == Config::ParamType::STRING) {
            driver_.drawStringCentered(85, str_val_.c_str(), COLOR_WHITE, COLOR_PANEL_BG);

            // Draw cursor under active char
            int16_t sx = (driver_.getWidth() - static_cast<int16_t>(str_val_.size() * 8)) / 2;
            int16_t cx = static_cast<int16_t>(sx + str_cursor_ * 8);
            driver_.drawHLine(cx, 103, 8, COLOR_YELLOW);
            driver_.drawHLine(cx, 104, 8, COLOR_YELLOW);

            if (force_full_render_) driver_.drawStringCentered(125, "Rot:Char | Push:Next", COLOR_CYAN, COLOR_PANEL_BG);
        }
    } else if (mode_ == EditorMode::PASSWORD_UNLOCK) {
        if (force_full_render_) {
            renderBox("UNLOCK DEVELOPER MODE");
            driver_.drawStringCentered(75, "Enter Dev Password:", COLOR_WHITE, COLOR_PANEL_BG);
            driver_.drawStringCentered(135, "Rot:Char | Push:Next", COLOR_LIGHTGREY, COLOR_PANEL_BG);
            renderControls();
        }
        
        driver_.fillRect(21, 98, 278, 24, COLOR_PANEL_BG);
        driver_.drawStringCentered(100, str_val_.c_str(), COLOR_YELLOW, COLOR_PANEL_BG);

        int16_t sx = (driver_.getWidth() - static_cast<int16_t>(str_val_.size() * 8)) / 2;
        int16_t cx = static_cast<int16_t>(sx + str_cursor_ * 8);
        driver_.drawHLine(cx, 118, 8, COLOR_CYAN);
        driver_.drawHLine(cx, 119, 8, COLOR_CYAN);
    } else if (mode_ == EditorMode::PASSWORD_CHANGE_OLD) {
        if (force_full_render_) {
            renderBox("CHANGE DEV PASSWORD");
            driver_.drawStringCentered(75, "Enter CURRENT Password:", COLOR_WHITE, COLOR_PANEL_BG);
            driver_.drawStringCentered(135, "Rot:Char | Push:Next", COLOR_LIGHTGREY, COLOR_PANEL_BG);
            renderControls();
        }
        
        driver_.fillRect(21, 98, 278, 24, COLOR_PANEL_BG);
        driver_.drawStringCentered(100, str_val_.c_str(), COLOR_YELLOW, COLOR_PANEL_BG);

        int16_t sx = (driver_.getWidth() - static_cast<int16_t>(str_val_.size() * 8)) / 2;
        int16_t cx = static_cast<int16_t>(sx + str_cursor_ * 8);
        driver_.drawHLine(cx, 118, 8, COLOR_CYAN);
        driver_.drawHLine(cx, 119, 8, COLOR_CYAN);
    } else if (mode_ == EditorMode::PASSWORD_CHANGE_NEW) {
        if (force_full_render_) {
            renderBox("NEW DEV PASSWORD");
            driver_.drawStringCentered(75, "Enter NEW Password:", COLOR_WHITE, COLOR_PANEL_BG);
            driver_.drawStringCentered(135, "Rot:Char | Push:Next", COLOR_LIGHTGREY, COLOR_PANEL_BG);
            renderControls();
        }
        
        driver_.fillRect(21, 98, 278, 24, COLOR_PANEL_BG);
        driver_.drawStringCentered(100, str_val_.c_str(), COLOR_GREEN, COLOR_PANEL_BG);

        int16_t sx = (driver_.getWidth() - static_cast<int16_t>(str_val_.size() * 8)) / 2;
        int16_t cx = static_cast<int16_t>(sx + str_cursor_ * 8);
        driver_.drawHLine(cx, 118, 8, COLOR_YELLOW);
        driver_.drawHLine(cx, 119, 8, COLOR_YELLOW);
    }
    
    force_full_render_ = false;
}

EditorResult UiEditor::handleEvent(UiEvent event, bool& out_reboot_required) {
    out_reboot_required = false;

    if (event == UiEvent::BACK_CLICK) {
        return EditorResult::CANCELLED;
    }

    if (mode_ == EditorMode::PARAM_VALUE) {
        if (!schema_) return EditorResult::CANCELLED;

        if (event == UiEvent::ROTATE_CW) {
            if (schema_->type == Config::ParamType::INT) {
                int32_t step = schema_->step > 0 ? static_cast<int32_t>(schema_->step) : 1;
                if (int_val_ + step <= static_cast<int32_t>(schema_->max_val)) {
                    int_val_ += step;
                    render();
                }
            } else if (schema_->type == Config::ParamType::FLOAT) {
                float step = schema_->step > 0 ? schema_->step : 1.0f;
                if (float_val_ + step <= schema_->max_val) {
                    float_val_ += step;
                    render();
                }
            } else if (schema_->type == Config::ParamType::BOOL) {
                bool_val_ = !bool_val_;
                render();
            } else if (schema_->type == Config::ParamType::CHOICE) {
                if (choice_val_ < static_cast<int>(schema_->choices.size()) - 1) {
                    choice_val_++;
                    render();
                }
            } else if (schema_->type == Config::ParamType::STRING) {
                if (!str_val_.empty() && str_cursor_ < str_val_.size()) {
                    char c = str_val_[str_cursor_];
                    const char* p = strchr(CHAR_SET, c);
                    size_t idx = p ? (p - CHAR_SET) : 0;
                    idx = (idx + 1) % CHAR_SET_LEN;
                    str_val_[str_cursor_] = CHAR_SET[idx];
                    render();
                }
            }
        } else if (event == UiEvent::ROTATE_CCW) {
            if (schema_->type == Config::ParamType::INT) {
                int32_t step = schema_->step > 0 ? static_cast<int32_t>(schema_->step) : 1;
                if (int_val_ - step >= static_cast<int32_t>(schema_->min_val)) {
                    int_val_ -= step;
                    render();
                }
            } else if (schema_->type == Config::ParamType::FLOAT) {
                float step = schema_->step > 0 ? schema_->step : 1.0f;
                if (float_val_ - step >= schema_->min_val) {
                    float_val_ -= step;
                    render();
                }
            } else if (schema_->type == Config::ParamType::BOOL) {
                bool_val_ = !bool_val_;
                render();
            } else if (schema_->type == Config::ParamType::CHOICE) {
                if (choice_val_ > 0) {
                    choice_val_--;
                    render();
                }
            } else if (schema_->type == Config::ParamType::STRING) {
                if (!str_val_.empty() && str_cursor_ < str_val_.size()) {
                    char c = str_val_[str_cursor_];
                    const char* p = strchr(CHAR_SET, c);
                    size_t idx = p ? (p - CHAR_SET) : 0;
                    idx = (idx + CHAR_SET_LEN - 1) % CHAR_SET_LEN;
                    str_val_[str_cursor_] = CHAR_SET[idx];
                    render();
                }
            }
        } else if (event == UiEvent::PUSH_CLICK) {
            if (schema_->type == Config::ParamType::STRING) {
                if (str_cursor_ < str_val_.size() - 1) {
                    str_cursor_++;
                    render();
                    return EditorResult::EDITING;
                }
            }

            // Commit value to AppConfig
            if (schema_->type == Config::ParamType::INT) {
                Config::AppConfig::instance().setParamInt(schema_->key, int_val_, out_reboot_required);
            } else if (schema_->type == Config::ParamType::FLOAT) {
                Config::AppConfig::instance().setParamFloat(schema_->key, float_val_, out_reboot_required);
            } else if (schema_->type == Config::ParamType::BOOL) {
                Config::AppConfig::instance().setParamBool(schema_->key, bool_val_, out_reboot_required);
            } else if (schema_->type == Config::ParamType::CHOICE) {
                Config::AppConfig::instance().setParamInt(schema_->key, choice_val_, out_reboot_required);
            } else if (schema_->type == Config::ParamType::STRING) {
                Config::AppConfig::instance().setParamString(schema_->key, str_val_.c_str(), out_reboot_required);
            }
            return EditorResult::CONFIRMED;
        }
    } else if (mode_ == EditorMode::PASSWORD_UNLOCK ||
               mode_ == EditorMode::PASSWORD_CHANGE_OLD ||
               mode_ == EditorMode::PASSWORD_CHANGE_NEW) {

        if (event == UiEvent::ROTATE_CW) {
            if (!str_val_.empty() && str_cursor_ < str_val_.size()) {
                char c = str_val_[str_cursor_];
                const char* p = strchr(CHAR_SET, c);
                size_t idx = p ? (p - CHAR_SET) : 0;
                idx = (idx + 1) % CHAR_SET_LEN;
                str_val_[str_cursor_] = CHAR_SET[idx];
                render();
            }
        } else if (event == UiEvent::ROTATE_CCW) {
            if (!str_val_.empty() && str_cursor_ < str_val_.size()) {
                char c = str_val_[str_cursor_];
                const char* p = strchr(CHAR_SET, c);
                size_t idx = p ? (p - CHAR_SET) : 0;
                idx = (idx + CHAR_SET_LEN - 1) % CHAR_SET_LEN;
                str_val_[str_cursor_] = CHAR_SET[idx];
                render();
            }
        } else if (event == UiEvent::PUSH_CLICK) {
            if (str_cursor_ < str_val_.size() - 1) {
                str_cursor_++;
                render();
                return EditorResult::EDITING;
            }

            if (mode_ == EditorMode::PASSWORD_UNLOCK) {
                bool ok = Config::AppConfig::instance().unlockDeveloperMode(str_val_.c_str());
                if (ok) {
                    return EditorResult::CONFIRMED;
                } else {
                    // Flash error and reset cursor
                    driver_.drawStringCentered(145, "INCORRECT PASSWORD!", COLOR_RED, COLOR_PANEL_BG);
#if defined(ESP_PLATFORM)
                    vTaskDelay(pdMS_TO_TICKS(1000));
#endif
                    str_cursor_ = 0;
                    render();
                    return EditorResult::EDITING;
                }
            } else if (mode_ == EditorMode::PASSWORD_CHANGE_OLD) {
                // Verify old password
                const auto& d = Config::AppConfig::instance().data();
                if (strcmp(str_val_.c_str(), d.dev_mode_password) != 0) {
                    driver_.drawStringCentered(145, "INCORRECT CURRENT PW!", COLOR_RED, COLOR_PANEL_BG);
#if defined(ESP_PLATFORM)
                    vTaskDelay(pdMS_TO_TICKS(1000));
#endif
                    str_cursor_ = 0;
                    render();
                    return EditorResult::EDITING;
                }
                old_pw_buf_ = str_val_;
                mode_ = EditorMode::PASSWORD_CHANGE_NEW;
                str_val_ = "NEW_2026";
                str_cursor_ = 0;
                render();
                return EditorResult::EDITING;
            } else if (mode_ == EditorMode::PASSWORD_CHANGE_NEW) {
                bool ok = Config::AppConfig::instance().changeDeveloperPassword(old_pw_buf_.c_str(), str_val_.c_str());
                if (ok) {
                    driver_.drawStringCentered(145, "PASSWORD CHANGED!", COLOR_GREEN, COLOR_PANEL_BG);
#if defined(ESP_PLATFORM)
                    vTaskDelay(pdMS_TO_TICKS(1000));
#endif
                    return EditorResult::CONFIRMED;
                }
            }
        }
    }

    return EditorResult::EDITING;
}

} // namespace display
