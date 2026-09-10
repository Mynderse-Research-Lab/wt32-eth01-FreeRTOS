#include "UiMenu.h"
#include "ConfigSchema.h"
#include "AppConfig.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace display {

UiMenu::UiMenu(St7789Driver& driver)
    : driver_(driver) {
    init();
}

void UiMenu::init() {
    buildMenuTree();
    reset();
}

void UiMenu::reset() {
    nav_stack_.clear();
    selection_stack_.clear();
    scroll_stack_.clear();

    nav_stack_.push_back(0); // Root menu
    selection_stack_.push_back(0);
    scroll_stack_.push_back(0);

    force_full_render_ = true;
    last_rendered_sel_ = -1;
    last_rendered_scroll_ = -1;
    last_rendered_menu_ = -1;
}

void UiMenu::buildMenuTree() {
    menus_.clear();

    // 0: Root Menu
    MenuLevel root;
    root.title = "SETTINGS";
    root.items = {
        {"Motion Profile",       MenuItemType::SUBMENU, "", 1, Config::AccessLevel::END_USER},
        {"X Axis Kinematics",    MenuItemType::SUBMENU, "", 2, Config::AccessLevel::END_USER},
        {"Z Axis Kinematics",    MenuItemType::SUBMENU, "", 3, Config::AccessLevel::END_USER},
        {"Theta Axis Kinematics",MenuItemType::SUBMENU, "", 4, Config::AccessLevel::END_USER},
        {"Geometry & Gripper",   MenuItemType::SUBMENU, "", 5, Config::AccessLevel::END_USER},
        {"Conveyor Intercept",   MenuItemType::SUBMENU, "", 6, Config::AccessLevel::END_USER},
        {"Homing & Calibration", MenuItemType::SUBMENU, "", 7, Config::AccessLevel::END_USER},
        {"Network (LAN8720)",    MenuItemType::SUBMENU, "", 8, Config::AccessLevel::END_USER},
        {"Display Settings",     MenuItemType::SUBMENU, "", 9, Config::AccessLevel::END_USER},
        {"Developer Options",    MenuItemType::SUBMENU, "", 10, Config::AccessLevel::DEVELOPER},
        {"Unlock Developer Mode",MenuItemType::ACTION_UNLOCK_DEV, "", -1, Config::AccessLevel::END_USER},
        {"Factory Reset",        MenuItemType::ACTION_FACTORY_RESET, "", -1, Config::AccessLevel::END_USER},
        {"About Gantry",         MenuItemType::ACTION_ABOUT, "", -1, Config::AccessLevel::END_USER},
    };
    menus_.push_back(root);

    // Create submenus for each section
    auto createSubmenu = [this](const std::string& title, const char* section_name) -> int {
        MenuLevel lvl;
        lvl.title = title;
        for (const auto& s : Config::getParamSchemas()) {
            if (strcmp(s.section, section_name) == 0) {
                lvl.items.push_back({s.label, MenuItemType::PARAM, s.key, -1, s.access});
            }
        }
        menus_.push_back(lvl);
        return static_cast<int>(menus_.size() - 1);
    };

    // 1: Motion Profile
    createSubmenu("MOTION PROFILE", "Motion Profile");
    // 2: X Axis
    createSubmenu("X AXIS", "X Axis");
    // 3: Z Axis
    createSubmenu("Z AXIS", "Z Axis");
    // 4: Theta Axis
    createSubmenu("THETA AXIS", "Theta Axis");
    // 5: Geometry & Gripper
    createSubmenu("GEOMETRY & GRIPPER", "Geometry");
    // 6: Conveyor Intercept
    createSubmenu("CONVEYOR INTERCEPT", "Conveyor");
    // 7: Homing & Calibration
    createSubmenu("HOMING & CALIBRATION", "Homing & Cal");
    // 8: Network
    createSubmenu("NETWORK (LAN8720)", "Network");
    // 9: Display
    createSubmenu("DISPLAY SETTINGS", "Display");

    // 10: Developer Options Root
    MenuLevel dev_menu;
    dev_menu.title = "DEVELOPER OPTIONS";
    dev_menu.items = {
        {"Console Security",     MenuItemType::SUBMENU, "", 11, Config::AccessLevel::DEVELOPER},
        {"Console Options",      MenuItemType::SUBMENU, "", 12, Config::AccessLevel::DEVELOPER},
        {"EtherNet/IP Originator",MenuItemType::SUBMENU, "", 13, Config::AccessLevel::DEVELOPER},
        {"Change Dev Password",  MenuItemType::ACTION_CHANGE_DEV_PW, "", -1, Config::AccessLevel::DEVELOPER},
    };
    menus_.push_back(dev_menu);

    // 11: Console Security
    createSubmenu("CONSOLE SECURITY", "Console Sec");
    // 12: Console Options
    createSubmenu("CONSOLE OPTIONS", "Console Opt");
    // 13: EtherNet/IP
    createSubmenu("ETHERNET/IP", "EtherNet/IP");
}

std::vector<size_t> UiMenu::getVisibleItemIndices() const {
    std::vector<size_t> indices;
    if (nav_stack_.empty()) return indices;

    int cur_menu = nav_stack_.back();
    if (cur_menu < 0 || cur_menu >= static_cast<int>(menus_.size())) return indices;

    bool dev_unlocked = Config::AppConfig::instance().isDeveloperModeUnlocked();
    const auto& items = menus_[cur_menu].items;

    for (size_t i = 0; i < items.size(); ++i) {
        if (items[i].access == Config::AccessLevel::DEVELOPER && !dev_unlocked) {
            continue;
        }
        // If developer mode is already unlocked, don't show "Unlock Developer Mode" action
        if (items[i].type == MenuItemType::ACTION_UNLOCK_DEV && dev_unlocked) {
            continue;
        }
        indices.push_back(i);
    }
    return indices;
}

void UiMenu::render() {
    if (nav_stack_.empty()) return;
    int cur_menu = nav_stack_.back();
    if (cur_menu < 0 || cur_menu >= static_cast<int>(menus_.size())) return;

    auto visible = getVisibleItemIndices();
    int& sel = selection_stack_.back();
    int& scroll = scroll_stack_.back();

    if (visible.empty()) {
        sel = 0;
        scroll = 0;
    } else {
        if (sel >= static_cast<int>(visible.size())) sel = static_cast<int>(visible.size()) - 1;
        if (sel < 0) sel = 0;
    }

    constexpr int VISIBLE_ROWS = 10;
    constexpr int ROW_HEIGHT = 18;
    constexpr int START_Y = 28;

    // Adjust scroll
    if (sel < scroll) {
        scroll = sel;
    } else if (sel >= scroll + VISIBLE_ROWS) {
        scroll = sel - VISIBLE_ROWS + 1;
    }

    bool menu_changed = force_full_render_ || (cur_menu != last_rendered_menu_);
    bool scroll_changed = (scroll != last_rendered_scroll_);
    bool is_full_render = menu_changed || scroll_changed;

    if (menu_changed) {
        driver_.fillScreen(COLOR_BLACK);

        // 1. Header Bar
        driver_.fillRect(0, 0, driver_.getWidth(), 22, COLOR_HEADER_BG);
        char title_buf[48];
        std::snprintf(title_buf, sizeof(title_buf), "< %s", menus_[cur_menu].title.c_str());
        driver_.drawString(8, 3, title_buf, COLOR_WHITE, COLOR_HEADER_BG);
    }

    // 2. Draw Menu Items
    for (int r = 0; r < VISIBLE_ROWS; ++r) {
        int item_idx = scroll + r;
        if (item_idx >= static_cast<int>(visible.size())) break;

        bool is_selected = (item_idx == sel);
        bool was_selected = (item_idx == last_rendered_sel_);

        if (!is_full_render && (is_selected == was_selected)) {
            continue; // Skip unchanged items
        }

        size_t raw_idx = visible[item_idx];
        const auto& item = menus_[cur_menu].items[raw_idx];

        int16_t y = static_cast<int16_t>(START_Y + r * ROW_HEIGHT);
        uint16_t bg = is_selected ? COLOR_BLUE : COLOR_BLACK;
        uint16_t fg = is_selected ? COLOR_WHITE : COLOR_LIGHTGREY;

        driver_.fillRect(4, y, driver_.getWidth() - 8, ROW_HEIGHT, bg);
        driver_.drawString(12, y + 1, item.label.c_str(), fg, bg);

        if (item.type == MenuItemType::SUBMENU) {
            driver_.drawString(driver_.getWidth() - 24, y + 1, ">", COLOR_YELLOW, bg);
        } else if (item.type == MenuItemType::PARAM) {
            // Draw current parameter value
            const auto* schema = Config::findParamSchema(item.param_key.c_str());
            char val_str[32] = "";
            if (schema) {
                if (schema->type == Config::ParamType::INT) {
                    int32_t iv = 0;
                    Config::AppConfig::instance().getParamInt(schema->key, iv);
                    std::snprintf(val_str, sizeof(val_str), "%ld %s%s", (long)iv, schema->units, schema->reboot_required ? "*" : "");
                } else if (schema->type == Config::ParamType::FLOAT) {
                    float fv = 0.0f;
                    Config::AppConfig::instance().getParamFloat(schema->key, fv);
                    std::snprintf(val_str, sizeof(val_str), "%.2f %s%s", fv, schema->units, schema->reboot_required ? "*" : "");
                } else if (schema->type == Config::ParamType::BOOL) {
                    bool bv = false;
                    Config::AppConfig::instance().getParamBool(schema->key, bv);
                    std::snprintf(val_str, sizeof(val_str), "[%s]%s", bv ? "ON" : "OFF", schema->reboot_required ? "*" : "");
                } else if (schema->type == Config::ParamType::STRING) {
                    char sv[24] = "";
                    Config::AppConfig::instance().getParamString(schema->key, sv, sizeof(sv));
                    std::snprintf(val_str, sizeof(val_str), "%s%s", sv, schema->reboot_required ? "*" : "");
                } else if (schema->type == Config::ParamType::CHOICE) {
                    int32_t cv = 0;
                    Config::AppConfig::instance().getParamInt(schema->key, cv);
                    const char* c_name = (cv >= 0 && cv < (int)schema->choices.size()) ? schema->choices[cv] : "?";
                    std::snprintf(val_str, sizeof(val_str), "%s%s", c_name, schema->reboot_required ? "*" : "");
                }
            }
            int16_t vx = static_cast<int16_t>(driver_.getWidth() - 16 - strlen(val_str) * 8);
            driver_.drawString(vx, y + 1, val_str, is_selected ? COLOR_YELLOW : COLOR_CYAN, bg);
        }
    }

    if (menu_changed) {
        // 3. Footer Bar
        driver_.fillRect(0, 220, driver_.getWidth(), 20, COLOR_PANEL_BG);
        driver_.drawStringCentered(222, "ROTARY:MOVE | PUSH:SELECT | KO:BACK", COLOR_LIGHTGREY, COLOR_PANEL_BG);
    }

    force_full_render_ = false;
    last_rendered_sel_ = sel;
    last_rendered_scroll_ = scroll;
    last_rendered_menu_ = cur_menu;
}

bool UiMenu::handleEvent(UiEvent event, std::string& out_param_to_edit, MenuItemType& out_action) {
    if (nav_stack_.empty()) return false;
    auto visible = getVisibleItemIndices();
    if (visible.empty()) {
        if (event == UiEvent::BACK_CLICK) {
            popMenu();
            render();
        }
        return false;
    }

    int& sel = selection_stack_.back();

    if (event == UiEvent::ROTATE_CW) {
        if (sel < static_cast<int>(visible.size()) - 1) {
            sel++;
            render();
        }
        return false;
    } else if (event == UiEvent::ROTATE_CCW) {
        if (sel > 0) {
            sel--;
            render();
        }
        return false;
    } else if (event == UiEvent::BACK_CLICK) {
        popMenu();
        render();
        return false;
    } else if (event == UiEvent::PUSH_CLICK) {
        int cur_menu = nav_stack_.back();
        size_t raw_idx = visible[sel];
        const auto& item = menus_[cur_menu].items[raw_idx];

        if (item.type == MenuItemType::SUBMENU && item.submenu_id >= 0) {
            nav_stack_.push_back(item.submenu_id);
            selection_stack_.push_back(0);
            scroll_stack_.push_back(0);
            render();
            return false;
        } else if (item.type == MenuItemType::PARAM) {
            out_param_to_edit = item.param_key;
            out_action = MenuItemType::PARAM;
            return true;
        } else {
            out_action = item.type;
            return true;
        }
    }

    return false;
}

void UiMenu::popMenu() {
    if (nav_stack_.size() > 1) {
        nav_stack_.pop_back();
        selection_stack_.pop_back();
        scroll_stack_.pop_back();
        force_full_render_ = true;
    }
}

} // namespace display
