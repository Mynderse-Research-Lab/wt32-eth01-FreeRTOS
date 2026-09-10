#ifndef UI_MENU_H
#define UI_MENU_H

#include "St7789Driver.h"
#include "UiInput.h"
#include "AppConfig.h"
#include <vector>
#include <string>

namespace display {

enum class MenuItemType {
    SUBMENU,
    PARAM,
    ACTION_UNLOCK_DEV,
    ACTION_CHANGE_DEV_PW,
    ACTION_FACTORY_RESET,
    ACTION_ABOUT
};

struct MenuItem {
    std::string label;
    MenuItemType type;
    std::string param_key;
    int submenu_id{-1};
    Config::AccessLevel access{Config::AccessLevel::END_USER};
};

struct MenuLevel {
    std::string title;
    std::vector<MenuItem> items;
};

class UiMenu {
public:
    explicit UiMenu(St7789Driver& driver);

    void init();
    void reset();

    void render();
    bool handleEvent(UiEvent event, std::string& out_param_to_edit, MenuItemType& out_action);

    void popMenu();

    /// Mark the screen as dirty so the next render() does a full redraw.
    void invalidate() { force_full_render_ = true; }

private:
    void buildMenuTree();
    std::vector<size_t> getVisibleItemIndices() const;

    St7789Driver& driver_;
    std::vector<MenuLevel> menus_;
    std::vector<int> nav_stack_;       // Menu IDs stack
    std::vector<int> selection_stack_; // Selected indices stack
    std::vector<int> scroll_stack_;    // Scroll offset stack

    bool force_full_render_ = true;
    int last_rendered_sel_ = -1;
    int last_rendered_scroll_ = -1;
    int last_rendered_menu_ = -1;
};

} // namespace display

#endif // UI_MENU_H
