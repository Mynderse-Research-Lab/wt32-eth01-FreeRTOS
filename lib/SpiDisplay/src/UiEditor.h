#ifndef UI_EDITOR_H
#define UI_EDITOR_H

#include "St7789Driver.h"
#include "UiInput.h"
#include "ConfigSchema.h"
#include <string>

namespace display {

enum class EditorMode {
    PARAM_VALUE,
    PASSWORD_UNLOCK,
    PASSWORD_CHANGE_OLD,
    PASSWORD_CHANGE_NEW
};

enum class EditorResult {
    EDITING,
    CONFIRMED,
    CANCELLED
};

class UiEditor {
public:
    explicit UiEditor(St7789Driver& driver);

    void startParamEdit(const std::string& param_key);
    void startPasswordUnlock();
    void startPasswordChange();

    void render();
    EditorResult handleEvent(UiEvent event, bool& out_reboot_required);

    const std::string& getCurrentParamKey() const { return param_key_; }
    const std::string& getPasswordResult() const { return str_val_; }

    /// Mark the screen as dirty so the next render() redraws chrome.
    void forceFullRender() { force_full_render_ = true; }

private:
    void renderBox(const char* title);
    void renderControls();

    St7789Driver& driver_;
    EditorMode mode_{EditorMode::PARAM_VALUE};
    std::string param_key_;
    const Config::ParamSchema* schema_{nullptr};

    // Value buffers
    int32_t int_val_{0};
    float float_val_{0.0f};
    bool bool_val_{false};
    int32_t choice_val_{0};
    std::string str_val_;
    size_t str_cursor_{0};

    // Password change temporary
    std::string old_pw_buf_;
    bool force_full_render_{true};
};

} // namespace display

#endif // UI_EDITOR_H
