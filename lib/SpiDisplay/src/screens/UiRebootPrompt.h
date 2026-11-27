#ifndef UI_REBOOT_PROMPT_H
#define UI_REBOOT_PROMPT_H

#include "St7789Driver.h"
#include "UiInput.h"

namespace display {

class UiRebootPrompt {
public:
    explicit UiRebootPrompt(St7789Driver& driver);

    void show();
    void render();

    // Returns true if dialog should close, false if still showing.
    bool handleEvent(UiEvent event);

private:
    St7789Driver& driver_;
    bool select_yes_{true};
    bool force_full_render_{true};
};

} // namespace display

#endif // UI_REBOOT_PROMPT_H
