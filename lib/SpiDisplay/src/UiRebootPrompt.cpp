#include "UiRebootPrompt.h"

#if defined(ESP_PLATFORM)
#include "esp_system.h"
#endif

namespace display {

UiRebootPrompt::UiRebootPrompt(St7789Driver& driver)
    : driver_(driver) {}

void UiRebootPrompt::show() {
    select_yes_ = true;
    force_full_render_ = true;
    render();
}

void UiRebootPrompt::render() {
    int16_t bx = 25;
    int16_t by = 45;
    int16_t bw = driver_.getWidth() - 50;
    int16_t bh = 150;

    if (force_full_render_) {
        driver_.fillRect(bx, by, bw, bh, COLOR_PANEL_BG);
        driver_.drawRect(bx, by, bw, bh, COLOR_RED);
        driver_.drawRect(bx + 1, by + 1, bw - 2, bh - 2, COLOR_RED);

        // Title
        driver_.fillRect(bx + 2, by + 2, bw - 4, 24, COLOR_RED);
        driver_.drawStringCentered(by + 6, "REBOOT REQUIRED", COLOR_WHITE, COLOR_RED);

        driver_.drawStringCentered(85, "Settings were changed that", COLOR_WHITE, COLOR_PANEL_BG);
        driver_.drawStringCentered(105, "require a restart.", COLOR_WHITE, COLOR_PANEL_BG);
        driver_.drawStringCentered(130, "Reboot system now?", COLOR_YELLOW, COLOR_PANEL_BG);

        force_full_render_ = false;
    }

    // Buttons — always redraw (only two small rects)
    int16_t btn_y = 155;
    int16_t yes_x = 70;
    int16_t no_x  = 180;

    driver_.fillRect(yes_x, btn_y, 70, 24, select_yes_ ? COLOR_GREEN : COLOR_BORDER);
    driver_.drawString(yes_x + 20, btn_y + 4, "YES", select_yes_ ? COLOR_BLACK : COLOR_WHITE, select_yes_ ? COLOR_GREEN : COLOR_BORDER);

    driver_.fillRect(no_x, btn_y, 70, 24, !select_yes_ ? COLOR_GREEN : COLOR_BORDER);
    driver_.drawString(no_x + 25, btn_y + 4, "NO", !select_yes_ ? COLOR_BLACK : COLOR_WHITE, !select_yes_ ? COLOR_GREEN : COLOR_BORDER);
}

bool UiRebootPrompt::handleEvent(UiEvent event) {
    if (event == UiEvent::ROTATE_CW || event == UiEvent::ROTATE_CCW) {
        select_yes_ = !select_yes_;
        render();
        return false;
    } else if (event == UiEvent::BACK_CLICK) {
        return true; // Dismiss
    } else if (event == UiEvent::PUSH_CLICK) {
        if (select_yes_) {
            driver_.fillScreen(COLOR_BLACK);
            driver_.drawStringCentered(110, "REBOOTING...", COLOR_YELLOW, COLOR_BLACK);
#if defined(ESP_PLATFORM)
            esp_restart();
#endif
        }
        return true; // Dismiss if NO
    }
    return false;
}

} // namespace display
