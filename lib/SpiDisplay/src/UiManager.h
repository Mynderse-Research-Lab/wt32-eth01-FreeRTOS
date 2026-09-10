#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "St7789Driver.h"
#include "UiInput.h"
#include "UiDashboard.h"
#include "UiMenu.h"
#include "UiEditor.h"
#include "UiRebootPrompt.h"

namespace display {

enum class UiScreenState {
    DASHBOARD,
    MENU,
    EDITOR,
    REBOOT_PROMPT,
    ABOUT
};

class UiManager {
public:
    static UiManager& instance();

    bool init(const SpiDisplayConfig& cfg);
    void update(uint8_t port_b_val, const DashboardTelemetry& telem);

    St7789Driver& getDriver() { return driver_; }

private:
    UiManager();
    ~UiManager() = default;

    St7789Driver driver_;
    UiInput input_;
    UiDashboard dashboard_;
    UiMenu menu_;
    UiEditor editor_;
    UiRebootPrompt reboot_prompt_;

    UiScreenState state_{UiScreenState::DASHBOARD};
    bool initialized_{false};

    mcp23s17_handle_t mcp_{nullptr};
    int mcp_blk_pin_{-1};
    int esp_blk_pin_{-1};
    uint8_t pwm_cycle_{0};
    uint32_t last_activity_s_{0};
    int32_t prev_orient_{-1};
};

} // namespace display

#endif // UI_MANAGER_H
