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

    /** Full refresh: input + telemetry draw (call at display rate, e.g. 15 Hz). */
    void update(uint8_t port_b_val, const DashboardTelemetry& telem);

    /**
     * Encoder/button sample only (call at ~100 Hz). Dispatches UI events and
     * redraws menus/editors on interaction; skips dashboard telemetry SPI.
     */
    void pollInput(uint8_t port_b_val);

    St7789Driver& getDriver() { return driver_; }

private:
    UiManager();
    ~UiManager() = default;

    void applyBacklight(uint32_t uptime_s);
    void dispatchEvent(UiEvent evt, const DashboardTelemetry& telem,
                       bool refresh_dashboard_telem);

    St7789Driver driver_;
    UiInput input_;
    UiDashboard dashboard_;
    UiMenu menu_;
    UiEditor editor_;
    UiRebootPrompt reboot_prompt_;

    UiScreenState state_{UiScreenState::DASHBOARD};
    bool initialized_{false};
    DashboardTelemetry last_telem_{};

    mcp23s17_handle_t mcp_{nullptr};
    int mcp_blk_pin_{-1};
    int esp_blk_pin_{-1};
    uint8_t pwm_cycle_{0};
    uint32_t last_activity_s_{0};
    int32_t prev_orient_{-1};
};

} // namespace display

#endif // UI_MANAGER_H
