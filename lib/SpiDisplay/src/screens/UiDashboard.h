#ifndef UI_DASHBOARD_H
#define UI_DASHBOARD_H

#include "St7789Driver.h"
#include <cstdint>

namespace display {

struct DashboardTelemetry {
    float x_mm = 0.0f;
    float z_mm = 0.0f;
    float theta_deg = 0.0f;
    bool x_homed = false;
    bool z_homed = false;
    bool theta_homed = false;
    bool gripper_open = true;
    bool eip_x_ok = false;
    bool eip_z_ok = false;
    bool eip_th_ok = false;
    char lan_ip[24] = "0.0.0.0";
    bool lan_link = false;
    float belt_speed_mm_s = 0.0f;
    const char* motion_state = "IDLE";
    uint32_t uptime_s = 0;
    bool dev_unlocked = false;
};

class UiDashboard {
public:
    explicit UiDashboard(St7789Driver& driver);

    void renderFull(const DashboardTelemetry& telem);
    void updateTelemetry(const DashboardTelemetry& telem);

private:
    St7789Driver& driver_;
    DashboardTelemetry last_telem_{};
    bool first_render_{true};
};

} // namespace display

#endif // UI_DASHBOARD_H
