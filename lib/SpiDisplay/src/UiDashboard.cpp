#include "UiDashboard.h"
#include <cstdio>
#include <cstring>

namespace display {

UiDashboard::UiDashboard(St7789Driver& driver)
    : driver_(driver) {}

void UiDashboard::renderFull(const DashboardTelemetry& telem) {
    last_telem_ = telem;
    first_render_ = false;

    driver_.fillScreen(COLOR_BLACK);

    // 1. Header Bar
    driver_.fillRect(0, 0, driver_.getWidth(), 22, COLOR_HEADER_BG);
    driver_.drawString(8, 3, "GANTRY CONTROLLER", COLOR_WHITE, COLOR_HEADER_BG);

    char time_str[16];
    uint32_t hrs = telem.uptime_s / 3600;
    uint32_t mins = (telem.uptime_s % 3600) / 60;
    uint32_t secs = telem.uptime_s % 60;
    std::snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu", (unsigned long)hrs, (unsigned long)mins, (unsigned long)secs);
    driver_.drawString(driver_.getWidth() - 80, 3, time_str, COLOR_CYAN, COLOR_HEADER_BG);

    // 2. Motion Box (Left side: X=4, Y=26, W=154, H=110)
    driver_.drawRect(4, 26, 154, 110, COLOR_BORDER);
    driver_.fillRect(5, 27, 152, 18, COLOR_PANEL_BG);
    driver_.drawString(10, 28, "AXIS POSITIONS", COLOR_YELLOW, COLOR_PANEL_BG);

    char buf[64];
    std::snprintf(buf, sizeof(buf), "X: %7.2f mm", telem.x_mm);
    driver_.drawString(10, 50, buf, COLOR_WHITE, COLOR_BLACK);
    driver_.drawString(120, 50, telem.x_homed ? "[H]" : "[?]", telem.x_homed ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);

    std::snprintf(buf, sizeof(buf), "Z: %7.2f mm", telem.z_mm);
    driver_.drawString(10, 72, buf, COLOR_WHITE, COLOR_BLACK);
    driver_.drawString(120, 72, telem.z_homed ? "[H]" : "[?]", telem.z_homed ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);

    std::snprintf(buf, sizeof(buf), "O: %7.2f deg", telem.theta_deg);
    driver_.drawString(10, 94, buf, COLOR_WHITE, COLOR_BLACK);
    driver_.drawString(120, 94, telem.theta_homed ? "[H]" : "[?]", telem.theta_homed ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);

    std::snprintf(buf, sizeof(buf), "Grip: %s", telem.gripper_open ? "OPEN  " : "CLOSED");
    driver_.drawString(10, 116, buf, telem.gripper_open ? COLOR_CYAN : COLOR_ORANGE, COLOR_BLACK);

    // 3. Subsystem Box (Right side: X=162, Y=26, W=154, H=110)
    driver_.drawRect(162, 26, 154, 110, COLOR_BORDER);
    driver_.fillRect(163, 27, 152, 18, COLOR_PANEL_BG);
    driver_.drawString(168, 28, "SUBSYSTEMS", COLOR_YELLOW, COLOR_PANEL_BG);

    driver_.drawString(168, 50, "EIP X:", COLOR_LIGHTGREY, COLOR_BLACK);
    driver_.drawString(230, 50, telem.eip_x_ok ? "[ OK ]" : "[OFF]", telem.eip_x_ok ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);

    driver_.drawString(168, 72, "EIP Z:", COLOR_LIGHTGREY, COLOR_BLACK);
    driver_.drawString(230, 72, telem.eip_z_ok ? "[ OK ]" : "[OFF]", telem.eip_z_ok ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);

    driver_.drawString(168, 94, "EIP O:", COLOR_LIGHTGREY, COLOR_BLACK);
    driver_.drawString(230, 94, telem.eip_th_ok ? "[ OK ]" : "[OFF]", telem.eip_th_ok ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);

    driver_.drawString(168, 116, "DEV: ", COLOR_LIGHTGREY, COLOR_BLACK);
    driver_.drawString(210, 116, telem.dev_unlocked ? "[UNLOCKED]" : "[LOCKED]", telem.dev_unlocked ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);

    // 4. Status Box (Bottom: X=4, Y=140, W=312, H=72)
    driver_.drawRect(4, 140, 312, 72, COLOR_BORDER);

    std::snprintf(buf, sizeof(buf), "Plant LAN: %s (%s)", telem.lan_ip, telem.lan_link ? "UP" : "DOWN");
    driver_.drawString(10, 146, buf, telem.lan_link ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);

    std::snprintf(buf, sizeof(buf), "Belt Speed: %6.1f mm/s", telem.belt_speed_mm_s);
    driver_.drawString(10, 168, buf, COLOR_CYAN, COLOR_BLACK);

    std::snprintf(buf, sizeof(buf), "Motion State: %s", telem.motion_state);
    driver_.drawString(10, 190, buf, COLOR_WHITE, COLOR_BLACK);

    // 5. Footer Bar
    driver_.fillRect(0, 218, driver_.getWidth(), 22, COLOR_PANEL_BG);
    driver_.drawStringCentered(221, "PRESS ENCODER TO ENTER MENU", COLOR_YELLOW, COLOR_PANEL_BG);
}

void UiDashboard::updateTelemetry(const DashboardTelemetry& telem) {
    if (first_render_) {
        renderFull(telem);
        return;
    }

    char buf[64];

    // Uptime
    if (telem.uptime_s != last_telem_.uptime_s) {
        uint32_t hrs = telem.uptime_s / 3600;
        uint32_t mins = (telem.uptime_s % 3600) / 60;
        uint32_t secs = telem.uptime_s % 60;
        std::snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)hrs, (unsigned long)mins, (unsigned long)secs);
        driver_.drawString(driver_.getWidth() - 80, 3, buf, COLOR_CYAN, COLOR_HEADER_BG);
    }

    // Coordinates
    if (telem.x_mm != last_telem_.x_mm || telem.x_homed != last_telem_.x_homed) {
        std::snprintf(buf, sizeof(buf), "X: %7.2f mm", telem.x_mm);
        driver_.drawString(10, 50, buf, COLOR_WHITE, COLOR_BLACK);
        driver_.drawString(120, 50, telem.x_homed ? "[H]" : "[?]", telem.x_homed ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);
    }

    if (telem.z_mm != last_telem_.z_mm || telem.z_homed != last_telem_.z_homed) {
        std::snprintf(buf, sizeof(buf), "Z: %7.2f mm", telem.z_mm);
        driver_.drawString(10, 72, buf, COLOR_WHITE, COLOR_BLACK);
        driver_.drawString(120, 72, telem.z_homed ? "[H]" : "[?]", telem.z_homed ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);
    }

    if (telem.theta_deg != last_telem_.theta_deg || telem.theta_homed != last_telem_.theta_homed) {
        std::snprintf(buf, sizeof(buf), "O: %7.2f deg", telem.theta_deg);
        driver_.drawString(10, 94, buf, COLOR_WHITE, COLOR_BLACK);
        driver_.drawString(120, 94, telem.theta_homed ? "[H]" : "[?]", telem.theta_homed ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);
    }

    // Gripper
    if (telem.gripper_open != last_telem_.gripper_open) {
        std::snprintf(buf, sizeof(buf), "Grip: %s", telem.gripper_open ? "OPEN  " : "CLOSED");
        driver_.drawString(10, 116, buf, telem.gripper_open ? COLOR_CYAN : COLOR_ORANGE, COLOR_BLACK);
    }

    // Subsystems
    if (telem.eip_x_ok != last_telem_.eip_x_ok) {
        driver_.drawString(230, 50, telem.eip_x_ok ? "[ OK ]" : "[OFF]", telem.eip_x_ok ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);
    }
    if (telem.eip_z_ok != last_telem_.eip_z_ok) {
        driver_.drawString(230, 72, telem.eip_z_ok ? "[ OK ]" : "[OFF]", telem.eip_z_ok ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);
    }
    if (telem.eip_th_ok != last_telem_.eip_th_ok) {
        driver_.drawString(230, 94, telem.eip_th_ok ? "[ OK ]" : "[OFF]", telem.eip_th_ok ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);
    }
    if (telem.dev_unlocked != last_telem_.dev_unlocked) {
        driver_.drawString(210, 116, telem.dev_unlocked ? "[UNLOCKED]" : "[LOCKED]", telem.dev_unlocked ? COLOR_GREEN : COLOR_DARKGREY, COLOR_BLACK);
    }

    // Status details
    if (strcmp(telem.lan_ip, last_telem_.lan_ip) != 0 || telem.lan_link != last_telem_.lan_link) {
        std::snprintf(buf, sizeof(buf), "Plant LAN: %-15s (%s)", telem.lan_ip, telem.lan_link ? "UP  " : "DOWN");
        driver_.drawString(10, 146, buf, telem.lan_link ? COLOR_GREEN : COLOR_RED, COLOR_BLACK);
    }

    if (telem.belt_speed_mm_s != last_telem_.belt_speed_mm_s) {
        std::snprintf(buf, sizeof(buf), "Belt Speed: %6.1f mm/s", telem.belt_speed_mm_s);
        driver_.drawString(10, 168, buf, COLOR_CYAN, COLOR_BLACK);
    }

    if (strcmp(telem.motion_state, last_telem_.motion_state) != 0) {
        std::snprintf(buf, sizeof(buf), "Motion State: %-12s", telem.motion_state);
        driver_.drawString(10, 190, buf, COLOR_WHITE, COLOR_BLACK);
    }

    last_telem_ = telem;
}

} // namespace display
