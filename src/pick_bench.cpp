/**
 * @file pick_bench.cpp
 * @brief Pick scheduler: GPIO marker, beltcal and netstat console entry points.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

// gripmark and vismark share GPIO17 (the TFT backlight). The pin leaves the
// backlight PWM when the first marker turns on and returns when both are off.
static bool setMarker(std::atomic<bool>& flag, const std::atomic<bool>& other, bool on,
                      const char* name, const char* meaning, char* out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return false;
    }
    const gpio_num_t pin = static_cast<gpio_num_t>(TFT_BLK_GPIO);
    if (on) {
        // Routing the pin to plain GPIO detaches the backlight PWM; the UI's
        // duty updates then go nowhere until both markers are off.
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << TFT_BLK_GPIO;
        io.mode = GPIO_MODE_OUTPUT;
        if (gpio_config(&io) != ESP_OK) {
            snprintf(out, out_len, "GPIO%d config failed", TFT_BLK_GPIO);
            return false;
        }
        gpio_set_level(pin, 0);
        flag.store(true, std::memory_order_release);
        snprintf(out, out_len, "%s marker on GPIO%d (TFT backlight off): %s%s", name,
                 TFT_BLK_GPIO, meaning,
                 other.load(std::memory_order_acquire) ? " (other marker also on)" : "");
        return true;
    }
    flag.store(false, std::memory_order_release);
    gpio_set_level(pin, 0);
    if (other.load(std::memory_order_acquire)) {
        snprintf(out, out_len, "%s marker off; GPIO%d stays a marker for the other one", name,
                 TFT_BLK_GPIO);
        return true;
    }
    if (ledc_set_pin(TFT_BLK_GPIO, LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) != ESP_OK) {
        snprintf(out, out_len, "marker off; backlight PWM not restored (reboot restores it)");
        return false;
    }
    snprintf(out, out_len, "%s marker off; backlight PWM restored on GPIO%d", name, TFT_BLK_GPIO);
    return true;
}

// Belt locator (beltcal): after bring-up the operator jogs (`move`) to each belt
// edge, the conveyor frame edges and the test battery; each joint X is saved to
// its cfg key at once. "z" records the joint Z with the open jaw tips
// on the belt (RAM until apply: the planner's belt surface, col_z_min_mm).
static float s_belt_z_mm = 0.0f;
static bool s_belt_z_set = false;

static bool beltCalReady(char* out, size_t out_len) {
    if (s_gantry == nullptr || !s_gantry->isWorkspaceCalibrated()) {
        snprintf(out, out_len, "run bring-up (calibrate all) first");
        return false;
    }
    return true;
}

}  // namespace PickSchedulerImpl

using namespace PickSchedulerImpl;

bool pickSchedulerGripMark(bool on, char* out, size_t out_len) {
    return setMarker(s_grip_mark, s_vis_mark, on, "grip",
                     "high from grip fire to MCP write done", out, out_len);
}

bool pickSchedulerVisMark(bool on, char* out, size_t out_len) {
    return setMarker(s_vis_mark, s_grip_mark, on, "vision",
                     "high from frame receipt to acceptDetect", out, out_len);
}

bool pickSchedulerBeltCal(const char* args, char* out, size_t out_len) {
    if (args == nullptr || out == nullptr || out_len == 0) {
        return false;
    }
    auto& app = Config::AppConfig::instance();
    const auto& c = app.data();
    bool reboot = false;

    struct Edge {
        const char* arg;
        const char* key;
        const char* what;
    };
    static const Edge kEdges[] = {
        {"near", "cv_belt_near_mm", "belt near edge"},
        {"far", "cv_belt_far_mm", "belt far edge"},
        {"cnvyr near", "cv_cnv_near_mm", "conveyor near edge"},
        {"cnvyr far", "cv_cnv_far_mm", "conveyor far edge"},
        {"batt", "batt_x", "test battery"},
    };
    for (const Edge& e : kEdges) {
        if (std::strcmp(args, e.arg) != 0) {
            continue;
        }
        if (!beltCalReady(out, out_len)) {
            return false;
        }
        const float x = s_gantry->getCurrentJointConfig().x;
        if (!app.setParamFloat(e.key, x, reboot)) {
            snprintf(out, out_len, "%s %.1f mm rejected", e.key, static_cast<double>(x));
            return false;
        }
        snprintf(out, out_len, "%s at joint X %.1f mm (saved %s)", e.what,
                 static_cast<double>(x), e.key);
        return true;
    }
    if (std::strcmp(args, "z") == 0) {
        if (!beltCalReady(out, out_len)) {
            return false;
        }
        s_belt_z_mm = s_gantry->getCurrentJointConfig().z;
        s_belt_z_set = true;
        snprintf(out, out_len,
                 "belt surface at joint Z %.2f mm (col_z_min_mm %.2f; beltcal apply saves it)",
                 static_cast<double>(s_belt_z_mm),
                 static_cast<double>(c.gantry_conveyor_collision_z_min_mm));
        return true;
    }

    const float width = std::fabs(c.conveyor_belt_far_x_mm - c.conveyor_belt_near_x_mm);
    const float centre = 0.5f * (c.conveyor_belt_near_x_mm + c.conveyor_belt_far_x_mm);
    // Vision and picksim x are end-effector X (tcp_x = vis_x_off_mm + x_across);
    // the recorded X are joint X. Picks run at theta 0 here, as when jogging.
    const auto tcpX = [](float joint_x) {
        return s_gantry != nullptr
                   ? s_gantry->forwardKinematics(Gantry::JointConfig(joint_x, 0.0f, 0.0f)).x
                   : joint_x;
    };
    const float centre_tcp = tcpX(centre);
    const float batt_tcp = tcpX(c.test_battery_x_mm);
    if (args[0] == '\0' || std::strcmp(args, "show") == 0) {
        int len =
            snprintf(out, out_len,
                     "belt joint X %.1f..%.1f: width %.1f centre %.1f (cv_belt_w_mm %.1f, "
                     "vis_x_off_mm %.1f) | conveyor joint X %.1f..%.1f (col_x_min_mm %.1f)",
                     static_cast<double>(c.conveyor_belt_near_x_mm),
                     static_cast<double>(c.conveyor_belt_far_x_mm), static_cast<double>(width),
                     static_cast<double>(centre), static_cast<double>(c.conveyor_belt_width_mm),
                     static_cast<double>(c.conveyor_x_across_to_gantry_x_offset_mm),
                     static_cast<double>(c.conveyor_frame_near_x_mm),
                     static_cast<double>(c.conveyor_frame_far_x_mm),
                     static_cast<double>(c.gantry_conveyor_collision_x_min_mm));
        // picksim x is across the belt from the camera: batt_x - vis_x_off_mm.
        if (len > 0 && static_cast<size_t>(len) < out_len) {
            len += snprintf(out + len, out_len - len, " | batt_x %.1f (picksim x %.1f)",
                            static_cast<double>(c.test_battery_x_mm),
                            static_cast<double>(batt_tcp -
                                                c.conveyor_x_across_to_gantry_x_offset_mm));
        }
        if (len > 0 && static_cast<size_t>(len) < out_len) {
            if (s_belt_z_set) {
                snprintf(out + len, out_len - len, " | belt Z %.2f (col_z_min_mm %.2f)",
                         static_cast<double>(s_belt_z_mm),
                         static_cast<double>(c.gantry_conveyor_collision_z_min_mm));
            } else {
                snprintf(out + len, out_len - len, " | belt Z unset (col_z_min_mm %.2f)",
                         static_cast<double>(c.gantry_conveyor_collision_z_min_mm));
            }
        }
        return true;
    }

    const bool with_centre = std::strcmp(args, "apply center") == 0;
    if (with_centre || std::strcmp(args, "apply") == 0) {
        const float old_w = c.conveyor_belt_width_mm;
        const float old_x = c.conveyor_x_across_to_gantry_x_offset_mm;
        const float old_cx = c.gantry_conveyor_collision_x_min_mm;
        const float old_z = c.gantry_conveyor_collision_z_min_mm;
        const float cnv_near = c.conveyor_frame_near_x_mm;
        if (!app.setParamFloat("cv_belt_w_mm", width, reboot)) {
            snprintf(out, out_len, "belt width %.1f mm rejected", static_cast<double>(width));
            return false;
        }
        // The camera sits at the belt centre (Kconfig CONVEYOR_X_OFFSET), so
        // vision x_across = 0 maps to the midpoint. Opt-in: it moves every pick.
        if (with_centre && !app.setParamFloat("vis_x_off_mm", centre_tcp, reboot)) {
            snprintf(out, out_len, "width saved; camera X %.1f rejected",
                     static_cast<double>(centre_tcp));
            return false;
        }
        // The collision zone starts at the conveyor's near frame edge (joint X).
        if (!app.setParamFloat("col_x_min_mm", cnv_near, reboot)) {
            snprintf(out, out_len, "conveyor near edge %.1f mm rejected",
                     static_cast<double>(cnv_near));
            return false;
        }
        if (s_belt_z_set && !app.setParamFloat("col_z_min_mm", s_belt_z_mm, reboot)) {
            snprintf(out, out_len, "belt Z %.2f mm rejected", static_cast<double>(s_belt_z_mm));
            return false;
        }
        int len = snprintf(out, out_len, "cv_belt_w_mm %.1f -> %.1f, col_x_min_mm %.1f -> %.1f",
                           static_cast<double>(old_w), static_cast<double>(width),
                           static_cast<double>(old_cx), static_cast<double>(cnv_near));
        if (with_centre && len > 0 && static_cast<size_t>(len) < out_len) {
            len += snprintf(out + len, out_len - len, ", vis_x_off_mm %.1f -> %.1f",
                            static_cast<double>(old_x), static_cast<double>(centre_tcp));
        }
        if (s_belt_z_set && len > 0 && static_cast<size_t>(len) < out_len) {
            snprintf(out + len, out_len - len, ", col_z_min_mm %.2f -> %.2f",
                     static_cast<double>(old_z), static_cast<double>(s_belt_z_mm));
        }
        return true;
    }
    snprintf(out, out_len,
             "usage: beltcal [near|far|cnvyr near|cnvyr far|batt|z|show|apply|apply center]");
    return false;
}

// netstat (N-2): CellNet receive counters, plus the EMAC's own drop counters:
// frames the DMA dropped for want of a free receive buffer, and receive FIFO
// overflows. The EMAC register clears on read, so the counts are summed here.
bool pickSchedulerNetStats(bool reset, char* out, size_t out_len) {
    static uint32_t s_emac_missed = 0;
    static uint32_t s_emac_overflow = 0;
    static bool s_emac_rollover = false;
    const uint32_t reg = EMAC_DMA.dmamissedfr.val;  // one read: it clears
    s_emac_missed += reg & 0xFFFFu;
    s_emac_overflow += (reg >> 17) & 0x7FFu;
    s_emac_rollover = s_emac_rollover || ((reg >> 16) & 1u) != 0 || ((reg >> 28) & 1u) != 0;
    if (s_net_l2 == nullptr) {
        snprintf(out, out_len, "CellNet is not running");
        return false;
    }
    if (reset) {
        s_net_l2->resetStats();
        s_emac_missed = 0;
        s_emac_overflow = 0;
        s_emac_rollover = false;
        snprintf(out, out_len, "netstat reset");
        return true;
    }
    const CellNet::CellNetStats st = s_net_l2->getStats();
    snprintf(out, out_len,
             "netstat t_ms=%lld tx=%lu rx=%lu rx_invalid=%lu seq_drops=%lu "
             "emac_missed=%lu emac_overflow=%lu%s",
             static_cast<long long>(esp_timer_get_time() / 1000),
             static_cast<unsigned long>(st.tx_frames), static_cast<unsigned long>(st.rx_frames),
             static_cast<unsigned long>(st.rx_invalid_frames),
             static_cast<unsigned long>(st.rx_sequence_drops),
             static_cast<unsigned long>(s_emac_missed),
             static_cast<unsigned long>(s_emac_overflow),
             s_emac_rollover ? " (an EMAC counter rolled over)" : "");
    return true;
}
