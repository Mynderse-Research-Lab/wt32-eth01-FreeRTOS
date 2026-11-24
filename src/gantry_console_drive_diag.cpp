/**
 * @file gantry_console_drive_diag.cpp
 * @brief Gantry test console: drive Ethernet link and auto-tune commands.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

// Read-only dump of one drive port's CIP Ethernet Link object (class 0xF6) for
// the Class 1 dropout investigation. X P1 is the W5500's link partner, so its
// counters are the switch-port counters for that cable. "clear" reads the
// counters with Get_and_Clear so the next read shows only new errors.
void runEthLinkCommand(const GantryTestConsoleConfig *cfg, const char *cmd) {
    char axis[8] = {};
    int port = 1;
    if (sscanf(cmd, "ethlink %7s %d", axis, &port) < 1 || port < 1 || port > 2) {
        ESP_LOGE(TAG, "Usage: ethlink <x|z|t> [port 1|2] [clear]");
        return;
    }
    const bool clear = strstr(cmd, " clear") != nullptr;
    const char *ip = nullptr;
    switch (tolower(static_cast<unsigned char>(axis[0]))) {
        case 'x': ip = CONFIG_EIP_TARGET_IP_X; break;
        case 'z': ip = CONFIG_EIP_TARGET_IP_Z; break;
        case 't': ip = CONFIG_EIP_TARGET_IP_THETA; break;
        default: break;
    }
    if (ip == nullptr) {
        ESP_LOGE(TAG, "Usage: ethlink <x|z|t> [port 1|2] [clear]");
        return;
    }
    if (cfg == nullptr || cfg->w5500_hal == nullptr) {
        ESP_LOGE(TAG, "W5500 not available");
        return;
    }

    eip::EipSocketW5500Tcp tcp(*(cfg->w5500_hal));
    if (!tcp.connect(ip, eip::EipSession::kDefaultPort)) {
        ESP_LOGE(TAG, "ethlink: connect %s:44818 failed", ip);
        return;
    }
    eip::EipSession session(tcp);
    if (!session.registerSession()) {
        ESP_LOGE(TAG, "ethlink: RegisterSession failed (%s)", ip);
        tcp.close();
        return;
    }

    const uint16_t inst = static_cast<uint16_t>(port);
    auto query = [&](eip::EthLinkAttr attr, bool get_clear, eip::Bytes &data) -> bool {
        const eip::Bytes req = get_clear ? eip::buildEthLinkGetAndClearRequest(inst, attr)
                                         : eip::buildEthLinkGetRequest(inst, attr);
        eip::Bytes reply;
        if (!session.sendExplicit(req, reply)) {
            ESP_LOGW(TAG, "  attr %u: no reply", static_cast<unsigned>(attr));
            return false;
        }
        eip::MessageRouterResponse mr;
        if (!eip::parseMessageRouterResponse(reply, mr)) {
            ESP_LOGW(TAG, "  attr %u: malformed reply", static_cast<unsigned>(attr));
            return false;
        }
        if (!mr.isSuccess()) {
            ESP_LOGW(TAG, "  attr %u: CIP status 0x%02X (not supported by this drive?)",
                     static_cast<unsigned>(attr), mr.general_status);
            return false;
        }
        data = mr.data;
        return true;
    };
    // name=value pairs, four per line so each log line stays short.
    auto printCounters = [](const char *title, const uint32_t *vals, size_t n,
                            const char *(*name)(size_t)) {
        ESP_LOGI(TAG, "  %s:", title);
        char line[160];
        size_t used = 0;
        for (size_t i = 0; i < n; ++i) {
            const int w = snprintf(line + used, sizeof(line) - used, " %s=%lu", name(i),
                                   static_cast<unsigned long>(vals[i]));
            if (w > 0) used += static_cast<size_t>(w);
            if (used >= sizeof(line)) used = sizeof(line) - 1;
            if ((i % 4) == 3 || i + 1 == n) {
                ESP_LOGI(TAG, "   %s", line);
                used = 0;
                line[0] = '\0';
            }
        }
    };

    ESP_LOGI(TAG, "=== Ethernet Link %c P%d (%s)%s ===",
             static_cast<char>(toupper(static_cast<unsigned char>(axis[0]))), port, ip,
             clear ? " - counters cleared after read" : "");
    eip::Bytes data;
    uint32_t speed = 0;
    if (query(eip::EthLinkAttr::kInterfaceSpeed, false, data) && eip::parseEthUdint(data, speed)) {
        ESP_LOGI(TAG, "  speed: %lu Mbit/s", static_cast<unsigned long>(speed));
    }
    eip::EthLinkFlags flags;
    if (query(eip::EthLinkAttr::kInterfaceFlags, false, data) &&
        eip::parseEthLinkFlags(data, flags)) {
        ESP_LOGI(TAG, "  flags=0x%08lX link=%s %s negotiation=%u (%s)%s%s",
                 static_cast<unsigned long>(flags.raw), flags.link_up ? "up" : "DOWN",
                 flags.full_duplex ? "full-duplex" : "HALF-duplex",
                 static_cast<unsigned>(flags.negotiation),
                 eip::ethNegotiationName(flags.negotiation),
                 flags.reset_required ? " reset-required" : "",
                 flags.hardware_fault ? " HARDWARE-FAULT" : "");
    }
    eip::EthInterfaceControl ctl;
    if (query(eip::EthLinkAttr::kInterfaceControl, false, data) &&
        eip::parseEthInterfaceControl(data, ctl)) {
        ESP_LOGI(TAG, "  control: auto-negotiate=%s forced=%s %u Mbit/s",
                 (ctl.control_bits & 0x01) ? "on" : "OFF",
                 (ctl.control_bits & 0x02) ? "full" : "half",
                 static_cast<unsigned>(ctl.forced_speed));
    }
    uint32_t ifc[eip::kEthInterfaceCounterCount] = {};
    if (query(eip::EthLinkAttr::kInterfaceCounters, clear, data) &&
        eip::parseEthCounters(data, ifc, eip::kEthInterfaceCounterCount)) {
        printCounters("interface counters", ifc, eip::kEthInterfaceCounterCount,
                      eip::ethInterfaceCounterName);
    }
    uint32_t media[eip::kEthMediaCounterCount] = {};
    if (query(eip::EthLinkAttr::kMediaCounters, clear, data) &&
        eip::parseEthCounters(data, media, eip::kEthMediaCounterCount)) {
        printCounters("media counters", media, eip::kEthMediaCounterCount,
                      eip::ethMediaCounterName);
    }

    session.unregisterSession();
    tcp.close();
}

void runAutotuneCommand(const GantryTestConsoleConfig *cfg, const char *cmd) {
#if !GANTRY_CONSOLE_AUTOTUNE
  (void)cfg;
  (void)cmd;
  ESP_LOGW(TAG, "autotune disabled for this version "
                "(use IndraWorks / KNX5100C / tools/hcs01_eng.py for drive tuning)");
#else
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }
  if (cfg->w5500_hal == nullptr) {
    ESP_LOGE(TAG, "W5500 HAL not available to console");
    return;
  }

  char axis[16] = {0};
  char action[16] = {0};
  int action_val = -1;
  int parsed = sscanf(cmd, "autotune %15s %15s %d", axis, action, &action_val);
  if (parsed < 1) {
    ESP_LOGE(TAG, "Usage: autotune <x|z|theta> [read|lock|reset|gain <N>]");
    return;
  }

  for (int i = 0; axis[i]; i++) axis[i] = (char)tolower((unsigned char)axis[i]);
  for (int i = 0; action[i]; i++) action[i] = (char)tolower((unsigned char)action[i]);

  bool is_theta = (strcmp(axis, "t") == 0 || strcmp(axis, "theta") == 0);
  bool is_x = (strcmp(axis, "x") == 0);
  bool is_z = (strcmp(axis, "z") == 0);

  if (!is_theta && !is_x && !is_z) {
    ESP_LOGE(TAG, "Unknown axis: %s. Use x, z, or theta.", axis);
    return;
  }

  if (cfg->gantry->isBusy()) {
    ESP_LOGE(TAG, "ERROR: Gantry is busy moving. Stop motion before auto-tuning.");
    return;
  }

  if (is_theta) {
    if (!cfg->gantry->hasThetaAxis()) {
      ESP_LOGE(TAG, "Theta axis not compiled in");
      return;
    }
    ESP_LOGI(TAG, "=== THETA (HCS01) COMWS INTERFACE ===");
    ESP_LOGI(TAG, "Connecting to HCS01 COMWS at %s:80...", hcs01::kDefaultEngIp);
    
    eip::EipSocketW5500Tcp tcp(*(cfg->w5500_hal));
    hcs01::Hcs01ComwsClient client(tcp);
    if (!client.connect(hcs01::kDefaultEngIp)) {
        ESP_LOGE(TAG, "Failed to connect/login to HCS01");
        return;
    }

    if (strcmp(action, "zero") == 0) {
        ESP_LOGI(TAG, "Login OK. Executing C0300 (Set Absolute Position) to zero the encoder...");
        if (client.runCommand("S-0-0447.0.0", 15000)) {
            ESP_LOGI(TAG, "C0300 succeeded. Saving to NV flash (C2200)...");
            if (client.runCommand("S-0-0264.0.0", 60000)) {
                ESP_LOGI(TAG, "C2200 saved! Origin zeroed at current physical pose.");
                ESP_LOGI(TAG, "You must now run 'home t' to capture ALIGNED origin.");
            } else {
                ESP_LOGE(TAG, "C2200 NV save failed.");
            }
        } else {
            ESP_LOGE(TAG, "C0300 failed. Check drive diagnostic log.");
        }
        client.disconnect();
        return;
    }

    ESP_LOGI(TAG, "Login OK. Starting C1800 Identification Sweep (+/- 45 deg)...");
    
    hcs01::TuningResult res = client.runAutotune();
    if (res.success) {
        ESP_LOGI(TAG, "Autotune Complete!");
        ESP_LOGI(TAG, "  Identified Inertia: %s", res.load_inertia);
        ESP_LOGI(TAG, "  Kp (Position)     : %s", res.kp);
        ESP_LOGI(TAG, "  Tn (Velocity)     : %s", res.tn);
        ESP_LOGI(TAG, "  Kv (Velocity)     : %s", res.kv);
        ESP_LOGI(TAG, "  Ka (Acceleration) : %s", res.ka);
        ESP_LOGI(TAG, "Parameters backed up to NV flash via C2200.");
    } else {
        ESP_LOGE(TAG, "Autotune failed or aborted. Check drive status.");
    }
    client.disconnect();
    return;
  }

  const char* target_ip = is_x ? CONFIG_EIP_TARGET_IP_X : CONFIG_EIP_TARGET_IP_Z;
  ESP_LOGI(TAG, "=== %c AXIS (Kinetix 5100) AUTO-TUNING ===", is_x ? 'X' : 'Z');
  
  if (is_x && !cfg->limit_switches_active) {
     ESP_LOGI(TAG, "WARNING: Ensure Z is in SAFE_Z band before moving X!");
  }

  ESP_LOGI(TAG, "Connecting CIP Explicit Messaging to %s:44818...", target_ip);
  eip::EipSocketW5500Tcp tcp(*(cfg->w5500_hal));
  if (!tcp.connect(target_ip, 44818)) {
      ESP_LOGE(TAG, "Failed to connect to %s", target_ip);
      return;
  }
  
  eip::EipSession session(tcp);
  if (!session.registerSession()) {
      ESP_LOGE(TAG, "CIP RegisterSession failed");
      tcp.close();
      return;
  }

  k5100::Kinetix5100TuningClient client(session);

  auto printSnapshot = [](k5100::Kinetix5100TuningClient& c) {
      k5100::TuningSnapshot snap;
      if (c.readTuningSnapshot(snap)) {
          ESP_LOGI(TAG, "Current Tuning Snapshot:");
          ESP_LOGI(TAG, "  GainAdjustMode    : %u (1=Mode1/Auto, 0=Manual)", snap.gain_adjust_mode);
          ESP_LOGI(TAG, "  SysGainResponse   : %u (Mode 1 bandwidth knob)", snap.sys_gain_response_level);
          ESP_LOGI(TAG, "  LoadInertiaRatio  : %u %%", snap.load_inertia_ratio);
          ESP_LOGI(TAG, "  PositionPropGain  : %u", snap.position_prop_gain);
          ESP_LOGI(TAG, "  VelocityPropGain  : %u", snap.velocity_prop_gain);
          ESP_LOGI(TAG, "  VelocityIntGain   : %u", snap.velocity_int_gain);
      } else {
          ESP_LOGE(TAG, "Failed to read tuning snapshot");
      }
  };

  if (parsed == 1) {
      ESP_LOGI(TAG, "Setting Mode 1 (Real-time Adaptive)...");
      if (client.setGainAdjustMode(k5100::kGainModeMode1)) {
          ESP_LOGI(TAG, "Mode 1 active. Drive will estimate inertia during motion.");
          ESP_LOGI(TAG, "REQUIREMENTS FOR INERTIA ESTIMATION (per UM004D Ch.9):");
          ESP_LOGI(TAG, "  1. Move axis back and forth (bi-directional)");
          ESP_LOGI(TAG, "  2. Speed must exceed 200 rpm (recommend >= 500 rpm)");
          ESP_LOGI(TAG, "  3. Accel 0 to 3000 rpm must occur in <= 1.5s");
          ESP_LOGI(TAG, "Run 'autotune %s read' to check live gains.", axis);
          printSnapshot(client);
      } else {
          ESP_LOGE(TAG, "Failed to set Mode 1");
      }
  } else if (strcmp(action, "read") == 0) {
      printSnapshot(client);
  } else if (strcmp(action, "lock") == 0) {
      ESP_LOGI(TAG, "Locking gains (Setting Mode 0 Manual)...");
      if (client.setGainAdjustMode(k5100::kGainModeManual)) {
          ESP_LOGI(TAG, "Gains locked. WARNING: Values are in RAM only.");
          ESP_LOGI(TAG, "Use Drive Panel or KNX5100C to 'Burn to EEPROM' before power cycle.");
          printSnapshot(client);
      } else {
          ESP_LOGE(TAG, "Failed to lock gains");
      }
  } else if (strcmp(action, "reset") == 0) {
      ESP_LOGI(TAG, "Resetting gains to factory default (Mode 4)...");
      if (client.setGainAdjustMode(k5100::kGainModeReset)) {
          ESP_LOGI(TAG, "Gains reset.");
      } else {
          ESP_LOGE(TAG, "Failed to reset gains");
      }
  } else if (strcmp(action, "gain") == 0 && parsed == 3) {
      ESP_LOGI(TAG, "Setting SysGainResponseLevel to %d...", action_val);
      if (client.setSysGainResponseLevel((uint16_t)action_val)) {
          ESP_LOGI(TAG, "Response level set.");
          printSnapshot(client);
      } else {
          ESP_LOGE(TAG, "Failed to set response level");
      }
  } else {
      ESP_LOGE(TAG, "Unknown action: %s", action);
  }

  session.unregisterSession();
  tcp.close();
#endif  // GANTRY_CONSOLE_AUTOTUNE
}

}  // namespace GantryConsoleImpl
