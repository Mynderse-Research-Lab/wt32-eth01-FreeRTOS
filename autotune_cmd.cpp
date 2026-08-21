#include "gantry_test_console.h"
#include "W5500Hal.h"
#include "EipSocketW5500.h"
#include "EipSession.h"
#include "Kinetix5100TuningClient.h"
#include "Hcs01ComwsClient.h"

void runAutotuneCommand(const GantryTestConsoleConfig *cfg, const char *cmd) {
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
    ESP_LOGI(TAG, "=== THETA (HCS01) INERTIA AUTO-TUNING ===");
    ESP_LOGI(TAG, "Connecting to HCS01 COMWS at " CONFIG_EIP_TARGET_IP_THETA ":80...");
    
    eip::EipSocketW5500Tcp tcp(*(cfg->w5500_hal));
    hcs01::Hcs01ComwsClient client(tcp);
    if (!client.connect(CONFIG_EIP_TARGET_IP_THETA)) {
        ESP_LOGE(TAG, "Failed to connect/login to HCS01");
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
}
