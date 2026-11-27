/**
 * @file gantry_console_thetapath.cpp
 * @brief Gantry test console: the test_theta_path sequence.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

static float pickThetaPathDeltaDeg(const Gantry::JointLimits &lim) {
  const float min_clamped = (lim.theta_min < -90.0f) ? -90.0f : lim.theta_min;
  const float max_clamped = (lim.theta_max > 90.0f) ? 90.0f : lim.theta_max;
  float d = (min_clamped < 0.0f) ? (0.5f * min_clamped)
                                 : (0.5f * max_clamped);
  if (fabsf(d) < 5.0f) {
    d = (fabsf(min_clamped) >= fabsf(max_clamped)) ? min_clamped
                                                   : max_clamped;
  }
  if (d < min_clamped) {
    d = min_clamped;
  }
  if (d > max_clamped) {
    d = max_clamped;
  }
  return d;
}

void testThetaPathTask(void *param) {
  auto *cfg = static_cast<GantryTestConsoleConfig *>(param);
  bool pass = false;

  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "ERROR: test_theta_path FAIL: gantry not initialized");
    g_testCycleInProgress = false;
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG,
           "[TEST_THETA_PATH] combined in-band X+Z+theta (25-75 window); "
           "dtheta from live thetalim");

  do {
    if (!cfg->gantry->isEnabled()) {
      ESP_LOGE(TAG, "ERROR: test_theta_path FAIL: enable and bring-up first");
      break;
    }
    if (cfg->gantry->isAlarmActive()) {
      ESP_LOGE(TAG, "ERROR: test_theta_path FAIL: alarm active");
      break;
    }
    if (!cfg->gantry->hasThetaAxis()) {
      ESP_LOGE(TAG, "ERROR: test_theta_path FAIL: no theta axis");
      break;
    }

    const Gantry::JointConfig cur0 = cfg->gantry->getCurrentJointConfig();
    if (fabsf(cur0.x) > 1.5f || fabsf(cur0.z) > 1.5f) {
      ESP_LOGI(TAG, "[TEST_THETA_PATH] retract to (0,0) first");
      if (!testCycleMoveLeg(cfg, "park", cur0.x, cur0.z, 0.0f, 0.0f, 0.0f, -1,
                            0, 0, 0)) {
        break;
      }
    }

    const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
    const float dtheta = pickThetaPathDeltaDeg(lim);
    ESP_LOGI(TAG,
             "[TEST_THETA_PATH] J: (220, 8, %.1f deg) from (0,0) thetalim "
             "%.1f .. %.1f",
             (double)dtheta, (double)lim.theta_min, (double)lim.theta_max);

    if (!testCycleMoveLeg(cfg, "J", 0.0f, 0.0f, 220.0f, 8.0f, dtheta, 1, 0, 0,
                          0) ||
        !testCycleMoveLeg(cfg, "K", 220.0f, 8.0f, 0.0f, 0.0f, 0.0f, 1, 0, 0,
                          0)) {
      break;
    }

    pass = true;
    ESP_LOGI(TAG, "OK test_theta_path PASS");
  } while (false);

  if (!pass) {
    if (cfg->gantry->isAbortRequested()) {
      ESP_LOGE(TAG, "ERROR: test_theta_path FAIL: aborted");
    } else {
      ESP_LOGE(TAG, "ERROR: test_theta_path FAIL");
    }
  }
  g_testCycleInProgress = false;
  vTaskDelete(nullptr);
}

}  // namespace GantryConsoleImpl
