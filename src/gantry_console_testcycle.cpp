/**
 * @file gantry_console_testcycle.cpp
 * @brief Gantry test console: the test_cycle sequence.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

const char *gantryErrorName(Gantry::GantryError err) {
  switch (err) {
    case Gantry::GantryError::OK:
      return "OK";
    case Gantry::GantryError::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case Gantry::GantryError::MOTOR_NOT_ENABLED:
      return "MOTOR_NOT_ENABLED";
    case Gantry::GantryError::NOT_CALIBRATED:
      return "NOT_CALIBRATED";
    case Gantry::GantryError::ALREADY_MOVING:
      return "ALREADY_MOVING";
    case Gantry::GantryError::INVALID_POSITION:
      return "INVALID_POSITION";
    case Gantry::GantryError::INVALID_PARAMETER:
      return "INVALID_PARAMETER";
    case Gantry::GantryError::TIMEOUT:
      return "TIMEOUT (alarm?)";
    case Gantry::GantryError::LIMIT_SWITCH_FAILED:
      return "LIMIT_SWITCH_FAILED";
    case Gantry::GantryError::CALIBRATION_FAILED:
      return "CALIBRATION_FAILED";
    case Gantry::GantryError::CONVERSION_ERROR:
      return "CONVERSION_ERROR";
    default:
      return "UNKNOWN";
  }
}

static bool testCycleWaitMs(const GantryTestConsoleConfig *cfg, uint32_t ms) {
  const TickType_t start = xTaskGetTickCount();
  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms)) {
    if (cfg->gantry->isAbortRequested()) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return true;
}

static bool testCycleWaitIdle(const GantryTestConsoleConfig *cfg, uint32_t timeout_ms) {
  const TickType_t start = xTaskGetTickCount();
  while (cfg->gantry->isBusy() || cfg->gantry->eipBringUpInProgress()) {
    if (cfg->gantry->isAbortRequested()) {
      return false;
    }
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
      ESP_LOGE(TAG, "ERROR: test_cycle idle wait timed out after %lu ms",
               (unsigned long)timeout_ms);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return true;
}

static bool testCyclePoseNear(const GantryTestConsoleConfig *cfg, float x_mm, float z_mm,
                       float eps_mm, bool check_theta, float theta_deg,
                       float eps_deg) {
  const Gantry::JointConfig cur = cfg->gantry->getCurrentJointConfig();
  const float dx = fabsf(cur.x - x_mm);
  const float dz = fabsf(cur.z - z_mm);
  if (dx > eps_mm || dz > eps_mm) {
    ESP_LOGE(TAG,
             "ERROR: test_cycle pose miss: got x=%.3f z=%.3f, want x=%.3f "
             "z=%.3f (eps=%.1f mm)",
             (double)cur.x, (double)cur.z, (double)x_mm, (double)z_mm,
             (double)eps_mm);
    return false;
  }
  if (check_theta && cfg->gantry->hasThetaAxis()) {
    if (fabsf(cur.theta - theta_deg) > eps_deg) {
      ESP_LOGE(TAG,
               "ERROR: test_cycle pose miss: got theta=%.3f, want %.3f "
               "(eps=%.1f deg)",
               (double)cur.theta, (double)theta_deg, (double)eps_deg);
      return false;
    }
    ESP_LOGI(TAG,
             "[TEST_CYCLE] pose ok: x=%.3f z=%.3f theta=%.3f (target %.3f / "
             "%.3f / %.3f)",
             (double)cur.x, (double)cur.z, (double)cur.theta, (double)x_mm,
             (double)z_mm, (double)theta_deg);
    return true;
  }
  ESP_LOGI(TAG, "[TEST_CYCLE] pose ok: x=%.3f z=%.3f (target %.3f / %.3f)",
           (double)cur.x, (double)cur.z, (double)x_mm, (double)z_mm);
  return true;
}

bool testCycleMoveLeg(const GantryTestConsoleConfig *cfg, const char *name,
                      float from_x_mm, float from_z_mm, float x_mm, float z_mm,
                      float theta_deg, int expected_segs, uint32_t speed_deg_per_s,
                      uint32_t accel_deg_per_s2, uint32_t decel_deg_per_s2,
                      uint32_t speed_mm_per_s, uint32_t accel_mm_per_s2,
                      uint32_t decel_mm_per_s2) {
  constexpr float kPoseEpsMm = 1.5f;
  constexpr float kPoseEpsDeg = 0.015f;
  const uint32_t lin_v =
      (speed_mm_per_s > 0u) ? speed_mm_per_s : g_moveSpeedMmPerS;
  const uint32_t lin_a =
      (accel_mm_per_s2 > 0u) ? accel_mm_per_s2 : g_moveAccelMmPerS2;
  const uint32_t lin_d =
      (decel_mm_per_s2 > 0u) ? decel_mm_per_s2 : g_moveDecelMmPerS2;
  const uint32_t th_v =
      (speed_deg_per_s > 0u) ? speed_deg_per_s : g_moveSpeedDegPerS;
  const uint32_t th_a =
      (accel_deg_per_s2 > 0u) ? accel_deg_per_s2 : g_moveAccelDegPerS2;
  const uint32_t th_d =
      (decel_deg_per_s2 > 0u) ? decel_deg_per_s2 : g_moveDecelDegPerS2;
  const Gantry::JointConfig cur = cfg->gantry->getCurrentJointConfig();
  const float ceiling = cfg->gantry->traverseClearanceZMm();
  Gantry::Path::PathSegment segs[3];
  const size_t n_cmd = Gantry::Path::planSegments(from_x_mm, from_z_mm, x_mm,
                                                  z_mm, ceiling, segs);
  const size_t n_live =
      Gantry::Path::planSegments(cur.x, cur.z, x_mm, z_mm, ceiling, segs);
  ESP_LOGI(TAG,
           "[TEST_CYCLE] %s: move (%.1f, %.1f, %.1f deg) live from "
           "(%.3f, %.3f, %.3f) n=%u; commanded from (%.1f, %.1f) n=%u; "
           "v=%lu mm/s a=%lu d=%lu; theta v=%lu deg/s a=%lu d=%lu",
           name, (double)x_mm, (double)z_mm, (double)theta_deg, (double)cur.x,
           (double)cur.z, (double)cur.theta, (unsigned)n_live,
           (double)from_x_mm, (double)from_z_mm, (unsigned)n_cmd,
           (unsigned long)lin_v, (unsigned long)lin_a,
           (unsigned long)lin_d, (unsigned long)th_v,
           (unsigned long)th_a, (unsigned long)th_d);
  if (expected_segs >= 0 && static_cast<int>(n_cmd) != expected_segs) {
    ESP_LOGE(TAG,
             "ERROR: test_cycle %s expected %d path segs from commanded "
             "start, planner gave %u",
             name, expected_segs, (unsigned)n_cmd);
    return false;
  }

  Gantry::JointConfig target{};
  target.x = x_mm;
  target.z = z_mm;
  target.theta = theta_deg;
  const Gantry::GantryError result =
      cfg->gantry->moveTo(target, lin_v, th_v, lin_a, lin_d, th_a, th_d);
  if (result != Gantry::GantryError::OK) {
    ESP_LOGE(TAG, "ERROR: test_cycle %s move failed: %s", name,
             gantryErrorName(result));
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  if (!testCycleWaitIdle(cfg, Gantry::Constants::TRAVEL_MEASUREMENT_TIMEOUT_MS)) {
    return false;
  }
  if (cfg->gantry->isAlarmActive()) {
    ESP_LOGE(TAG, "ERROR: test_cycle %s alarm after move", name);
    return false;
  }
  return testCyclePoseNear(cfg, x_mm, z_mm, kPoseEpsMm,
                           cfg->gantry->hasThetaAxis(), theta_deg, kPoseEpsDeg);
}

static bool testCycleGripAction(const GantryTestConsoleConfig *cfg, bool grip_closed) {
  ESP_LOGI(TAG, "[TEST_CYCLE] Gripper -> %s", grip_closed ? "CLOSED (GRIP)" : "OPEN (RELEASE)");
  cfg->gantry->grip(grip_closed);
  const uint32_t wait_ms =
      static_cast<uint32_t>(grip_closed ? stored().gantry_gripper_close_time_ms
                                        : stored().gantry_gripper_open_time_ms) + 60u;
  if (!testCycleWaitMs(cfg, wait_ms)) {
    ESP_LOGE(TAG, "ERROR: test_cycle aborted during gripper action");
    return false;
  }
  return true;
}

static bool testCycleSafetyLimitsRejection(const GantryTestConsoleConfig *cfg) {
  ESP_LOGI(TAG, "[TEST_CYCLE] Stage 3: Testing software limit bounds rejection...");
  const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
  
  // Test negative X limit rejection
  Gantry::JointConfig bad_target = cfg->gantry->getCurrentJointConfig();
  bad_target.x = lim.x_min - 50.0f;
  Gantry::GantryError err = cfg->gantry->moveTo(bad_target, 50, 30, 500, 500, 100, 100);
  if (err == Gantry::GantryError::OK) {
    ESP_LOGE(TAG, "ERROR: test_cycle FAIL: moveTo accepted out-of-bounds negative X target!");
    return false;
  }
  
  // Test positive Z limit rejection
  bad_target = cfg->gantry->getCurrentJointConfig();
  bad_target.z = lim.z_max + 50.0f;
  err = cfg->gantry->moveTo(bad_target, 50, 30, 500, 500, 100, 100);
  if (err == Gantry::GantryError::OK) {
    ESP_LOGE(TAG, "ERROR: test_cycle FAIL: moveTo accepted out-of-bounds positive Z target!");
    return false;
  }
  
  ESP_LOGI(TAG, "[TEST_CYCLE] Bounds rejection OK: out-of-envelope commands correctly rejected");
  return true;
}

static bool testCyclePickAndPlaceSequence(const GantryTestConsoleConfig *cfg) {
  ESP_LOGI(TAG, "[TEST_CYCLE] Stage 5: Pick-and-Place Automation Flow with Gripper");
  const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
  const float pick_x = 120.0f;
  const float place_x = 280.0f;
  const float pick_z = (lim.z_max > 120.0f) ? 120.0f : (lim.z_max * 0.8f);
  const float safe_z = cfg->gantry->traverseClearanceZMm();
  const float th_pick = (lim.theta_max >= 30.0f) ? 30.0f : 0.0f;
  const float th_place = (lim.theta_min <= -30.0f) ? -30.0f : 0.0f;

  // 1. Approach pick station at safe Z
  if (!testCycleMoveLeg(cfg, "P1_ApproachPick", 0.0f, safe_z, pick_x, safe_z, th_pick, 1, 0, 0, 0)) {
    return false;
  }
  // 2. Descend to pick height
  if (!testCycleMoveLeg(cfg, "P2_DescendPick", pick_x, safe_z, pick_x, pick_z, th_pick, 1, 0, 0, 0)) {
    return false;
  }
  // 3. Grip part
  if (!testCycleGripAction(cfg, true)) {
    return false;
  }
  // 4. Retract to safe Z
  if (!testCycleMoveLeg(cfg, "P3_AscendPick", pick_x, pick_z, pick_x, safe_z, th_pick, 1, 0, 0, 0)) {
    return false;
  }
  // 5. High-speed transfer to place station
  if (!testCycleMoveLeg(cfg, "P4_TransferPlace", pick_x, safe_z, place_x, safe_z, th_place, 1, 0, 0, 0,
                        120, 2000, 2000)) {
    return false;
  }
  // 6. Descend to place height
  if (!testCycleMoveLeg(cfg, "P5_DescendPlace", place_x, safe_z, place_x, pick_z, th_place, 1, 0, 0, 0)) {
    return false;
  }
  // 7. Release part
  if (!testCycleGripAction(cfg, false)) {
    return false;
  }
  // 8. Retract to safe Z & zero theta
  if (!testCycleMoveLeg(cfg, "P6_AscendClear", place_x, pick_z, place_x, safe_z, 0.0f, 1, 0, 0, 0)) {
    return false;
  }
  ESP_LOGI(TAG, "[TEST_CYCLE] Pick-and-place sequence PASSED");
  return true;
}

static bool testCycleDynamicsStress(const GantryTestConsoleConfig *cfg) {
  ESP_LOGI(TAG, "[TEST_CYCLE] Stage 6: Multi-Velocity Dynamics & Creep Positioning Validation");
  const float cur_x = cfg->gantry->getCurrentJointConfig().x;
  const float safe_z = cfg->gantry->traverseClearanceZMm();

  // 1. Fine creep move (15 mm/s) - tests low-speed PID / position hold & resolution
  ESP_LOGI(TAG, "[TEST_CYCLE] Dynamic Leg V1: Fine creep positioning (15 mm/s)");
  if (!testCycleMoveLeg(cfg, "V1_Creep", cur_x, safe_z, cur_x - 20.0f, safe_z, 0.0f, 1, 0, 0, 0,
                        15, 300, 300)) {
    return false;
  }

  // 2. High dynamics traverse (150 mm/s, 3000 mm/s2) - tests rapid acceleration & settling
  ESP_LOGI(TAG, "[TEST_CYCLE] Dynamic Leg V2: High dynamics traverse (150 mm/s, 3000 mm/s2)");
  if (!testCycleMoveLeg(cfg, "V2_HighDynamics", cur_x - 20.0f, safe_z, 50.0f, safe_z, 0.0f, 1, 0, 0, 0,
                        150, 3000, 3000)) {
    return false;
  }

  // 3. Return to datum at nominal speed
  if (!testCycleMoveLeg(cfg, "V3_ReturnDatum", 50.0f, safe_z, 0.0f, 0.0f, 0.0f, 1, 0, 0, 0)) {
    return false;
  }
  ESP_LOGI(TAG, "[TEST_CYCLE] Multi-velocity dynamics PASSED");
  return true;
}

static bool testCycleThetaCapacityLegs(const GantryTestConsoleConfig *cfg, float x_mm,
                                float z_mm) {
  if (!cfg->gantry->hasThetaAxis()) {
    ESP_LOGI(TAG, "[TEST_CYCLE] skip G-I: no theta axis");
    return true;
  }
  const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
  const float test_min = (lim.theta_min < -180.0f) ? -180.0f : lim.theta_min;
  const float test_max = (lim.theta_max > 180.0f) ? 180.0f : lim.theta_max;
  const uint32_t th_v = maxThetaSpeedDegPerS();
  const uint32_t th_a = maxThetaAccelDegPerS2();
  const uint32_t th_d = maxThetaDecelDegPerS2();
  ESP_LOGI(TAG,
           "[TEST_CYCLE] theta capacity G-I at (%.1f, %.1f): test sweep "
           "%.1f .. %.1f deg (joint envelope %.1f .. %.1f) at v=%lu deg/s "
           "a=%lu d=%lu deg/s2",
           (double)x_mm, (double)z_mm, (double)test_min,
           (double)test_max, (double)lim.theta_min,
           (double)lim.theta_max, (unsigned long)th_v,
           (unsigned long)th_a, (unsigned long)th_d);
  return testCycleMoveLeg(cfg, "G", x_mm, z_mm, x_mm, z_mm, test_min, 0,
                          th_v, th_a, th_d) &&
         testCycleMoveLeg(cfg, "H", x_mm, z_mm, x_mm, z_mm, test_max, 0,
                          th_v, th_a, th_d) &&
         testCycleMoveLeg(cfg, "I", x_mm, z_mm, x_mm, z_mm, 0.0f, 0, th_v, th_a,
                          th_d);
}

void testCycleTask(void *param) {
  auto *cfg = static_cast<GantryTestConsoleConfig *>(param);
  bool pass = false;

  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "ERROR: test_cycle FAIL: gantry not initialized");
    g_testCycleInProgress = false;
    vTaskDelete(nullptr);
    return;
  }

  const int64_t start_cycle_us = esp_timer_get_time();

  ESP_LOGI(TAG, "============================================================");
  ESP_LOGI(TAG, "=== HOLISTIC GANTRY TEST CYCLE STARTING                   ===");
  ESP_LOGI(TAG, "=== Stage 1: Servo arming & Pre-flight checks             ===");
  ESP_LOGI(TAG, "=== Stage 2: EIP Bring-up (Home + Calibrate + Park)       ===");
  ESP_LOGI(TAG, "=== Stage 3: Software Safety Limits Rejection Checks      ===");
  ESP_LOGI(TAG, "=== Stage 4: Standard 2D/3D Kinematic Legs (A through F)  ===");
  ESP_LOGI(TAG, "=== Stage 5: Pick-and-Place Automation Flow with Gripper  ===");
  ESP_LOGI(TAG, "=== Stage 6: Multi-Velocity Dynamics & Creep Positioning  ===");
  ESP_LOGI(TAG, "=== Stage 7: Theta Full-Envelope Capacity Rotation (G-I) ===");
  ESP_LOGI(TAG, "=== Stage 8: Accuracy Verification & Telemetry Summary    ===");
  ESP_LOGI(TAG, "============================================================");

  do {
    // ------------------------------------------------------------------------
    // Stage 1: Servo arming & Pre-flight checks
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "[TEST_CYCLE] Stage 1: Servo arming...");
    cfg->gantry->enable();
    if (!testCycleWaitMs(cfg, 2000)) {
      ESP_LOGE(TAG, "ERROR: test_cycle FAIL: aborted during servo arm");
      break;
    }
    cfg->gantry->enable();
    if (!testCycleWaitMs(cfg, 10000)) {
      ESP_LOGE(TAG, "ERROR: test_cycle FAIL: aborted during servo arm");
      break;
    }
    if (!cfg->gantry->isEnabled()) {
      ESP_LOGE(TAG, "ERROR: test_cycle FAIL: motors not enabled");
      break;
    }
    if (cfg->gantry->isAlarmActive()) {
      ESP_LOGE(TAG, "ERROR: test_cycle FAIL: alarm after enable");
      break;
    }

    // ------------------------------------------------------------------------
    // Stage 2: EIP Bring-up (Home + Calibrate + Park)
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "[TEST_CYCLE] Stage 2: EIP bring-up (home + calibrate)");
    if (!runEipBringUpSequence(cfg)) {
      ESP_LOGE(TAG, "ERROR: test_cycle FAIL: bring-up (home/cal) failed");
      break;
    }
    if (!testCyclePoseNear(cfg, GANTRY_CAL_X_PARK_MM,
                           cfg->gantry->traverseClearanceZMm(), 5.0f,
                           cfg->gantry->hasThetaAxis(), 0.0f, 0.015f)) {
      ESP_LOGE(TAG,
               "ERROR: test_cycle FAIL: not at X park / SAFE_Z after bring-up");
      break;
    }

    // ------------------------------------------------------------------------
    // Stage 3: Software Safety Limits Rejection Checks
    // ------------------------------------------------------------------------
    if (!testCycleSafetyLimitsRejection(cfg)) {
      break;
    }

    // ------------------------------------------------------------------------
    // Stage 4: Standard 2D/3D Kinematic Legs (A through F)
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "[TEST_CYCLE] Stage 4: Standard 2D/3D Kinematic Legs (A-F)");
    const float park_x = GANTRY_CAL_X_PARK_MM;
    const float park_z = cfg->gantry->traverseClearanceZMm();
    if (!testCycleMoveLeg(cfg, "A", park_x, park_z, 220.0f, 8.0f, 0.0f, 1, 0, 0, 0) ||
        !testCycleMoveLeg(cfg, "B", 220.0f, 8.0f, 220.0f, 130.0f, 0.0f, 1, 0, 0, 0) ||
        !testCycleMoveLeg(cfg, "C", 220.0f, 130.0f, 0.0f, 0.0f, 0.0f, 1, 0, 0, 0) ||
        !testCycleMoveLeg(cfg, "D", 0.0f, 0.0f, 350.0f, 130.0f, 0.0f, 2, 0, 0, 0) ||
        !testCycleMoveLeg(cfg, "E", 350.0f, 130.0f, 80.0f, 110.0f, 0.0f, 3, 0, 0, 0) ||
        !testCycleMoveLeg(cfg, "F", 80.0f, 110.0f, 0.0f, 0.0f, 0.0f, 1, 0, 0, 0)) {
      break;
    }

    // ------------------------------------------------------------------------
    // Stage 5: Pick-and-Place Automation Flow with Gripper
    // ------------------------------------------------------------------------
    if (!testCyclePickAndPlaceSequence(cfg)) {
      break;
    }

    // ------------------------------------------------------------------------
    // Stage 6: Multi-Velocity Dynamics & Creep Positioning Validation
    // ------------------------------------------------------------------------
    if (!testCycleDynamicsStress(cfg)) {
      break;
    }

    // ------------------------------------------------------------------------
    // Stage 7: Theta Full-Envelope Capacity Rotation (G-I)
    // ------------------------------------------------------------------------
    ESP_LOGI(TAG, "[TEST_CYCLE] Stage 7: Theta Full-Envelope Capacity Rotation (G-I)");
    if (!testCycleThetaCapacityLegs(cfg, 0.0f, 0.0f)) {
      break;
    }

    // ------------------------------------------------------------------------
    // Stage 8: Accuracy Verification & Telemetry Summary
    // ------------------------------------------------------------------------
    const int64_t elapsed_cycle_ms = (esp_timer_get_time() - start_cycle_us) / 1000;
    const Gantry::JointConfig final_pose = cfg->gantry->getCurrentJointConfig();
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, "=== HOLISTIC TEST CYCLE COMPLETE: PASS                   ===");
    ESP_LOGI(TAG, "=== Total Cycle Duration: %lld ms", (long long)elapsed_cycle_ms);
    ESP_LOGI(TAG, "=== Final Pose: x=%.3f mm, z=%.3f mm, theta=%.3f deg",
             (double)final_pose.x, (double)final_pose.z, (double)final_pose.theta);
    ESP_LOGI(TAG, "=== Class 1 Bus Timing Status: ACTIVE");
    eip::dumpClass1TimingStats();
    ESP_LOGI(TAG, "============================================================");

    pass = true;
    ESP_LOGI(TAG, "OK test_cycle PASS");
  } while (false);

  if (!pass) {
    if (cfg->gantry->isAbortRequested()) {
      ESP_LOGE(TAG, "ERROR: test_cycle FAIL: aborted");
    } else {
      ESP_LOGE(TAG, "ERROR: test_cycle FAIL");
    }
  }
  g_testCycleInProgress = false;
  vTaskDelete(nullptr);
}

}  // namespace GantryConsoleImpl
