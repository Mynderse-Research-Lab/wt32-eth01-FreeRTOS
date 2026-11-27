/**
 * @file test_gantry_orchestration_fault.cpp
 * @brief Link, grip and retract host tests.
 */

#include "test_gantry_orchestration_internal.h"

using namespace gantry_orch_test;

// Any one dropped drive link (2026-10-20: all three during a Z retract) makes
// the gantry not online; it is online again once every link is back.
void test_drives_online_requires_every_link(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(h.g->drivesOnline());
    h.x->online = false;
    TEST_ASSERT_FALSE(h.g->drivesOnline());
    h.x->online = true;
    h.z->online = false;
    TEST_ASSERT_FALSE(h.g->drivesOnline());
    h.z->online = true;
    h.t->online = false;
    TEST_ASSERT_FALSE(h.g->drivesOnline());
    h.t->online = true;
    TEST_ASSERT_TRUE(h.g->drivesOnline());
}

// After an E602 trip and alarmreset the X servo stays off (2026-10-20 15:35):
// the first axis without its servo on is reported so picks hold.
void test_inactive_servo_axis_reports_first_off_servo(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_NULL(h.g->inactiveServoAxis());
    h.x->servo_active = false;
    TEST_ASSERT_EQUAL_STRING("X", h.g->inactiveServoAxis());
    h.x->servo_active = true;
    h.t->servo_active = false;
    TEST_ASSERT_EQUAL_STRING("theta", h.g->inactiveServoAxis());
    h.z->servo_active = false;
    TEST_ASSERT_EQUAL_STRING("Z", h.g->inactiveServoAxis());
    h.z->servo_active = true;
    h.t->servo_active = true;
    TEST_ASSERT_NULL(h.g->inactiveServoAxis());
}

void test_theta_settled_at_requires_idle_no_alarm_and_position(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(h.g->thetaSettledAt(0.0f));
    TEST_ASSERT_TRUE(h.g->thetaSettledAt(0.05f));
    TEST_ASSERT_FALSE(h.g->thetaSettledAt(1.0f));
    h.t->alarm = true;
    TEST_ASSERT_FALSE(h.g->thetaSettledAt(0.0f));
    h.t->alarm = false;
    h.t->busy = true;
    TEST_ASSERT_FALSE(h.g->thetaSettledAt(0.0f));
}

#if CONFIG_GANTRY_THETA_SEQUENTIAL

void test_sequential_theta_after_in_band_path(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 40.0f, 0.0f, 15.0f, 50));
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    completeLinear(h);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
}

void test_sequential_theta_gated_when_path_ends_deep(void) {
    Harness h = makeHarness(0.0f, 10.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 80.0f, 15.0f, 50));
    completeLinear(h);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
}

#endif

void test_z_cap_on_z_only_segment(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 100.0f, 0.0f, 2000));
    TEST_ASSERT_TRUE(h.z->busy);
    const float cap = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, cap < 2000.0f ? cap : 2000.0f, h.z->moves.back().speed_mm_s);
}

void test_z_cap_on_combined_segment_keeps_it_straight(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 30.0f, 0.0f, 5000));
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_TRUE(h.z->busy);
    const float cap = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, cap, h.z->moves.back().speed_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, cap * 100.0f / 30.0f, h.x->moves.back().speed_mm_s);
}

void test_retract_to_safe_z_while_theta_alarmed(void) {
    Harness h = makeHarness(50.0f, 100.0f, 10.0f);
    h.t->alarm = true;
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->retractToSafeZ(2000, 3000, 3000));
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 30.0f, h.z->target_mm);
    const float cap = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, cap < 2000.0f ? cap : 2000.0f, h.z->moves.back().speed_mm_s);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.z->busy);
    h.z->completeMove();
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
    TEST_ASSERT_EQUAL_UINT(0u, h.x->moves.size());
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());

    // Already in the band: nothing to do.
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->retractToSafeZ(2000, 3000, 3000));
    TEST_ASSERT_FALSE(h.g->isBusy());

    h.z->mm = h.z->target_mm = 90.0f;
    h.z->alarm = true;
    TEST_ASSERT_EQUAL(GantryError::TIMEOUT, h.g->retractToSafeZ(2000, 3000, 3000));
}

void test_retract_stops_a_move_in_progress(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 100.0f, 0.0f, 2000));
    h.z->setMm(80.0f);
    h.x->alarm = true;
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->retractToSafeZ(2000, 3000, 3000));
    TEST_ASSERT_TRUE(h.z->stop_count > 0);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 30.0f, h.z->target_mm);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.z->busy);
}

void test_grip_reports_write_failure(void) {
    Harness h = makeHarness();
    TEST_ASSERT_FALSE(h.g->grip(true));
    h.g->setEndEffectorPin(3, true);
    TEST_ASSERT_TRUE(h.g->grip(true));
    TEST_ASSERT_TRUE(h.g->isGripperActive());
    HostTest::gpio_expander_write_result = ESP_FAIL;
    TEST_ASSERT_FALSE(h.g->grip(false));
    TEST_ASSERT_TRUE(h.g->isGripperActive());
    HostTest::gpio_expander_write_result = ESP_OK;
    TEST_ASSERT_TRUE(h.g->grip(false));
    TEST_ASSERT_FALSE(h.g->isGripperActive());
}

// A latched-up MCP23S17 reads every register as 0x00 while SPI writes still
// "succeed" (2026-10-20): the latch read-back must turn that into a failure.
void test_grip_fails_when_latch_does_not_confirm(void) {
    Harness h = makeHarness();
    h.g->setEndEffectorPin(3, true);
    HostTest::gpio_expander_latch_override = 0;
    TEST_ASSERT_FALSE(h.g->grip(true));
    TEST_ASSERT_FALSE(h.g->isGripperActive());
    TEST_ASSERT_TRUE(h.g->grip(false));  // latch 0 matches an open command
    HostTest::gpio_expander_latch_override = -1;
    TEST_ASSERT_TRUE(h.g->grip(true));
    TEST_ASSERT_TRUE(h.g->isGripperActive());
    TEST_ASSERT_TRUE(h.g->grip(false));
}

void test_cross_axis_fault_stops_orchestrator(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 10.0f, 0.0f));
    tick(*h.g);
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_TRUE(h.z->busy);
    
    // Simulate Z axis faulting mid-move (e.g., EIP drop or limit hit).
    h.z->alarm = true;
    h.z->stopMotion();
    
    // Run the orchestrator. It should notice Z is idle but off-target,
    // and invoke failPath(), stopping X.
    tick(*h.g);
    
    TEST_ASSERT_GREATER_THAN(0, h.x->stop_count);
    TEST_ASSERT_FALSE(h.g->isBusy());
}

void test_z_accel_cap_preserves_full_x_accel(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    Gantry::RuntimeParams rp;
    rp.z_speed_cap_mm_s = 900.0f;
    rp.z_accel_cap_mm_s2 = 5000.0f;
    rp.z_decel_cap_mm_s2 = 5500.0f;
    h.g->setRuntimeParams(rp);

    // Pure X move uses full path speed and acceleration without Z throttling.
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 200.0f, 0.0f, 0.0f, 3000, 18000));
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 3000.0f, h.x->moves.back().speed_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 18000.0f, h.x->moves.back().accel_mm_s2);
    completeLinear(h);
    tick(*h.g);

    // Pure Z plunge caps speed and accel/decel independently of X's 18000.
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 200.0f, 100.0f, 0.0f, 3000, 18000));
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 900.0f, h.z->moves.back().speed_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5000.0f, h.z->moves.back().accel_mm_s2);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5500.0f, h.z->moves.back().decel_mm_s2);
    completeLinear(h);
    tick(*h.g);

    // Retract also honors the Z accel/decel cap even when caller requests 18000.
    h.z->mm = h.z->target_mm = 100.0f;
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->retractToSafeZ(900, 18000, 18000));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5000.0f, h.z->moves.back().accel_mm_s2);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5500.0f, h.z->moves.back().decel_mm_s2);
}

void test_retract_to_safe_z_noop_inside_traverse_band(void) {
    Harness h = makeHarness(382.5f, 30.0f, 0.0f);
    // Kinetix settles inside 0.5 mm arrival band (30.08 mm <= 30.0 + 0.5 mm).
    h.z->mm = h.z->target_mm = 30.08f;
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->retractToSafeZ(900, 5000, 5000));
    TEST_ASSERT_FALSE(h.g->isBusy());
    TEST_ASSERT_EQUAL_UINT(0u, h.z->moves.size());
}

void test_chain_move_after_retract_refused_while_preload_held(void) {
    Harness h = makeHarness(382.5f, 30.0f, -45.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->preloadJointMove(JointConfig(382.5f, 119.0f, -45.0f), 900, 5000, 5000));
    TEST_ASSERT_TRUE(h.g->releasePreloadedMotion());
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->queueRetractToSafeZ(900, 5000, 5000));
    TEST_ASSERT_TRUE(h.g->isPreloadHeld());

    // Must refuse while the retract StartMotion edge is still held (plunge in flight).
    TEST_ASSERT_FALSE(h.g->chainMoveAfterRetract(JointConfig(10.0f, 90.8f, 90.0f),
                                                 1330, 1000, 8000, 8000, 3600, 3600, 0.0f, true));
    TEST_ASSERT_EQUAL_UINT(0u, h.x->moves.size());
}

void test_chain_move_after_retract_arms_x_and_theta_on_band_entry_then_descends(void) {
    Harness h = makeHarness(382.5f, 30.0f, -45.0f);
    Gantry::RuntimeParams rp;
    rp.z_speed_cap_mm_s = 900.0f;
    rp.z_accel_cap_mm_s2 = 5000.0f;
    rp.z_decel_cap_mm_s2 = 5000.0f;
    rp.theta_max_speed_deg_s = 1000.0f;
    h.g->setRuntimeParams(rp);

    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->preloadJointMove(JointConfig(382.5f, 119.0f, -45.0f), 900, 5000, 5000));
    TEST_ASSERT_TRUE(h.g->releasePreloadedMotion());
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->queueRetractToSafeZ(900, 5000, 5000));

    // Plunge reaches depth and retract edge fires.
    h.z->setMm(118.0f);
    TEST_ASSERT_TRUE(h.g->releasePreloadedMotion());
    TEST_ASSERT_FALSE(h.g->isPreloadHeld());

    TEST_ASSERT_TRUE(h.g->chainMoveAfterRetract(JointConfig(10.0f, 90.8f, 90.0f),
                                                1330, 1000, 8000, 8000, 3600, 3600, 0.0f, true));
    TEST_ASSERT_FALSE(h.g->isRetractHandedOff());
    // While Z is still deep below SAFE_Z (118 mm > 30 + 48 = 78 mm), X and theta stay locked.
    tick(*h.g);
    TEST_ASSERT_FALSE(h.x->busy);
    TEST_ASSERT_EQUAL_UINT(0u, h.x->moves.size());
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());

    // Above the 160 ms retract lead window (80 mm > 30 + 48 = 78 mm), both X and theta wait.
    h.z->setMm(80.0f);
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isRetractHandedOff());
    TEST_ASSERT_FALSE(h.x->busy);
    TEST_ASSERT_EQUAL_UINT(0u, h.x->moves.size());
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());

    // Entering the 160 ms retract lead window (<= 78 mm) arms X and full-speed theta together
    // well before Z completes so X is already travelling during the top of the Z retract.
    h.z->setMm(76.0f);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.g->isRetractHandedOff());
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_EQUAL_UINT(1u, h.x->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, h.x->target_mm);
#if !CONFIG_GANTRY_THETA_SEQUENTIAL
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, h.t->moves[0].speed_deg_s);
#endif

    // Complete Z retract, X traverse, and theta -> seg 2 Z descent to 90.8 mm starts immediately.
    const size_t z_before_desc = h.z->moves.size();
    h.z->completeMove();
    h.x->completeMove();
    if (h.t->busy) {
        h.t->completeMove();
    }
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(z_before_desc + 1u, h.z->moves.size());
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 90.8f, h.z->target_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 900.0f, h.z->moves.back().speed_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5000.0f, h.z->moves.back().accel_mm_s2);
    h.z->completeMove();
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
}

#if !CONFIG_GANTRY_THETA_SEQUENTIAL
void test_theta_full_speed_skips_window_throttling(void) {
    Harness h = makeHarness(10.0f, 30.0f, 90.0f);
    Gantry::RuntimeParams rp;
    rp.theta_max_speed_deg_s = 1000.0f;
    h.g->setRuntimeParams(rp);

    // 372.5 mm at 1330 mm/s would throttle 90 deg to ~428 deg/s in window mode;
    // theta_full_speed=true must run at the full 1000 deg/s cap immediately.
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(382.5f, 30.0f, 0.0f), 1330, 1000, 8000, 8000,
                                  3600, 3600, 0.0f, true));
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, h.t->moves[0].speed_deg_s);
}
#endif

void test_bin_retract_pick_move_arms_x_before_z_completes(void) {
    Harness h = makeHarness(10.0f, 90.8f, 90.0f);
    Gantry::RuntimeParams rp;
    rp.z_speed_cap_mm_s = 900.0f;
    rp.z_accel_cap_mm_s2 = 5000.0f;
    rp.z_decel_cap_mm_s2 = 5000.0f;
    rp.theta_max_speed_deg_s = 1000.0f;
    h.g->setRuntimeParams(rp);
    h.g->setSafeZHeight(48.4f);

    // Pick/staging move from bin (10, 90.8, 90) -> staging (382.5, 48.4, 0).
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(382.5f, 48.4f, 0.0f), 1330, 1000, 8000, 8000,
                                  3600, 3600, 0.0f, true));
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FALSE(h.x->busy);

    // Above lead threshold (83 mm > 48.4 + 0.78*42.4 = 81.47 mm): X still waiting.
    h.z->setMm(83.0f);
    tick(*h.g);
    TEST_ASSERT_FALSE(h.x->busy);

    // Inside X lead window (80.0 mm <= 81.47 mm): X launches while Z is still retracting.
    h.z->setMm(80.0f);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 382.5f, h.x->target_mm);
}
