/**
 * @file test_gantry_orchestration_move.cpp
 * @brief Move, theta window and budget host tests.
 */

#include "test_gantry_orchestration_internal.h"

using namespace gantry_orch_test;

void test_move_not_initialized(void) {
    auto x = std::make_unique<FakeLinearAxis>();
    auto z = std::make_unique<FakeLinearAxis>();
    auto t = std::make_unique<FakeRotaryAxis>();
    Gantry::Gantry g(std::move(x), std::move(z), std::move(t), -1);
    TEST_ASSERT_EQUAL(GantryError::NOT_INITIALIZED,
                      g.moveTo(JointConfig(1, 0, 0)));
}

void test_move_not_enabled(void) {
    HostTest::resetClock();
    auto xu = std::make_unique<FakeLinearAxis>();
    auto zu = std::make_unique<FakeLinearAxis>();
    auto tu = std::make_unique<FakeRotaryAxis>();
    Gantry::Gantry g(std::move(xu), std::move(zu), std::move(tu), -1);
    g.configureDriveManagedLimits();
    TEST_ASSERT_TRUE(g.begin());
    TEST_ASSERT_EQUAL(GantryError::MOTOR_NOT_ENABLED,
                      g.moveTo(JointConfig(1, 0, 0)));
}

void test_move_not_calibrated(void) {
    Harness h = makeHarness();
    h.g->setWorkspaceCalibrated(false);
    TEST_ASSERT_EQUAL(GantryError::NOT_CALIBRATED, go(h, 10.0f, 0.0f, 0.0f));
}

void test_workspace_cal_clears_on_drive_lost(void) {
    Harness h = makeHarness();
    TEST_ASSERT_TRUE(h.g->isWorkspaceCalibrated());

    h.x->drive_ref = DrivePositionRef::kUnknown;
    tick(*h.g);
    TEST_ASSERT_TRUE(h.g->isWorkspaceCalibrated());

    h.x->drive_ref = DrivePositionRef::kLost;
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isWorkspaceCalibrated());
    TEST_ASSERT_EQUAL(GantryError::NOT_CALIBRATED, go(h, 10.0f, 0.0f, 0.0f));

    h.x->drive_ref = DrivePositionRef::kHomed;
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isWorkspaceCalibrated());

    h.g->setWorkspaceCalibrated(true);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 10.0f, 0.0f, 0.0f));
}

void test_workspace_cal_clears_on_theta_lost(void) {
    Harness h = makeHarness();
    h.t->drive_ref = DrivePositionRef::kLost;
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isWorkspaceCalibrated());
}

// G-A9: with drive-managed limits, any fault with the stopped bit sets both
// soft switches. A fault-stop while X moves toward min must stop X but leave
// its position alone; only the EIP home sequence sets the datum.
void test_drive_fault_stop_does_not_rezero_x(void) {
    Harness h = makeHarness(200.0f, 0.0f, 0.0f);
    TEST_ASSERT_NOT_NULL(h.x->min_sw);
    TEST_ASSERT_NOT_NULL(h.x->max_sw);
    h.x->busy = true;
    tick(*h.g);  // records the current X count
    h.x->mm = 199.5f;  // moving toward min
    h.x->min_sw->setExternalActive(true);
    h.x->max_sw->setExternalActive(true);
    const int stops = h.x->stop_count;
    tick(*h.g);
    TEST_ASSERT_GREATER_THAN_INT(stops, h.x->stop_count);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 199.5f, h.x->mm);
}

void test_move_invalid_position(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::INVALID_POSITION,
                      go(h, 999.0f, 0.0f, 0.0f));
}

void test_orchestrator_bubbles_up_axis_rejection(void) {
    Harness h = makeHarness();
    h.x->reject_move = true;
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.0f));
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
}

void test_move_already_moving(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(h.g->isBusy());
    TEST_ASSERT_EQUAL(GantryError::ALREADY_MOVING, go(h, 20.0f, 0.0f, 0.0f));
}

void test_theta_refused_no_in_band_segment(void) {
    Harness h = makeHarness(0.0f, 100.0f, 0.0f);
    TEST_ASSERT_FALSE(h.g->zInTraverseBand());
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 80.0f, 45.0f));
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    completeLinear(h);
    tick(*h.g);
    completeLinear(h);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
}

void test_theta_only_while_deep_refused(void) {
    Harness h = makeHarness(0.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 100.0f, 45.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

void test_theta_only_to_joint_limits_in_band(void) {
    // test_cycle G/H: theta-only at an in-band pose to the firmware envelope.
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(h.g->zInTraverseBand());
    TEST_ASSERT_EQUAL(
        GantryError::OK,
        h.g->moveTo(JointConfig(0.0f, 0.0f, 180.0f), 50, 3600, 3000, 3000,
                    18000, 18000));
    if (h.t->moves.empty()) {
        tick(*h.g);
    }
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, h.t->moves[0].target_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 3600.0f, h.t->moves[0].speed_deg_s);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 18000.0f, h.t->moves[0].accel_deg_s2);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 18000.0f, h.t->moves[0].decel_deg_s2);

    h.t->completeMove();
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
    TEST_ASSERT_EQUAL(
        GantryError::OK,
        h.g->moveTo(JointConfig(0.0f, 0.0f, -180.0f), 50, 3600, 3000, 3000,
                    18000, 18000));
    if (h.t->moves.size() < 2u) {
        tick(*h.g);
    }
    TEST_ASSERT_EQUAL_UINT(2u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -180.0f, h.t->moves[1].target_deg);
}

void test_theta_only_return_to_zero_stays_busy(void) {
    // test_cycle I: in-band theta-only 179.5 → 0 must stay busy until complete.
    Harness h = makeHarness(0.0f, 0.0f, 179.5f);
    TEST_ASSERT_EQUAL(
        GantryError::OK,
        h.g->moveTo(JointConfig(0.0f, 0.0f, 0.0f), 50, 3600, 3000, 3000,
                    18000, 18000));
    if (h.t->moves.empty()) {
        tick(*h.g);
    }
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, h.t->moves[0].target_deg);
    TEST_ASSERT_TRUE(h.g->isBusy());
    tick(*h.g);
    TEST_ASSERT_TRUE(h.g->isBusy());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 179.5f, h.t->deg);
    h.t->completeMove();
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
}

#if !CONFIG_GANTRY_THETA_SEQUENTIAL

void test_theta_does_not_start_before_25_percent(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 90.0f, 50));
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    h.x->setMm(20.0f);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
}

void test_theta_starts_at_25_percent_window_speed(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 90.0f, 50));
    h.x->setMm(25.0f);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    // T_seg = 100/50 = 2 s; window 0.5*T = 1 s; 90 deg/s.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, h.t->moves[0].speed_deg_s);
}

void test_descent_held_until_theta_idle(void) {
    Harness h = makeHarness(0.0f, 30.0f, 0.0f);
    // In-band X of 1 mm then Z descent to 100. Short T_seg clamps theta.
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 1.0f, 100.0f, 180.0f, 50));
    TEST_ASSERT_TRUE(h.x->busy);
    h.x->setMm(0.5f);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.t->moves.size() >= 1u);
    const size_t z_moves_during_window = h.z->moves.size();
    h.x->completeMove();
    tick(*h.g);
    // Descent must not arm while theta is still busy.
    TEST_ASSERT_TRUE(h.t->busy);
    TEST_ASSERT_EQUAL_UINT(z_moves_during_window, h.z->moves.size());
    h.t->completeMove();
    tick(*h.g);
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, h.z->target_mm);
}

void test_stop_clears_pending_theta(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 90.0f, 50));
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    h.g->requestAbort();
    h.x->setMm(50.0f);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

void test_retract_starts_theta_after_band_entry(void) {
    Harness h = makeHarness(50.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 0.0f, 20.0f, 50));
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FALSE(h.x->busy);  // X deferred
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    h.z->setMm(30.0f);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_TRUE(h.t->moves.size() >= 1u);
}

void test_theta_start_fraction_zero_starts_with_x(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(100.0f, 0.0f, 90.0f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    TEST_ASSERT_TRUE(h.x->busy);
    tick(*h.g);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, h.x->mm);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    // T_seg = 2 s; window 0..75% = 1.5 s; 60 deg/s.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, h.t->moves[0].speed_deg_s);
}

void test_theta_start_fraction_caps_at_move_theta_speed(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(1.0f, 0.0f, 180.0f), 50, 100, 3000, 3000,
                                  1800, 1800, 0.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, h.t->moves[0].speed_deg_s);
}

void test_pick_theta_commands_a_tenth_while_waiting(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 0.0f, 0.2f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.2f, h.t->moves[0].target_deg);
    TEST_ASSERT_TRUE(h.g->isBusy());
}

void test_pick_theta_truncates_below_a_tenth(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 0.0f, 0.05f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

void test_pick_theta_quantizes_to_a_tenth(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 0.0f, 10.14f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.1f, h.t->moves[0].target_deg);
}

void test_pick_theta_runs_with_x_below_half_degree(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(100.0f, 0.0f, 0.2f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.2f, h.t->moves[0].target_deg);
}

void test_console_still_ignores_a_fifth_of_a_degree(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.2f, 50));
    h.x->completeMove();
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
}

void test_theta_refused_below_band_during_wait(void) {
    Harness h = makeHarness(0.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 100.0f, 45.0f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

void test_budget_matches_timed_fake_pre_position(void) {
    Harness h = makeHarness(0.0f, 30.0f, 0.0f);
    h.x->timed.enabled = true;
    h.z->timed.enabled = true;
    h.t->timed.enabled = true;
    PickIntercept::MotionLimits m;
    m.speed = 2000.0f;
    m.accel = 3000.0f;
    m.decel = 3000.0f;
    m.theta_cap = 360.0f;
    m.theta_accel = 1800.0f;
    m.theta_decel = 1800.0f;
    m.z_cap = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
    m.theta_start_frac = 0.0f;
    const int64_t start = HostTest::nowUs();
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(200.0f, 30.0f, 90.0f), 2000, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    for (int i = 0; i < 5000 && h.g->isBusy(); ++i) {
        tick(*h.g, 1);
    }
    TEST_ASSERT_FALSE(h.g->isBusy());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, h.x->mm);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, h.t->deg);
    const float elapsed = static_cast<float>(HostTest::nowUs() - start) * 1.0e-6f;
    const auto w = Gantry::Path::planThetaWindow(200.0f, m.speed, 90.0f, m.theta_cap, 0.0f);
    const float t_x = PickIntercept::profileSeconds(200.0f, m.speed, m.accel, m.decel);
    const float t_th = PickIntercept::profileSeconds(90.0f, w.speed_deg_per_s, m.theta_accel,
                                                     m.theta_decel);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, t_x > t_th ? t_x : t_th, elapsed);

    // The budget adds what the fakes do not model: preload, settle, rotary start.
    const auto budget =
        PickIntercept::budgetPick(0.0f, 30.0f, 0.0f, 200.0f, 30.0f, 30.0f, 100.0f, 90.0f, m);
    TEST_ASSERT_TRUE(budget.approach_s >= elapsed - 0.005f);
    TEST_ASSERT_TRUE(budget.approach_s <= elapsed + PickIntercept::kPreloadS +
                                              PickIntercept::kSettleS + PickIntercept::kTickS +
                                              PickIntercept::kThetaStartS + 0.005f);
}

#endif  // !SEQUENTIAL
