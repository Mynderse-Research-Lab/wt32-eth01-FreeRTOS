/**
 * @file test_gantry_orchestration.cpp
 * @brief Host tests for Gantry path SM, SAFE_Z interlocks, and theta window.
 */

#include "unity.h"

#ifndef CONFIG_GANTRY_THETA_SEQUENTIAL
#define CONFIG_GANTRY_THETA_SEQUENTIAL 0
#endif

#include "FakeAxes.h"
#include "Gantry.h"
#include "GantryPathProfile.h"
#include "axis_drivetrain_params.h"
#include "gpio_expander.h"
#include "host_test_clock.h"
#include "pick_intercept.h"

#include <memory>

using Gantry::DrivePositionRef;
using Gantry::EndEffectorPose;
using Gantry::GantryError;
using Gantry::JointConfig;
using Gantry::Test::FakeLinearAxis;
using Gantry::Test::FakeRotaryAxis;

namespace {

struct Harness {
    FakeLinearAxis* x = nullptr;
    FakeLinearAxis* z = nullptr;
    FakeRotaryAxis* t = nullptr;
    std::unique_ptr<Gantry::Gantry> g;
};

Harness makeHarness(float x_mm = 0.0f, float z_mm = 0.0f, float th_deg = 0.0f) {
    HostTest::resetClock();
    auto xu = std::make_unique<FakeLinearAxis>();
    auto zu = std::make_unique<FakeLinearAxis>();
    auto tu = std::make_unique<FakeRotaryAxis>();
    Harness h;
    h.x = xu.get();
    h.z = zu.get();
    h.t = tu.get();
    h.x->mm = h.x->target_mm = x_mm;
    h.z->mm = h.z->target_mm = z_mm;
    h.t->deg = h.t->target_deg = th_deg;
    h.g = std::make_unique<Gantry::Gantry>(std::move(xu), std::move(zu), std::move(tu), -1);
    h.g->configureDriveManagedLimits();
    TEST_ASSERT_TRUE(h.g->begin());
    h.g->enable();
    TEST_ASSERT_TRUE(h.g->isEnabled());
    h.g->setJointLimits(0.0f, 550.0f, 0.0f, 150.0f, -180.0f, 180.0f);
    h.g->setSafeZHeight(30.0f);
    h.g->setWorkspaceCalibrated(true);
    return h;
}

void tick(Gantry::Gantry& g, uint32_t ms = 10) {
    g.update();
    HostTest::advanceMs(ms);
}

void completeLinear(Harness& h) {
    if (h.x->busy) {
        h.x->completeMove();
    }
    if (h.z->busy) {
        h.z->completeMove();
    }
}

GantryError go(Harness& h, float x, float z, float th, uint32_t speed = 50,
               uint32_t accel = 3000) {
    return h.g->moveTo(JointConfig(x, z, th), speed, 30, accel, accel, 0, 0);
}

}  // namespace

void setUp(void) { HostTest::resetClock(); }
void tearDown(void) {}

static void test_move_not_initialized(void) {
    auto x = std::make_unique<FakeLinearAxis>();
    auto z = std::make_unique<FakeLinearAxis>();
    auto t = std::make_unique<FakeRotaryAxis>();
    Gantry::Gantry g(std::move(x), std::move(z), std::move(t), -1);
    TEST_ASSERT_EQUAL(GantryError::NOT_INITIALIZED,
                      g.moveTo(JointConfig(1, 0, 0)));
}

static void test_move_not_enabled(void) {
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

static void test_move_not_calibrated(void) {
    Harness h = makeHarness();
    h.g->setWorkspaceCalibrated(false);
    TEST_ASSERT_EQUAL(GantryError::NOT_CALIBRATED, go(h, 10.0f, 0.0f, 0.0f));
}

static void test_workspace_cal_clears_on_drive_lost(void) {
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

static void test_workspace_cal_clears_on_theta_lost(void) {
    Harness h = makeHarness();
    h.t->drive_ref = DrivePositionRef::kLost;
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isWorkspaceCalibrated());
}

// G-A9: with drive-managed limits, any fault with the stopped bit sets both
// soft switches. A fault-stop while X moves toward min must stop X but leave
// its position alone; only the EIP home sequence sets the datum.
static void test_drive_fault_stop_does_not_rezero_x(void) {
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

static void test_move_invalid_position(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::INVALID_POSITION,
                      go(h, 999.0f, 0.0f, 0.0f));
}

static void test_orchestrator_bubbles_up_axis_rejection(void) {
    Harness h = makeHarness();
    h.x->reject_move = true;
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.0f));
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
}

static void test_move_already_moving(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(h.g->isBusy());
    TEST_ASSERT_EQUAL(GantryError::ALREADY_MOVING, go(h, 20.0f, 0.0f, 0.0f));
}

static void test_theta_refused_no_in_band_segment(void) {
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

static void test_theta_only_while_deep_refused(void) {
    Harness h = makeHarness(0.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 100.0f, 45.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

static void test_theta_only_to_joint_limits_in_band(void) {
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

static void test_theta_only_return_to_zero_stays_busy(void) {
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

static void test_theta_does_not_start_before_25_percent(void) {
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

static void test_theta_starts_at_25_percent_window_speed(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 90.0f, 50));
    h.x->setMm(25.0f);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    // T_seg = 100/50 = 2 s; window 0.5*T = 1 s; 90 deg/s.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, h.t->moves[0].speed_deg_s);
}

static void test_descent_held_until_theta_idle(void) {
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

static void test_stop_clears_pending_theta(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 90.0f, 50));
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    h.g->requestAbort();
    h.x->setMm(50.0f);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

static void test_retract_starts_theta_after_band_entry(void) {
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

static void test_theta_start_fraction_zero_starts_with_x(void) {
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

static void test_theta_start_fraction_caps_at_move_theta_speed(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(1.0f, 0.0f, 180.0f), 50, 100, 3000, 3000,
                                  1800, 1800, 0.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, h.t->moves[0].speed_deg_s);
}

static void test_pick_theta_commands_a_tenth_while_waiting(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 0.0f, 0.2f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.2f, h.t->moves[0].target_deg);
    TEST_ASSERT_TRUE(h.g->isBusy());
}

static void test_pick_theta_truncates_below_a_tenth(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 0.0f, 0.05f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

static void test_pick_theta_quantizes_to_a_tenth(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 0.0f, 10.14f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.1f, h.t->moves[0].target_deg);
}

static void test_pick_theta_runs_with_x_below_half_degree(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(100.0f, 0.0f, 0.2f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.2f, h.t->moves[0].target_deg);
}

static void test_console_still_ignores_a_fifth_of_a_degree(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.2f, 50));
    h.x->completeMove();
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
}

static void test_theta_refused_below_band_during_wait(void) {
    Harness h = makeHarness(0.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK,
                      h.g->moveTo(JointConfig(0.0f, 100.0f, 45.0f), 50, 360, 3000, 3000,
                                  1800, 1800, 0.0f));
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    TEST_ASSERT_FALSE(h.g->isBusy());
}

static void test_budget_matches_timed_fake_pre_position(void) {
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

static void test_fail_path_off_target(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.0f));
    h.x->setMm(10.0f);
    h.x->busy = false;
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
    TEST_ASSERT_TRUE(h.x->stop_count >= 1);
}

static void test_deferred_x_arms_once_in_band(void) {
    Harness h = makeHarness(0.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 80.0f, 10.0f, 0.0f));
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FALSE(h.x->busy);
    const size_t x0 = h.x->moves.size();
    h.z->setMm(30.0f);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_EQUAL_UINT(x0 + 1u, h.x->moves.size());
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(x0 + 1u, h.x->moves.size());
}

static void test_in_band_xz_together(void) {
    Harness h = makeHarness(0.0f, 10.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 40.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_TRUE(h.z->busy);
}

static void test_three_segment_outbound(void) {
    Harness h = makeHarness(10.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 200.0f, 120.0f, 0.0f));
    // Seg1: Z to ceiling.
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FALSE(h.x->busy);
    h.z->completeMove();
    tick(*h.g);
    // Seg2: X at ceiling.
    TEST_ASSERT_TRUE(h.x->busy);
    h.x->completeMove();
    tick(*h.g);
    // Seg3: Z descent.
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 120.0f, h.z->target_mm);
    h.z->completeMove();
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
}

static void test_above_to_above_arms_x_at_band_entry(void) {
    Harness h = makeHarness(10.0f, 100.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 200.0f, 120.0f, 0.0f));
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FALSE(h.x->busy);

    // Crossing into SAFE_Z should arm X immediately; no extra dwell segment.
    h.z->setMm(30.0f);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_TRUE(h.x->busy);
}

static void test_pnp_gripper_then_retract(void) {
    Harness h = makeHarness();
    EndEffectorPose pose = h.g->forwardKinematics(JointConfig(0.0f, 80.0f, 90.0f));
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->moveTo(pose, 50, 30, 3000, 3000));
    h.z->completeMove();
    tick(*h.g);
    // Gripper close is 190 ms, then retract to the SAFE_Z ceiling.
    HostTest::advanceMs(200);
    tick(*h.g);
    TEST_ASSERT_TRUE(h.z->busy);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, h.z->target_mm);
}

static void test_bringup_requires_drive_managed(void) {
    auto xu = std::make_unique<FakeLinearAxis>();
    auto zu = std::make_unique<FakeLinearAxis>();
    auto tu = std::make_unique<FakeRotaryAxis>();
    Gantry::Gantry g(std::move(xu), std::move(zu), std::move(tu), -1);
    TEST_ASSERT_TRUE(g.begin());
    g.enable();
    TEST_ASSERT_FALSE(g.startEipBringUp());
}

static void test_bringup_z_minus_then_timeout(void) {
    Harness h = makeHarness(35.0f, 30.0f, 0.0f);
    TEST_ASSERT_TRUE(h.g->startEipBringUp());
    TEST_ASSERT_TRUE(h.g->eipBringUpInProgress());
    tick(*h.g);
    TEST_ASSERT_TRUE(h.z->busy);
    HostTest::advanceMs(90001);
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->eipBringUpInProgress());
}

static void test_bringup_phase_order_to_x_home(void) {
    Harness h = makeHarness(35.0f, 30.0f, 0.0f);
    TEST_ASSERT_TRUE(h.g->startEipBringUp());
    tick(*h.g);  // Z- seek starts
    TEST_ASSERT_TRUE(h.z->busy);
    h.z->a015 = true;
    tick(*h.g);  // trip -> creep
    TEST_ASSERT_TRUE(h.z->busy);
    h.z->a015 = false;
    h.z->completeMove();
    tick(*h.g);  // creep cleared -> settle
    tick(*h.g);  // settle -> X home seek
    tick(*h.g);  // X home seek starts
    TEST_ASSERT_TRUE(h.x->busy);
}

static void test_bringup_full_sequence_with_theta_90_orient(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(h.g->startEipBringUp());
    // 1. Z- seek & creep
    tick(*h.g);
    h.z->a015 = true;
    tick(*h.g);
    h.z->a015 = false;
    h.z->completeMove();
    tick(*h.g);  // creep cleared -> settle
    tick(*h.g);  // settle -> X home seek
    tick(*h.g);  // X home seek starts

    // 2. X home seek & creep
    TEST_ASSERT_TRUE(h.x->busy);
    h.x->a014 = true;
    tick(*h.g);
    h.x->a014 = false;
    h.x->completeMove();
    tick(*h.g);  // creep cleared -> settle
    tick(*h.g);  // settle -> X cal seek
    tick(*h.g);  // X cal seek starts

    // 3. X cal seek & creep
    TEST_ASSERT_TRUE(h.x->busy);
    h.x->a015 = true;
    tick(*h.g);
    h.x->a015 = false;
    h.x->completeMove();
    tick(*h.g);  // creep cleared -> settle
    tick(*h.g);  // settle -> parks X at 35mm

    // 4. X park move
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 35.0f, h.x->target_mm);
    h.x->completeMove();
    tick(*h.g);  // X park completes -> Theta orient starts

    // 5. Theta orient to 90 degrees before Z+ seek
    TEST_ASSERT_TRUE(h.t->busy);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 90.0f, h.t->target_deg);
    h.t->completeMove();
    tick(*h.g);  // Theta orient completes -> Z+ seek phase
    tick(*h.g);  // Z+ seek starts

    // 6. Z+ seek starts
    TEST_ASSERT_TRUE(h.z->busy);
    h.z->a014 = true;
    tick(*h.g);
    h.z->a014 = false;
    h.z->completeMove();
    tick(*h.g);  // creep cleared -> settle
    tick(*h.g);  // settle -> Z return to band ceiling

    // 7. Z return to SAFE_Z ceiling
    TEST_ASSERT_TRUE(h.z->busy);
    h.z->completeMove();
    tick(*h.g);  // return completes

    // Bring-up successfully completed
    TEST_ASSERT_FALSE(h.g->eipBringUpInProgress());
}

// Drive bring-up through Z-, X home/cal and X park until the theta orient
// to 90 deg has been commanded.
static void runBringUpToThetaOrient(Harness& h) {
    TEST_ASSERT_TRUE(h.g->startEipBringUp());
    tick(*h.g);
    h.z->a015 = true;
    tick(*h.g);
    h.z->a015 = false;
    h.z->completeMove();
    tick(*h.g);
    tick(*h.g);
    tick(*h.g);
    h.x->a014 = true;
    tick(*h.g);
    h.x->a014 = false;
    h.x->completeMove();
    tick(*h.g);
    tick(*h.g);
    tick(*h.g);
    h.x->a015 = true;
    tick(*h.g);
    h.x->a015 = false;
    h.x->completeMove();
    tick(*h.g);
    tick(*h.g);
    h.x->completeMove();
    tick(*h.g);
    TEST_ASSERT_TRUE(h.t->busy);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 90.0f, h.t->target_deg);
}

static void assertBringUpStoppedBeforeZPlus(Harness& h) {
    for (int i = 0; i < 5; ++i) {
        tick(*h.g);
    }
    TEST_ASSERT_FALSE_MESSAGE(h.g->eipBringUpInProgress(), "bring-up must stop on a failed orient");
    TEST_ASSERT_FALSE_MESSAGE(h.z->busy, "Z+ stroke must not start with theta off target");
}

// The drive faults mid-orient (F2028): theta goes idle short of 90 deg.
static void test_bringup_fails_when_theta_orient_faults(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    runBringUpToThetaOrient(h);
    h.t->deg = 38.0f;
    h.t->busy = false;
    h.t->alarm = true;
    tick(*h.g);
    assertBringUpStoppedBeforeZPlus(h);
}

// The axis times out and halts without an alarm: still not at 90 deg.
static void test_bringup_fails_when_theta_orient_times_out(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    runBringUpToThetaOrient(h);
    h.t->deg = 40.0f;
    h.t->busy = false;
    tick(*h.g);
    assertBringUpStoppedBeforeZPlus(h);
}

// Any one dropped drive link (2026-10-20: all three during a Z retract) makes
// the gantry not online; it is online again once every link is back.
static void test_drives_online_requires_every_link(void) {
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
static void test_inactive_servo_axis_reports_first_off_servo(void) {
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

static void test_theta_settled_at_requires_idle_no_alarm_and_position(void) {
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

static void test_sequential_theta_after_in_band_path(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 40.0f, 0.0f, 15.0f, 50));
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
    completeLinear(h);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(1u, h.t->moves.size());
}

static void test_sequential_theta_gated_when_path_ends_deep(void) {
    Harness h = makeHarness(0.0f, 10.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 80.0f, 15.0f, 50));
    completeLinear(h);
    tick(*h.g);
    TEST_ASSERT_EQUAL_UINT(0u, h.t->moves.size());
}

#endif

static void test_z_cap_on_z_only_segment(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 0.0f, 100.0f, 0.0f, 2000));
    TEST_ASSERT_TRUE(h.z->busy);
    const float cap = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, cap < 2000.0f ? cap : 2000.0f, h.z->moves.back().speed_mm_s);
}

static void test_z_cap_on_combined_segment_keeps_it_straight(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 30.0f, 0.0f, 5000));
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_TRUE(h.z->busy);
    const float cap = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, cap, h.z->moves.back().speed_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, cap * 100.0f / 30.0f, h.x->moves.back().speed_mm_s);
}

static void test_retract_to_safe_z_while_theta_alarmed(void) {
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

static void test_retract_stops_a_move_in_progress(void) {
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

static void test_grip_reports_write_failure(void) {
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
static void test_grip_fails_when_latch_does_not_confirm(void) {
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

static void test_cross_axis_fault_stops_orchestrator(void) {
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

static void test_z_accel_cap_preserves_full_x_accel(void) {
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

static void test_retract_to_safe_z_noop_inside_traverse_band(void) {
    Harness h = makeHarness(382.5f, 30.0f, 0.0f);
    // Kinetix settles inside 0.5 mm arrival band (30.08 mm <= 30.0 + 0.5 mm).
    h.z->mm = h.z->target_mm = 30.08f;
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->retractToSafeZ(900, 5000, 5000));
    TEST_ASSERT_FALSE(h.g->isBusy());
    TEST_ASSERT_EQUAL_UINT(0u, h.z->moves.size());
}

static void test_chain_move_after_retract_refused_while_preload_held(void) {
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

static void test_chain_move_after_retract_arms_x_and_theta_on_band_entry_then_descends(void) {
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
static void test_theta_full_speed_skips_window_throttling(void) {
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

static void test_bin_retract_pick_move_arms_x_before_z_completes(void) {
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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_move_not_initialized);
    RUN_TEST(test_move_not_enabled);
    RUN_TEST(test_move_not_calibrated);
    RUN_TEST(test_workspace_cal_clears_on_drive_lost);
    RUN_TEST(test_workspace_cal_clears_on_theta_lost);
    RUN_TEST(test_drive_fault_stop_does_not_rezero_x);
    RUN_TEST(test_move_invalid_position);
    RUN_TEST(test_orchestrator_bubbles_up_axis_rejection);
    RUN_TEST(test_move_already_moving);
    RUN_TEST(test_theta_refused_no_in_band_segment);
    RUN_TEST(test_theta_only_while_deep_refused);
    RUN_TEST(test_theta_only_to_joint_limits_in_band);
    RUN_TEST(test_theta_only_return_to_zero_stays_busy);
#if !CONFIG_GANTRY_THETA_SEQUENTIAL
    RUN_TEST(test_theta_does_not_start_before_25_percent);
    RUN_TEST(test_theta_starts_at_25_percent_window_speed);
    RUN_TEST(test_descent_held_until_theta_idle);
    RUN_TEST(test_stop_clears_pending_theta);
    RUN_TEST(test_retract_starts_theta_after_band_entry);
    RUN_TEST(test_theta_start_fraction_zero_starts_with_x);
    RUN_TEST(test_theta_start_fraction_caps_at_move_theta_speed);
    RUN_TEST(test_pick_theta_commands_a_tenth_while_waiting);
    RUN_TEST(test_pick_theta_truncates_below_a_tenth);
    RUN_TEST(test_pick_theta_quantizes_to_a_tenth);
    RUN_TEST(test_pick_theta_runs_with_x_below_half_degree);
    RUN_TEST(test_console_still_ignores_a_fifth_of_a_degree);
    RUN_TEST(test_theta_refused_below_band_during_wait);
    RUN_TEST(test_budget_matches_timed_fake_pre_position);
#endif
    RUN_TEST(test_fail_path_off_target);
    RUN_TEST(test_deferred_x_arms_once_in_band);
    RUN_TEST(test_in_band_xz_together);
    RUN_TEST(test_three_segment_outbound);
    RUN_TEST(test_above_to_above_arms_x_at_band_entry);
    RUN_TEST(test_pnp_gripper_then_retract);
    RUN_TEST(test_bringup_requires_drive_managed);
    RUN_TEST(test_bringup_z_minus_then_timeout);
    RUN_TEST(test_bringup_phase_order_to_x_home);
    RUN_TEST(test_bringup_full_sequence_with_theta_90_orient);
    RUN_TEST(test_bringup_fails_when_theta_orient_faults);
    RUN_TEST(test_bringup_fails_when_theta_orient_times_out);
    RUN_TEST(test_theta_settled_at_requires_idle_no_alarm_and_position);
    RUN_TEST(test_drives_online_requires_every_link);
    RUN_TEST(test_inactive_servo_axis_reports_first_off_servo);
#if CONFIG_GANTRY_THETA_SEQUENTIAL
    RUN_TEST(test_sequential_theta_after_in_band_path);
    RUN_TEST(test_sequential_theta_gated_when_path_ends_deep);
#endif
    RUN_TEST(test_z_cap_on_z_only_segment);
    RUN_TEST(test_z_cap_on_combined_segment_keeps_it_straight);
    RUN_TEST(test_z_accel_cap_preserves_full_x_accel);
    RUN_TEST(test_retract_to_safe_z_while_theta_alarmed);
    RUN_TEST(test_retract_to_safe_z_noop_inside_traverse_band);
    RUN_TEST(test_chain_move_after_retract_refused_while_preload_held);
    RUN_TEST(test_chain_move_after_retract_arms_x_and_theta_on_band_entry_then_descends);
#if !CONFIG_GANTRY_THETA_SEQUENTIAL
    RUN_TEST(test_theta_full_speed_skips_window_throttling);
#endif
    RUN_TEST(test_bin_retract_pick_move_arms_x_before_z_completes);
    RUN_TEST(test_retract_stops_a_move_in_progress);
    RUN_TEST(test_grip_reports_write_failure);
    RUN_TEST(test_grip_fails_when_latch_does_not_confirm);
    RUN_TEST(test_cross_axis_fault_stops_orchestrator);
    return UNITY_END();
}
