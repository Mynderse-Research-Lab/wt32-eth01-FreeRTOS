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
#include "host_test_clock.h"

#include <memory>

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

static void test_move_invalid_position(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::INVALID_POSITION,
                      go(h, 999.0f, 0.0f, 0.0f));
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

static void test_pnp_gripper_then_retract(void) {
    Harness h = makeHarness();
    EndEffectorPose pose;
    pose.x = Gantry::KinematicParameters().theta_x_offset_mm;
    pose.y = 0.0f;
    pose.z = 80.0f;
    pose.theta = 0.0f;
    TEST_ASSERT_EQUAL(GantryError::OK, h.g->moveTo(pose, 50, 30, 3000, 3000));
    h.z->completeMove();
    tick(*h.g);
    // Gripper dwell then retract to SAFE_Z ceiling.
    HostTest::advanceMs(150);
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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_move_not_initialized);
    RUN_TEST(test_move_not_enabled);
    RUN_TEST(test_move_invalid_position);
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
#endif
    RUN_TEST(test_fail_path_off_target);
    RUN_TEST(test_deferred_x_arms_once_in_band);
    RUN_TEST(test_in_band_xz_together);
    RUN_TEST(test_three_segment_outbound);
    RUN_TEST(test_pnp_gripper_then_retract);
    RUN_TEST(test_bringup_requires_drive_managed);
    RUN_TEST(test_bringup_z_minus_then_timeout);
    RUN_TEST(test_bringup_phase_order_to_x_home);
#if CONFIG_GANTRY_THETA_SEQUENTIAL
    RUN_TEST(test_sequential_theta_after_in_band_path);
    RUN_TEST(test_sequential_theta_gated_when_path_ends_deep);
#endif
    return UNITY_END();
}
