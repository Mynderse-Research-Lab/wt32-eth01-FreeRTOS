/**
 * @file test_gantry_orchestration_bringup.cpp
 * @brief Path and bring-up host tests.
 */

#include "test_gantry_orchestration_internal.h"

using namespace gantry_orch_test;

void test_fail_path_off_target(void) {
    Harness h = makeHarness();
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 100.0f, 0.0f, 0.0f));
    h.x->setMm(10.0f);
    h.x->busy = false;
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->isBusy());
    TEST_ASSERT_TRUE(h.x->stop_count >= 1);
}

void test_deferred_x_arms_once_in_band(void) {
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

void test_in_band_xz_together(void) {
    Harness h = makeHarness(0.0f, 10.0f, 0.0f);
    TEST_ASSERT_EQUAL(GantryError::OK, go(h, 40.0f, 0.0f, 0.0f));
    TEST_ASSERT_TRUE(h.x->busy);
    TEST_ASSERT_TRUE(h.z->busy);
}

void test_three_segment_outbound(void) {
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

void test_above_to_above_arms_x_at_band_entry(void) {
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

void test_pnp_gripper_then_retract(void) {
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

void test_bringup_requires_drive_managed(void) {
    auto xu = std::make_unique<FakeLinearAxis>();
    auto zu = std::make_unique<FakeLinearAxis>();
    auto tu = std::make_unique<FakeRotaryAxis>();
    Gantry::Gantry g(std::move(xu), std::move(zu), std::move(tu), -1);
    TEST_ASSERT_TRUE(g.begin());
    g.enable();
    TEST_ASSERT_FALSE(g.startEipBringUp());
}

void test_bringup_z_minus_then_timeout(void) {
    Harness h = makeHarness(35.0f, 30.0f, 0.0f);
    TEST_ASSERT_TRUE(h.g->startEipBringUp());
    TEST_ASSERT_TRUE(h.g->eipBringUpInProgress());
    tick(*h.g);
    TEST_ASSERT_TRUE(h.z->busy);
    HostTest::advanceMs(90001);
    tick(*h.g);
    TEST_ASSERT_FALSE(h.g->eipBringUpInProgress());
}

void test_bringup_phase_order_to_x_home(void) {
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

void test_bringup_full_sequence_with_theta_90_orient(void) {
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
void test_bringup_fails_when_theta_orient_faults(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    runBringUpToThetaOrient(h);
    h.t->deg = 38.0f;
    h.t->busy = false;
    h.t->alarm = true;
    tick(*h.g);
    assertBringUpStoppedBeforeZPlus(h);
}

// The axis times out and halts without an alarm: still not at 90 deg.
void test_bringup_fails_when_theta_orient_times_out(void) {
    Harness h = makeHarness(0.0f, 0.0f, 0.0f);
    runBringUpToThetaOrient(h);
    h.t->deg = 40.0f;
    h.t->busy = false;
    tick(*h.g);
    assertBringUpStoppedBeforeZPlus(h);
}
