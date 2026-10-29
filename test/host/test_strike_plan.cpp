/**
 * @file test_strike_plan.cpp
 * @brief Host tests for the simulation strike planner port (strike_plan.h).
 *
 * Reference values come from compute_strike_plan() in tools/sim/gantry_twin.py
 * with the sim geometry (belt 101.95, SAFE_Z 44.82, 90 mm jaw, 800 mm/s,
 * 20000 mm/s2, 8 ms valve, 15 ms tau, 450 mm/s jaw) and zero offsets.
 */

#include "strike_plan.h"

#include "unity.h"

#include <cmath>

// Unity is built without double support; float is exact enough at these magnitudes.
#define ASSERT_NEAR(tol, expected, actual)                                        \
    TEST_ASSERT_FLOAT_WITHIN(static_cast<float>(tol), static_cast<float>(expected), \
                             static_cast<float>(actual))

void setUp(void) {}
void tearDown(void) {}

namespace {

constexpr double kTolS = 2e-6;

StrikePlan::Inputs simInputs(double width_mm, double height_mm, double theta_deg,
                             double belt_mm_s, double z_grasp_mm) {
    StrikePlan::Inputs in;
    in.width_mm = width_mm;
    in.height_mm = height_mm;
    in.theta_deg = theta_deg;
    in.belt_mm_s = belt_mm_s;
    in.z_belt_mm = 101.95;
    in.z_ceiling_mm = 44.82;
    in.z_min_mm = -50.0;
    in.z_grasp_mm = z_grasp_mm;
    in.pad_half_open_mm = 45.0;
    in.vz_mm_s = 800.0;
    in.az_mm_s2 = 20000.0;
    in.valve_delay_s = 0.008;
    return in;
}

void assertPlan(const StrikePlan::Plan& p, double tau_plunge, double tau_band, double t_z,
                double t_close, double t_retract) {
    TEST_ASSERT_TRUE(p.ok);
    TEST_ASSERT_FALSE(p.clamped);
    ASSERT_NEAR(kTolS, tau_plunge, p.tau_plunge_s);
    ASSERT_NEAR(kTolS, tau_band, p.tau_band_s);
    ASSERT_NEAR(kTolS, t_z, p.t_z_s);
    ASSERT_NEAR(kTolS, t_close, p.t_close_s);
    ASSERT_NEAR(kTolS, t_retract, p.t_retract_s);
}

}  // namespace

// Crossing belt (theta 0): the plunge waits for the cell to enter the gap.
static void test_matches_sim_nine_volt_flatface_600_at_90(void) {
    assertPlan(StrikePlan::plan(simInputs(26.6, 17.3, 0.0, 600.0, 99.0)), 0.107725, 0.044614,
               -0.107611, -0.085280, 0.000114);
}

// Belt along the jaws (theta 90): no crossing, depth reached at arrival.
static void test_matches_sim_nine_volt_shortface_1500_at_0(void) {
    assertPlan(StrikePlan::plan(simInputs(17.3, 26.6, 90.0, 1500.0, 99.0)), 0.107725, 0.052471,
               -0.107725, -0.095752, 0.000000);
}

// Slow belt: depth is reached at the 15 ms cap after arrival.
static void test_matches_sim_pack_300_at_45(void) {
    assertPlan(StrikePlan::plan(simInputs(68.3, 26.7, 45.0, 300.0, 99.0)), 0.107725, 0.052562,
               -0.092725, -0.022913, 0.015000);
}

static void test_matches_sim_power_tool_makita_1000_at_30(void) {
    assertPlan(StrikePlan::plan(simInputs(73.8, 49.6, 60.0, 1000.0, 98.0)), 0.106475, 0.079034,
               -0.091475, -0.016115, 0.015000);
}

static void test_each_offset_moves_only_its_event(void) {
    const StrikePlan::Inputs base = simInputs(26.6, 17.3, 0.0, 600.0, 99.0);
    const StrikePlan::Plan p0 = StrikePlan::plan(base);

    StrikePlan::Inputs in = base;
    in.plunge_offset_ms = 5;
    StrikePlan::Plan p = StrikePlan::plan(in);
    ASSERT_NEAR(kTolS, p0.t_z_s + 0.005, p.t_z_s);
    ASSERT_NEAR(kTolS, p0.t_close_s, p.t_close_s);
    // A later plunge pushes the retract out so the full stroke is reached.
    ASSERT_NEAR(kTolS, p.t_z_s + p.tau_plunge_s, p.t_retract_s);

    in = base;
    in.close_offset_ms = -7;
    p = StrikePlan::plan(in);
    ASSERT_NEAR(kTolS, p0.t_z_s, p.t_z_s);
    ASSERT_NEAR(kTolS, p0.t_close_s - 0.007, p.t_close_s);
    ASSERT_NEAR(kTolS, p0.t_retract_s, p.t_retract_s);
    TEST_ASSERT_FALSE(p.clamped);

    in = base;
    in.retract_offset_ms = 12;
    p = StrikePlan::plan(in);
    ASSERT_NEAR(kTolS, p0.t_z_s, p.t_z_s);
    ASSERT_NEAR(kTolS, p0.t_close_s, p.t_close_s);
    ASSERT_NEAR(kTolS, p0.t_retract_s + 0.012, p.t_retract_s);
    TEST_ASSERT_FALSE(p.clamped);
}

// The retract never reverses Z before the plunge finishes.
static void test_early_retract_is_clamped_to_full_plunge(void) {
    StrikePlan::Inputs in = simInputs(26.6, 17.3, 0.0, 600.0, 99.0);
    in.retract_offset_ms = -40;
    const StrikePlan::Plan p = StrikePlan::plan(in);
    TEST_ASSERT_TRUE(p.ok);
    TEST_ASSERT_TRUE(p.clamped);
    ASSERT_NEAR(kTolS, p.t_z_s + p.tau_plunge_s, p.t_retract_s);
}

static void test_close_stays_between_plunge_and_retract(void) {
    StrikePlan::Inputs in = simInputs(26.6, 17.3, 0.0, 600.0, 99.0);
    in.close_offset_ms = -100;
    StrikePlan::Plan p = StrikePlan::plan(in);
    TEST_ASSERT_TRUE(p.clamped);
    ASSERT_NEAR(kTolS, p.t_z_s + StrikePlan::kMinGapS, p.t_close_s);

    in.close_offset_ms = 100;
    p = StrikePlan::plan(in);
    TEST_ASSERT_TRUE(p.clamped);
    ASSERT_NEAR(kTolS, p.t_retract_s - StrikePlan::kMinGapS, p.t_close_s);
}

static void test_offsets_are_limited_to_100_ms(void) {
    StrikePlan::Inputs in = simInputs(17.3, 26.6, 90.0, 1500.0, 99.0);
    const StrikePlan::Plan p0 = StrikePlan::plan(in);
    in.retract_offset_ms = 250;
    const StrikePlan::Plan p = StrikePlan::plan(in);
    TEST_ASSERT_TRUE(p.clamped);
    ASSERT_NEAR(kTolS, p0.t_retract_s + 0.100, p.t_retract_s);
}

static void test_hover_clears_tall_object_and_respects_ceiling(void) {
    // Short cell: hover at the band ceiling.
    StrikePlan::Plan p = StrikePlan::plan(simInputs(26.6, 17.3, 0.0, 600.0, 99.0));
    ASSERT_NEAR(1e-9, 44.82, p.z_hover_mm);
    // Tall shortface power tool (111.7 mm): hover 6 mm above its top.
    p = StrikePlan::plan(simInputs(49.6, 73.8, 90.0, 600.0, 61.0));
    TEST_ASSERT_TRUE(p.ok);
    ASSERT_NEAR(1e-9, 101.95 - 73.8 - 6.0, p.z_hover_mm);
}

static void test_rejects_unpickable_inputs(void) {
    TEST_ASSERT_FALSE(StrikePlan::plan(simInputs(0.0, 17.3, 0.0, 600.0, 99.0)).ok);
    TEST_ASSERT_FALSE(StrikePlan::plan(simInputs(92.1, 53.0, 0.0, 600.0, 99.0)).ok);
    TEST_ASSERT_FALSE(StrikePlan::plan(simInputs(26.6, 17.3, 0.0, 0.0, 99.0)).ok);
    // Grasp above the hover cannot be reached by a plunge.
    TEST_ASSERT_FALSE(StrikePlan::plan(simInputs(26.6, 17.3, 0.0, 600.0, 30.0)).ok);
}

static void test_jaw_stroke_solves_pneumatic_model(void) {
    const double t = StrikePlan::jawStrokeSeconds(31.7, 450.0, 0.015);
    const double q = 450.0 * (t - 0.015 * (1.0 - std::exp(-t / 0.015)));
    ASSERT_NEAR(1e-6, 31.7, q);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_matches_sim_nine_volt_flatface_600_at_90);
    RUN_TEST(test_matches_sim_nine_volt_shortface_1500_at_0);
    RUN_TEST(test_matches_sim_pack_300_at_45);
    RUN_TEST(test_matches_sim_power_tool_makita_1000_at_30);
    RUN_TEST(test_each_offset_moves_only_its_event);
    RUN_TEST(test_early_retract_is_clamped_to_full_plunge);
    RUN_TEST(test_close_stays_between_plunge_and_retract);
    RUN_TEST(test_offsets_are_limited_to_100_ms);
    RUN_TEST(test_hover_clears_tall_object_and_respects_ceiling);
    RUN_TEST(test_rejects_unpickable_inputs);
    RUN_TEST(test_jaw_stroke_solves_pneumatic_model);
    return UNITY_END();
}
