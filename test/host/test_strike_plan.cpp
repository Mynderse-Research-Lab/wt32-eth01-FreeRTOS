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

// Crossing belt (theta 0): exact trapezoid top-crossing (G-C6 fix) places
// tau_band at 37.881 ms (previously over-predicted as 44.614 ms by accel-only formula).
static void test_matches_sim_nine_volt_flatface_600_at_90(void) {
    assertPlan(StrikePlan::plan(simInputs(26.6, 17.3, 0.0, 600.0, 99.0)), 0.107725, 0.037881,
               -0.107725, -0.085394, 0.000000);
}

// Belt along the jaws (theta 90): no crossing, depth reached at arrival.
static void test_matches_sim_nine_volt_shortface_1500_at_0(void) {
    assertPlan(StrikePlan::plan(simInputs(17.3, 26.6, 90.0, 1500.0, 99.0)), 0.107725, 0.049563,
               -0.107725, -0.095752, 0.000000);
}

// Slow belt: depth is reached 12.968 ms after arrival using exact tau_band.
static void test_matches_sim_pack_300_at_45(void) {
    assertPlan(StrikePlan::plan(simInputs(68.3, 26.7, 45.0, 300.0, 99.0)), 0.107725, 0.049688,
               -0.094757, -0.024945, 0.012968);
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

// The plan reports the bottom dwell the offsets ask for, either sign.
static void test_reports_requested_dwell(void) {
    const StrikePlan::Inputs base = simInputs(26.6, 17.3, 0.0, 600.0, 99.0);
    ASSERT_NEAR(kTolS, 0.0, StrikePlan::plan(base).dwell_s);

    StrikePlan::Inputs in = base;
    in.retract_offset_ms = 30;
    ASSERT_NEAR(kTolS, 0.030, StrikePlan::plan(in).dwell_s);

    // An earlier plunge with the retract unchanged leaves Z waiting at depth.
    in = base;
    in.plunge_offset_ms = -20;
    ASSERT_NEAR(kTolS, 0.020, StrikePlan::plan(in).dwell_s);

    // A later plunge asks for the retract first; it is held to the plunge end.
    in = base;
    in.plunge_offset_ms = 20;
    const StrikePlan::Plan p = StrikePlan::plan(in);
    ASSERT_NEAR(kTolS, -0.020, p.dwell_s);
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

static void test_plunge_time_uses_the_z_decel(void) {
    StrikePlan::Inputs in = simInputs(26.6, 17.3, 0.0, 600.0, 99.0);
    in.az_mm_s2 = 5000.0;
    const double dist = 99.0 - 44.82;
    // 800 mm/s needs 64 + 64 mm at 5000 mm/s2, so this stroke is a triangle.
    in.dz_mm_s2 = 0.0;
    const StrikePlan::Plan same = StrikePlan::plan(in);
    ASSERT_NEAR(1e-9, 2.0 * std::sqrt(dist / 5000.0), same.tau_plunge_s);

    in.dz_mm_s2 = 10000.0;
    const StrikePlan::Plan fast = StrikePlan::plan(in);
    const double v_peak = std::sqrt(2.0 * dist * 5000.0 * 10000.0 / 15000.0);
    ASSERT_NEAR(1e-9, v_peak / 5000.0 + v_peak / 10000.0, fast.tau_plunge_s);

    // Long stroke: full-speed cruise between ramps of 64 mm and 32 mm.
    ASSERT_NEAR(1e-9, 0.16 + 0.08 + (300.0 - 96.0) / 800.0,
                StrikePlan::trapezoidSeconds(300.0, 800.0, 5000.0, 10000.0));
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

static void test_trapezoid_time_to_distance_accel_cruise_decel(void) {
    // Trapezoidal stroke: D=300, v=800, a=5000 (d_acc=64, t_acc=0.16), d=10000 (d_dec=32, t_dec=0.08).
    // Total duration = 0.16 + 0.08 + (300 - 96)/800 = 0.495 s.
    ASSERT_NEAR(1e-6, 0.0, StrikePlan::trapezoidTimeToDistance(0.0, 300.0, 800.0, 5000.0, 10000.0));
    ASSERT_NEAR(1e-6, std::sqrt(2.0 * 25.0 / 5000.0),
                StrikePlan::trapezoidTimeToDistance(25.0, 300.0, 800.0, 5000.0, 10000.0));
    ASSERT_NEAR(1e-6, 0.16 + (164.0 - 64.0) / 800.0,
                StrikePlan::trapezoidTimeToDistance(164.0, 300.0, 800.0, 5000.0, 10000.0));
    ASSERT_NEAR(1e-6, 0.495 - std::sqrt(2.0 * 18.0 / 10000.0),
                StrikePlan::trapezoidTimeToDistance(282.0, 300.0, 800.0, 5000.0, 10000.0));
    ASSERT_NEAR(1e-6, 0.495,
                StrikePlan::trapezoidTimeToDistance(300.0, 300.0, 800.0, 5000.0, 10000.0));

    // Triangular stroke (live 9V plunge: 70.6 mm at vz=900, az=dz=5000; to_top=47.1 mm > 35.3 mm mid).
    const double total = StrikePlan::trapezoidSeconds(70.6, 900.0, 5000.0, 5000.0);
    const double t_top = StrikePlan::trapezoidTimeToDistance(47.1, 70.6, 900.0, 5000.0, 5000.0);
    const double expected_band = std::sqrt(2.0 * 23.5 / 5000.0);
    ASSERT_NEAR(1e-6, expected_band, total - t_top);
}

static void test_z_deadtime_keeps_physical_events_invariant_across_speeds(void) {
    // Live rig 9V at 45 deg across 300..600 mm/s: modeling Z plunge/retract
    // deadtime shifts command times earlier while physical top crossing, grasp
    // arrival, lift start, and jaw contact stay identical to zero-deadtime physics.
    const double speeds[] = {300.0, 400.0, 500.0, 600.0};
    for (double v : speeds) {
        StrikePlan::Inputs ideal;
        ideal.width_mm = 17.5;
        ideal.height_mm = 26.5;
        ideal.theta_deg = 45.0;
        ideal.belt_mm_s = v;
        ideal.z_belt_mm = 122.0;
        ideal.z_ceiling_mm = 48.4;
        ideal.z_min_mm = 0.0;
        ideal.z_grasp_mm = 119.0;
        ideal.pad_half_open_mm = 40.0;
        ideal.vz_mm_s = 900.0;
        ideal.az_mm_s2 = 5000.0;
        ideal.dz_mm_s2 = 5000.0;
        ideal.valve_delay_s = 0.030;
        ideal.jaw_speed_mm_s = 210.0;

        StrikePlan::Inputs with_dead = ideal;
        with_dead.z_plunge_dead_s = 0.030;
        with_dead.z_retract_dead_s = 0.022;

        const StrikePlan::Plan p0 = StrikePlan::plan(ideal);
        const StrikePlan::Plan pd = StrikePlan::plan(with_dead);
        TEST_ASSERT_TRUE(p0.ok);
        TEST_ASSERT_TRUE(pd.ok);
        TEST_ASSERT_FALSE(pd.clamped);

        // Command timestamps advance by exactly each axis's deadtime.
        ASSERT_NEAR(kTolS, p0.t_z_s - 0.030, pd.t_z_s);
        ASSERT_NEAR(kTolS, p0.t_retract_s - 0.022, pd.t_retract_s);
        ASSERT_NEAR(kTolS, p0.t_close_s, pd.t_close_s);

        // Physical event timestamps are 100% invariant to actuator deadtime.
        ASSERT_NEAR(kTolS, p0.t_z_done_s, pd.t_z_s + pd.z_plunge_dead_s + pd.tau_plunge_s);
        ASSERT_NEAR(kTolS, p0.t_z_done_s, pd.t_retract_s + pd.z_retract_dead_s);
        ASSERT_NEAR(kTolS, p0.t_top_s,
                    pd.t_z_s + pd.z_plunge_dead_s + (pd.tau_plunge_s - pd.tau_band_s));
        ASSERT_NEAR(kTolS, 0.0, pd.dwell_s);
    }
}

static void test_z_deadtime_retract_floor_prevents_midair_reversal(void) {
    StrikePlan::Inputs in = simInputs(17.5, 26.5, 45.0, 500.0, 99.0);
    in.az_mm_s2 = 5000.0;
    in.dz_mm_s2 = 5000.0;
    in.z_plunge_dead_s = 0.030;
    in.z_retract_dead_s = 0.022;
    in.retract_offset_ms = -15;

    const StrikePlan::Plan p = StrikePlan::plan(in);
    TEST_ASSERT_TRUE(p.ok);
    TEST_ASSERT_TRUE(p.clamped);
    ASSERT_NEAR(kTolS, -0.015, p.dwell_s);
    // Physical retract start (t_retract_s + z_retract_dead) is held to physical plunge end.
    ASSERT_NEAR(kTolS, p.t_z_s + p.z_plunge_dead_s + p.tau_plunge_s,
                p.t_retract_s + p.z_retract_dead_s);
    ASSERT_NEAR(kTolS, p.z_plunge_dead_s + p.tau_plunge_s - p.z_retract_dead_s,
                p.z_to_retract_floor_s);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_matches_sim_nine_volt_flatface_600_at_90);
    RUN_TEST(test_matches_sim_nine_volt_shortface_1500_at_0);
    RUN_TEST(test_matches_sim_pack_300_at_45);
    RUN_TEST(test_matches_sim_power_tool_makita_1000_at_30);
    RUN_TEST(test_each_offset_moves_only_its_event);
    RUN_TEST(test_early_retract_is_clamped_to_full_plunge);
    RUN_TEST(test_reports_requested_dwell);
    RUN_TEST(test_close_stays_between_plunge_and_retract);
    RUN_TEST(test_offsets_are_limited_to_100_ms);
    RUN_TEST(test_hover_clears_tall_object_and_respects_ceiling);
    RUN_TEST(test_plunge_time_uses_the_z_decel);
    RUN_TEST(test_rejects_unpickable_inputs);
    RUN_TEST(test_jaw_stroke_solves_pneumatic_model);
    RUN_TEST(test_trapezoid_time_to_distance_accel_cruise_decel);
    RUN_TEST(test_z_deadtime_keeps_physical_events_invariant_across_speeds);
    RUN_TEST(test_z_deadtime_retract_floor_prevents_midair_reversal);
    return UNITY_END();
}
