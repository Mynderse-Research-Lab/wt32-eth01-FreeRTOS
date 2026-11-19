// Host tests for theta_trace_analysis.h (thtrace summary of one HCS01 move).

#include "unity.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include "theta_trace_analysis.h"

// Unity is built without double support.
#define ASSERT_NEAR(tol, expected, actual)     TEST_ASSERT_TRUE_MESSAGE(std::fabs((actual) - (expected)) <= (tol), #actual)

void setUp(void) {}
void tearDown(void) {}

namespace {

constexpr double kPi = 3.14159265358979323846;

// 0 -> -45 deg at 1980 deg/s, 9900 deg/s2: triangle, peak sqrt(45 * 9900).
ThetaTrace::Move moveMinus45() {
    ThetaTrace::Move m;
    m.dist_deg = -45.0;
    m.vel_deg_s = 1980.0;
    m.acc_deg_s2 = 9900.0;
    m.dec_deg_s2 = 9900.0;
    m.start_edge_us = 50000.0;
    return m;
}

struct Synth {
    std::vector<int32_t> t;
    std::vector<float> rel;
    std::vector<float> vel;
};

// Ideal profile from t0_us, then a decaying ring of amp_deg past the target.
Synth synth(const ThetaTrace::Move& m, double t0_us, double amp_deg, double tau_s,
            double ring_hz, int n) {
    const Gantry::TrapezoidalProfile p = Gantry::TrajectoryPlanner::calculateProfile(
        0.0f, static_cast<float>(m.dist_deg), static_cast<float>(m.vel_deg_s),
        static_cast<float>(m.acc_deg_s2), static_cast<float>(m.dec_deg_s2));
    const double t_end_us = t0_us + static_cast<double>(p.total_time) * 1.0e6;
    const double dir = m.dist_deg < 0.0 ? -1.0 : 1.0;
    Synth s;
    double prev = 0.0;
    for (int i = 0; i < n; ++i) {
        const double t_us = 1000.0 * i;
        double x = ThetaTrace::idealDeg(p, m.dist_deg, (t_us - t0_us) / 1.0e6);
        if (t_us > t_end_us) {
            const double dt = (t_us - t_end_us) / 1.0e6;
            x += dir * amp_deg * std::exp(-dt / tau_s) * std::sin(2.0 * kPi * ring_hz * dt);
        }
        s.t.push_back(static_cast<int32_t>(t_us));
        s.rel.push_back(static_cast<float>(x));
        s.vel.push_back(static_cast<float>(i > 0 ? (x - prev) / 0.001 : 0.0));
        prev = x;
    }
    return s;
}

}  // namespace

static void test_profile_time_triangle(void) {
    const ThetaTrace::Move m = moveMinus45();
    const Synth s = synth(m, 60000.0, 0.0, 0.02, 25.0, 400);
    const ThetaTrace::Result r = ThetaTrace::analyze(s.t.data(), s.rel.data(), s.vel.data(),
                                                     static_cast<int>(s.t.size()), m);
    // 2 * sqrt(45 / 9900) s
    ASSERT_NEAR(0.05, 2000.0 * std::sqrt(45.0 / 9900.0), r.profile_ms);
}

static void test_ideal_trace_has_latency_and_no_error(void) {
    const ThetaTrace::Move m = moveMinus45();
    const Synth s = synth(m, 60000.0, 0.0, 0.02, 25.0, 400);
    const ThetaTrace::Result r = ThetaTrace::analyze(s.t.data(), s.rel.data(), s.vel.data(),
                                                     static_cast<int>(s.t.size()), m);
    TEST_ASSERT_TRUE(r.i_motion > 0);
    ASSERT_NEAR(20.0, 60000.0, r.t0_us);
    ASSERT_NEAR(0.02, 10.0, r.latency_ms);
    TEST_ASSERT_TRUE(r.max_follow_deg < 0.002);
    TEST_ASSERT_TRUE(r.overshoot_deg < 1e-4);
    ASSERT_NEAR(1e-4, 0.0, r.final_err_deg);
    TEST_ASSERT_TRUE(r.i_settle >= r.i_band);
    TEST_ASSERT_TRUE(r.i_band > r.i_motion);
    // Peak speed of the triangle, from the synthetic finite difference.
    ASSERT_NEAR(10.0, std::sqrt(45.0 * 9900.0), r.peak_vel_deg_s);
}

static void test_ring_gives_overshoot_and_late_settle(void) {
    const ThetaTrace::Move m = moveMinus45();
    const double t0_us = 60000.0;
    const double amp = 0.5;
    const double tau = 0.02;
    const double hz = 25.0;
    const Synth s = synth(m, t0_us, amp, tau, hz, 600);
    const int n = static_cast<int>(s.t.size());
    const ThetaTrace::Result r = ThetaTrace::analyze(s.t.data(), s.rel.data(), s.vel.data(), n, m);

    // Expected from the samples themselves: largest excursion past -45 and
    // the last sample outside the 0.1 deg band.
    double over = 0.0;
    int last_out = -1;
    for (int i = 0; i < n; ++i) {
        const double e = static_cast<double>(s.rel[i]) - m.dist_deg;
        if (-e > over) {
            over = -e;
        }
        if (std::fabs(e) > ThetaTrace::kSettleBandDeg) {
            last_out = i;
        }
    }
    TEST_ASSERT_TRUE(over > 0.25);
    ASSERT_NEAR(1e-6, over, r.overshoot_deg);
    TEST_ASSERT_EQUAL_INT(last_out + 1, r.i_settle);
    TEST_ASSERT_TRUE(r.i_band < r.i_settle);
    // The ring starts after the ideal ends, so tracking stays tight.
    TEST_ASSERT_TRUE(r.max_follow_deg < 0.002);
}

static void test_lagging_axis_shows_following_error(void) {
    const ThetaTrace::Move m = moveMinus45();
    Synth s = synth(m, 60000.0, 0.0, 0.02, 25.0, 400);
    // Decel 20 % weaker than commanded: the last part lags the ideal.
    const Gantry::TrapezoidalProfile slow = Gantry::TrajectoryPlanner::calculateProfile(
        0.0f, -45.0f, 1980.0f, 9900.0f, 7920.0f);
    for (size_t i = 0; i < s.t.size(); ++i) {
        s.rel[i] = static_cast<float>(
            ThetaTrace::idealDeg(slow, -45.0, (s.t[i] - 60000.0) / 1.0e6));
    }
    const ThetaTrace::Result r = ThetaTrace::analyze(s.t.data(), s.rel.data(), s.vel.data(),
                                                     static_cast<int>(s.t.size()), m);
    TEST_ASSERT_TRUE(r.max_follow_deg > 0.5);
    TEST_ASSERT_TRUE(r.i_follow > r.i_motion);
}

static void test_no_motion_and_empty(void) {
    const ThetaTrace::Move m = moveMinus45();
    std::vector<int32_t> t = {0, 1000, 2000};
    std::vector<float> rel = {0.0f, 0.001f, -0.002f};
    ThetaTrace::Result r = ThetaTrace::analyze(t.data(), rel.data(), nullptr, 3, m);
    TEST_ASSERT_EQUAL_INT(-1, r.i_motion);
    TEST_ASSERT_EQUAL_INT(-1, r.i_settle);
    ASSERT_NEAR(1e-6, 44.998, r.final_err_deg);
    r = ThetaTrace::analyze(nullptr, nullptr, nullptr, 0, m);
    TEST_ASSERT_EQUAL_INT(-1, r.i_motion);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_profile_time_triangle);
    RUN_TEST(test_ideal_trace_has_latency_and_no_error);
    RUN_TEST(test_ring_gives_overshoot_and_late_settle);
    RUN_TEST(test_lagging_axis_shows_following_error);
    RUN_TEST(test_no_motion_and_empty);
    return UNITY_END();
}
