/**
 * @file test_gantry_orchestration_internal.h
 * @brief Shared harness for the gantry orchestration host tests.
 */

#pragma once

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

namespace gantry_orch_test {

struct Harness {
    FakeLinearAxis* x = nullptr;
    FakeLinearAxis* z = nullptr;
    FakeRotaryAxis* t = nullptr;
    std::unique_ptr<Gantry::Gantry> g;
};

inline
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

inline
void tick(Gantry::Gantry& g, uint32_t ms = 10) {
    g.update();
    HostTest::advanceMs(ms);
}

inline
void completeLinear(Harness& h) {
    if (h.x->busy) {
        h.x->completeMove();
    }
    if (h.z->busy) {
        h.z->completeMove();
    }
}

inline
GantryError go(Harness& h, float x, float z, float th, uint32_t speed = 50,
               uint32_t accel = 3000) {
    return h.g->moveTo(JointConfig(x, z, th), speed, 30, accel, accel, 0, 0);
}


}  // namespace

