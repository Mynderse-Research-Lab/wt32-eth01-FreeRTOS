/**
 * @file test_eip_axis_limits.cpp
 * @brief Linear axis limit-switch tests: a move that starts on a switch and
 *        heads away from it is not aborted by that switch (G-A13), while a
 *        switch ahead of the move still aborts it.
 */

#include "test_eip_axis_internal.h"

using namespace eip_axis_test;

namespace {

// Active, ready and CIP connected, with a limit warning and no stopped bit:
// the frame the Kinetix sends once an axis starts moving off a limit switch.
Bytes limitWarningFrame(int32_t position_puu, uint16_t warning_code) {
    Bytes fb = makeK5100Input154(position_puu, false, false, false, 0, warning_code, true);
    fb[9] |= 0x04 | 0x08 | 0x20;
    return fb;
}

bool stopMotionSet(eip::EipProcessImage& image) {
    Bytes cmd;
    TEST_ASSERT_TRUE(image.getCommand(cmd));
    return (cmd[1] & 0x04) != 0;
}

}  // namespace

// Z resting on A014 (joint max), seeking A015: the drive raises A014 only once
// the axis moves, so the warning is absent when the move starts. The move must
// run on instead of aborting as a fresh A014 trip (bench 2026-11-27).
void test_linear_escape_from_a014_at_joint_max_is_not_aborted(void) {
    eip::EipProcessImage image;
    image.setOnline(true);
    Gantry::EipLinearAxisConfig cfg;
    cfg.puu_per_mm = 1000.0;
    cfg.speed_ref_per_mm_s = 15.0;
    Gantry::GantryEipLinearAxis axis(image, cfg);
    axis.setJointMinWarningA015(true);  // Z: A015 = joint min, A014 = joint max
    TEST_ASSERT_TRUE(axis.begin());
    TEST_ASSERT_TRUE(axis.enable());
    armAxisForMove(axis, image, 130000);

    TEST_ASSERT_TRUE(axis.moveToMm(0.0f, 50.0f, 2000.0f, 2000.0f));
    pumpAbsoluteToRun(axis, image, 130000);
    TEST_ASSERT_TRUE(axis.isBusy());

    for (int i = 0; i < 20; ++i) {
        image.setFeedback(limitWarningFrame(130000 - 250 * i, 0x0014));
        axis.update();
        TEST_ASSERT_FALSE_MESSAGE(stopMotionSet(image), "no StopMotion while leaving A014");
    }
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "A014 behind the move must not abort it");
}

// X resting on A014 (joint min) and moving toward max: the same escape.
void test_linear_escape_from_a014_at_joint_min_is_not_aborted(void) {
    eip::EipProcessImage image;
    image.setOnline(true);
    Gantry::EipLinearAxisConfig cfg;
    cfg.puu_per_mm = 1000.0;
    cfg.speed_ref_per_mm_s = 15.0;
    Gantry::GantryEipLinearAxis axis(image, cfg);  // X mapping: A014 = joint min
    TEST_ASSERT_TRUE(axis.begin());
    TEST_ASSERT_TRUE(axis.enable());
    armAxisForMove(axis, image, 0);

    TEST_ASSERT_TRUE(axis.moveToMm(100.0f, 50.0f, 2000.0f, 2000.0f));
    pumpAbsoluteToRun(axis, image, 0);
    TEST_ASSERT_TRUE(axis.isBusy());

    for (int i = 0; i < 20; ++i) {
        image.setFeedback(limitWarningFrame(250 * i, 0x0014));
        axis.update();
        TEST_ASSERT_FALSE_MESSAGE(stopMotionSet(image), "no StopMotion while leaving A014");
    }
    TEST_ASSERT_TRUE_MESSAGE(axis.isBusy(), "A014 behind the move must not abort it");
}

// Z seeking A015 (joint min): A015 is ahead of the move, so it still aborts.
// Bring-up finds the switch this way.
void test_linear_seek_toward_a015_still_aborts_on_a015(void) {
    eip::EipProcessImage image;
    image.setOnline(true);
    Gantry::EipLinearAxisConfig cfg;
    cfg.puu_per_mm = 1000.0;
    cfg.speed_ref_per_mm_s = 15.0;
    Gantry::GantryEipLinearAxis axis(image, cfg);
    axis.setJointMinWarningA015(true);
    TEST_ASSERT_TRUE(axis.begin());
    TEST_ASSERT_TRUE(axis.enable());
    armAxisForMove(axis, image, 50000);

    TEST_ASSERT_TRUE(axis.moveToMm(-200.0f, 50.0f, 2000.0f, 2000.0f));
    pumpAbsoluteToRun(axis, image, 50000);
    TEST_ASSERT_TRUE(axis.isBusy());

    image.setFeedback(limitWarningFrame(1000, 0x0015));
    axis.update();
    TEST_ASSERT_TRUE_MESSAGE(stopMotionSet(image), "A015 ahead of the move must abort it");
}
