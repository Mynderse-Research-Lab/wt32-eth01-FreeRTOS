/**
 * @file test_pick_intercept_internal.h
 * @brief Shared cell, frame and limit helpers for the pick intercept host tests.
 */

#pragma once

#include "unity.h"

#include "pick_intercept.h"

#include <cmath>

namespace pick_intercept_test {

inline
PickIntercept::CellGeom cell18650() {
  return PickIntercept::CellGeom{65.0f, 18.3f, 18.3f};
}

inline
PickIntercept::CellGeom cell21700() {
  return PickIntercept::CellGeom{70.0f, 21.2f, 21.2f};
}

inline
PickIntercept::ConveyorFrame frameAt(int64_t gantry_us, uint32_t ts, float speed,
                                            float disp_m, uint8_t seq) {
  PickIntercept::ConveyorFrame frame;
  frame.gantry_us = gantry_us;
  frame.timestamp_us = ts;
  frame.speed_mm_s = speed;
  frame.displacement_m = disp_m;
  frame.sequence = seq;
  return frame;
}

inline
PickIntercept::MotionLimits limits(float speed = 2000.0f, float accel = 3000.0f,
                                          float z_cap = 1000.0f) {
  PickIntercept::MotionLimits m;
  m.speed = speed;
  m.accel = accel;
  m.decel = accel;
  m.theta_cap = 360.0f;
  m.theta_accel = 1800.0f;
  m.theta_decel = 1800.0f;
  m.z_cap = z_cap;
  m.theta_start_frac = 0.0f;
  return m;
}

inline
PickIntercept::StrikeTiming timingFor(PickIntercept::Clearance mode, float side_gap,
                                             float along) {
  PickIntercept::StrikeTiming t;
  t.mode = mode;
  t.side_gap_mm = side_gap;
  t.along_factor = along;
  t.jaw_mm_s = 210.0f;
  t.valve_s = 0.020f;
  t.guard_s = 0.015f;
  t.z_top_s = 0.050f;
  t.z_tail_s = 0.100f;
  return t;
}

}  // namespace

