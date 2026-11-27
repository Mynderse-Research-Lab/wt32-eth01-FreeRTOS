/**
 * @file pick_intercept.h
 * @brief Frames, body, belt, clearance, and motion time for one pick. No FreeRTOS.
 *
 * Grouped as in docs/PICK_SCHEDULER_KINEMATICS.md: frames, body, belt odometer,
 * clearance, motion time, then neighbours and the part tracker. Vision Y is
 * camera-relative. Cell angles are in the vision frame, which is also the pose frame:
 * 0° is the long axis along +X (across the belt), 90° along +Y (downstream), positive
 * about +Z (up). The theta joint is pose - 90° (GantryKinematics). The jaws grip across
 * the cell.
 * Fingers go to grasp height early only when the X-projection misses both
 * finger lanes; otherwise Z waits and must still be at grasp height before contact.
 */

#ifndef PICK_INTERCEPT_H
#define PICK_INTERCEPT_H

#include "pick_intercept_vision.h"
#include "pick_intercept_belt.h"
#include "pick_intercept_clearance.h"
#include "pick_intercept_strike.h"
#include "pick_intercept_plan.h"
#include "pick_intercept_tracker.h"

#include "coordinated_grasp.h"

#endif  // PICK_INTERCEPT_H
