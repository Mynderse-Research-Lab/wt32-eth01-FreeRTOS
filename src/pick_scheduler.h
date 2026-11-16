/**
 * @file pick_scheduler.h
 * @brief Dynamic intercept pick scheduler powered by OSI Layer-2 telemetry.
 */

#ifndef PICK_SCHEDULER_H
#define PICK_SCHEDULER_H

#include "CellNetL2.h"
#include "Gantry.h"

struct PickSchedulerTaskConfig {
  Gantry::Gantry *gantry;
  CellNet::CellNetL2Node *net_l2;
};

void pickSchedulerTask(void *param);

// Queue one detection on the vision path. y_bat_mm is camera-relative. The
// diagnostic stamps the live belt displacement as the capture sample. Width,
// length, and height-from-belt are required. Class 1–10 is a pick (AAA, AA, D,
// C, 9V, CR123A, 18650, 21700, 26650, 32650). Class 11–13 is a pack occupant
// (laptop, Li-ion pack, power-tool) and is not picked. item_id must be nonzero.
bool pickSchedulerInject(float x_across_mm, float y_bat_mm, float theta_deg,
                         uint8_t battery_class, uint32_t item_id, float width_mm, float length_mm,
                         float height_from_belt_mm);

// Dry run, same inputs as pickSchedulerInject. Writes one line into out.
// False when the belt clock is not valid or the vision contract rejects the sample.
bool pickSchedulerPlan(float x_across_mm, float y_bat_mm, float theta_deg,
                       uint8_t battery_class, uint32_t item_id, char* out, size_t out_len,
                       float width_mm, float length_mm, float height_from_belt_mm);

// Pick or skip per cell angle (-90..85 deg, 5 deg steps; 0° across, 90° along the belt) at the
// current belt speed for one battery class. Writes one line into out.
bool pickSchedulerCover(uint8_t battery_class, char* out, size_t out_len);

// pickcal valve|jaw|jawoff|cam. Writes one line into out.
bool pickSchedulerCalibrate(const char* args, char* out, size_t out_len);
// Dev belt clock: synthetic conveyor frames at mm_s (0 = off, real frames
// resume). Restarts the belt epoch, so queued parts are dropped.
bool pickSchedulerBeltSim(float mm_s, char* out, size_t out_len);
// Last strike's Z trace. "" = one summary line into out; "raw <start> <count>"
// logs the samples (t_us:z_mm, t from the trace start) and a range line into out.
bool pickSchedulerTrace(const char* args, char* out, size_t out_len);
// Grip timing marker on GPIO17 (borrows the TFT backlight): high from the
// strike's grip fire until the MCP write returns. Off restores the backlight.
bool pickSchedulerGripMark(bool on, char* out, size_t out_len);
// Vision receipt marker on the same pin: high when a CellNetL2 vision frame is
// stamped (receipt_us), low when the scheduler takes it. Off restores the
// backlight unless gripmark is still on.
bool pickSchedulerVisMark(bool on, char* out, size_t out_len);
// Belt locator after bring-up, all in joint X (the `move` frame): "near" / "far"
// save each belt edge (cv_belt_near_mm / cv_belt_far_mm), "cnvyr near" / "cnvyr
// far" the conveyor frame edges (cv_cnv_near_mm / cv_cnv_far_mm), "batt" the
// test battery (batt_x), "z" the joint Z with the open jaw tips on the belt.
// "apply" derives cv_belt_w_mm and col_x_min_mm (plus col_z_min_mm when z was
// recorded); "apply center" also sets cv_x_off_mm to the belt centre's
// end-effector X (theta 0).
bool pickSchedulerBeltCal(const char* args, char* out, size_t out_len);
// CellNet receive counters and EMAC drop counters (netstat); reset zeroes them.
bool pickSchedulerNetStats(bool reset, char* out, size_t out_len);

#endif  // PICK_SCHEDULER_H

