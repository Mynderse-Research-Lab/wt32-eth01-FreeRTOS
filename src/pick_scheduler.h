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

// Queue one detection on the vision path. y_bat_mm is camera-relative and the
// call time is the capture time. Refused until a conveyor frame has set the
// clock, and from the console unless the cell is started.
// Omitted width, length, and height-from-belt (0) use the class-1 cell:
// 65 mm long, 18.3 mm across and 18.3 mm above the belt.
bool pickSchedulerInject(float x_across_mm, float y_bat_mm, float theta_deg,
                         float width_mm = 0.0f, float length_mm = 0.0f,
                         float height_from_belt_mm = 0.0f);

// Dry run, same inputs as pickSchedulerInject. Writes one line into out.
// False when the belt clock is not valid. Omitted size uses the class-1 cell.
bool pickSchedulerPlan(float x_across_mm, float y_bat_mm, float theta_deg,
                       char* out, size_t out_len, float width_mm = 0.0f,
                       float length_mm = 0.0f, float height_from_belt_mm = 0.0f);

// Pick or skip per cell angle (-90..85 deg, 5 deg steps; 0° along +Y) at the
// current belt speed for one battery class. Writes one line into out.
bool pickSchedulerCover(uint8_t battery_class, char* out, size_t out_len);

// pickcal valve|jaw|jawoff|cam. Writes one line into out.
bool pickSchedulerCalibrate(const char* args, char* out, size_t out_len);

#endif  // PICK_SCHEDULER_H

