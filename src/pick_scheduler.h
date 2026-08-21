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

#endif  // PICK_SCHEDULER_H

