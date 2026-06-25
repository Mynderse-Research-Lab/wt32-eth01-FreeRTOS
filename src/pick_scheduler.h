#ifndef PICK_SCHEDULER_H
#define PICK_SCHEDULER_H

#include "Gantry.h"

namespace MqttBridge {
class Bridge;
}

struct PickSchedulerTaskConfig {
    Gantry::Gantry* gantry;
    MqttBridge::Bridge* bridge;
};

void pickSchedulerTask(void* param);

#endif  // PICK_SCHEDULER_H
