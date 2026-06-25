#ifndef MQTT_BRIDGE_TYPES_H
#define MQTT_BRIDGE_TYPES_H

#include <stdint.h>

namespace MqttBridge {

struct PickFrame {
    float x_mm = 0.0f;
    float z_mm = 0.0f;
    float theta_deg = 0.0f;
    uint64_t t_epoch_us = 0;
    uint64_t eta_epoch_us = 0;
};

}  // namespace MqttBridge

#endif  // MQTT_BRIDGE_TYPES_H
