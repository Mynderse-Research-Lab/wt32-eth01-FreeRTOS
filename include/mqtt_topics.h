#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

// Broker on Pi (ADSB-PI-base broker.env BROKER_IP; same /24 as ethernet_app_config.h).
#define MQTT_BROKER_URI_DEFAULT      "mqtt://192.168.1.5:1883"
#define MQTT_CLIENT_ID_PREFIX        "wt32-gantry-"
#define MQTT_GANTRY_ID_DEFAULT       "G01"

// Topic shape for per-gantry pick commands.
#define MQTT_TOPIC_PICK_FMT          "/gantry/%s/pick"
#define MQTT_TOPIC_GANTRY_STATUS     "/gantry/status"

// Pick payload (JSON on MQTT_TOPIC_PICK_FMT), all fields required:
//   {"t_epoch_us": <uint64>, "X": <float mm>, "Z": <float mm>,
//    "THETA": <float deg>, "ETA_epoch": <uint64>}
// ETA_epoch must be >= t_epoch_us. No conveyor topics are consumed.

#endif  // MQTT_TOPICS_H
