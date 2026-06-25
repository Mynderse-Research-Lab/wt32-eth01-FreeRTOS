#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#include "EthernetLink.h"
#include "MqttBridgeTypes.h"

#include "esp_event.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace MqttBridge {

class Bridge {
public:
    explicit Bridge(EthernetLink* ethernet_link);
    ~Bridge();

    bool start(const char* gantry_id);
    QueueHandle_t pickQueue() const;
    bool publishStatusJson(const char* json_payload);

private:
    static void handleMqttEvent(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
    void onMqttEvent(esp_mqtt_event_handle_t event);
    void onConnected();
    void onData(esp_mqtt_event_handle_t event);
    bool parsePickFrame(const char* payload, int len, PickFrame* out) const;
    bool topicMatchesPick(const char* topic, int topic_len) const;
    void buildPickTopic();

    EthernetLink* ethernet_link_ = nullptr;
    esp_mqtt_client_handle_t mqtt_ = nullptr;
    QueueHandle_t pick_queue_ = nullptr;
    char gantry_id_[16] = {0};
    char pick_topic_[64] = {0};
};

}  // namespace MqttBridge

#endif  // MQTT_BRIDGE_H
