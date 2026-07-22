#include "MqttBridge.h"

#include "ethernet_app_config.h"
#include "mqtt_topics.h"

#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <cstdio>
#include <cstring>

namespace MqttBridge {

static const char* TAG = "MqttBridge";
static constexpr uint32_t PICK_QUEUE_DEPTH = 8;

Bridge::Bridge(EthernetLink* ethernet_link) : ethernet_link_(ethernet_link) {
    pick_queue_ = xQueueCreate(PICK_QUEUE_DEPTH, sizeof(PickFrame));
}

Bridge::~Bridge() {
    if (mqtt_ != nullptr) {
        esp_mqtt_client_stop(mqtt_);
        esp_mqtt_client_destroy(mqtt_);
        mqtt_ = nullptr;
    }
    if (pick_queue_ != nullptr) {
        vQueueDelete(pick_queue_);
        pick_queue_ = nullptr;
    }
}

bool Bridge::start(const char* gantry_id) {
    if (gantry_id == nullptr || gantry_id[0] == '\0') {
        ESP_LOGE(TAG, "Invalid gantry id");
        return false;
    }
    if (pick_queue_ == nullptr) {
        ESP_LOGE(TAG, "Queue allocation failed");
        return false;
    }
    if (ethernet_link_ == nullptr) {
        ESP_LOGE(TAG, "EthernetLink not provided");
        return false;
    }
    if (!ethernet_link_->start()) {
        ESP_LOGE(TAG, "Ethernet startup failed");
        return false;
    }
    if (!ethernet_link_->waitForUp(ETH_IP_WAIT_TIMEOUT_MS)) {
        ESP_LOGW(TAG,
                 "Ethernet PHY/IP not ready within %d ms — MQTT not started "
                 "(EIP/console unaffected; restore LAN8720 link and reboot to enable MQTT)",
                 ETH_IP_WAIT_TIMEOUT_MS);
        return false;
    }

    std::snprintf(gantry_id_, sizeof(gantry_id_), "%s", gantry_id);
    buildPickTopic();

    char client_id[32] = {0};
    std::snprintf(client_id, sizeof(client_id), "%s%s", MQTT_CLIENT_ID_PREFIX, gantry_id_);

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI_DEFAULT;
    mqtt_cfg.credentials.client_id = client_id;

    mqtt_ = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_ == nullptr) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return false;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        mqtt_, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID), &Bridge::handleMqttEvent, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_register_event failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_mqtt_client_start(mqtt_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_start failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "MQTT bridge started (id=%s, broker=%s)", gantry_id_, MQTT_BROKER_URI_DEFAULT);
    return true;
}

QueueHandle_t Bridge::pickQueue() const {
    return pick_queue_;
}

bool Bridge::publishStatusJson(const char* json_payload) {
    if (mqtt_ == nullptr || json_payload == nullptr) {
        return false;
    }
    int msg_id = esp_mqtt_client_publish(mqtt_, MQTT_TOPIC_GANTRY_STATUS, json_payload, 0, 0, 0);
    return msg_id >= 0;
}

void Bridge::handleMqttEvent(void* handler_args, esp_event_base_t, int32_t, void* event_data) {
    auto* self = static_cast<Bridge*>(handler_args);
    auto* event = static_cast<esp_mqtt_event_handle_t>(event_data);
    if (self == nullptr || event == nullptr) {
        return;
    }
    self->onMqttEvent(event);
}

void Bridge::onMqttEvent(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            onConnected();
            break;
        case MQTT_EVENT_DATA:
            onData(event);
            break;
        default:
            break;
    }
}

void Bridge::onConnected() {
    ESP_LOGI(TAG, "MQTT connected; subscribing to %s", pick_topic_);
    esp_mqtt_client_subscribe(mqtt_, pick_topic_, 1);
}

void Bridge::onData(esp_mqtt_event_handle_t event) {
    if (event->topic == nullptr || event->data == nullptr || event->data_len <= 0) {
        return;
    }

    if (topicMatchesPick(event->topic, event->topic_len)) {
        PickFrame frame = {};
        if (!parsePickFrame(event->data, event->data_len, &frame)) {
            ESP_LOGW(TAG, "Rejected pick frame payload");
            return;
        }

        if (xQueueSend(pick_queue_, &frame, 0) != pdTRUE) {
            PickFrame dropped = {};
            (void)xQueueReceive(pick_queue_, &dropped, 0);
            (void)xQueueSend(pick_queue_, &frame, 0);
            ESP_LOGW(TAG, "Pick queue overflow, dropped oldest frame");
        } else {
            ESP_LOGI(TAG,
                     "Pick frame queued: X=%.3f Z=%.3f THETA=%.3f t_epoch_us=%llu ETA_epoch=%llu",
                     frame.x_mm,
                     frame.z_mm,
                     frame.theta_deg,
                     static_cast<unsigned long long>(frame.t_epoch_us),
                     static_cast<unsigned long long>(frame.eta_epoch_us));
        }
        return;
    }

}

bool Bridge::parsePickFrame(const char* payload, int len, PickFrame* out) const {
    if (payload == nullptr || len <= 0 || out == nullptr) {
        return false;
    }

    cJSON* json = cJSON_ParseWithLength(payload, len);
    if (json == nullptr) {
        return false;
    }

    const cJSON* x = cJSON_GetObjectItemCaseSensitive(json, "X");
    const cJSON* z = cJSON_GetObjectItemCaseSensitive(json, "Z");
    const cJSON* theta = cJSON_GetObjectItemCaseSensitive(json, "THETA");
    const cJSON* t_epoch = cJSON_GetObjectItemCaseSensitive(json, "t_epoch_us");
    const cJSON* eta_epoch = cJSON_GetObjectItemCaseSensitive(json, "ETA_epoch");
    if (!cJSON_IsNumber(x) || !cJSON_IsNumber(z) || !cJSON_IsNumber(theta) ||
        !cJSON_IsNumber(t_epoch) || !cJSON_IsNumber(eta_epoch)) {
        cJSON_Delete(json);
        return false;
    }

    out->x_mm = static_cast<float>(x->valuedouble);
    out->z_mm = static_cast<float>(z->valuedouble);
    out->theta_deg = static_cast<float>(theta->valuedouble);
    out->t_epoch_us = static_cast<uint64_t>(t_epoch->valuedouble);
    out->eta_epoch_us = static_cast<uint64_t>(eta_epoch->valuedouble);
    if (out->eta_epoch_us < out->t_epoch_us) {
        cJSON_Delete(json);
        return false;
    }

    cJSON_Delete(json);
    return true;
}

bool Bridge::topicMatchesPick(const char* topic, int topic_len) const {
    const int expected_len = static_cast<int>(std::strlen(pick_topic_));
    return topic_len == expected_len && std::strncmp(topic, pick_topic_, expected_len) == 0;
}

void Bridge::buildPickTopic() {
    std::snprintf(pick_topic_, sizeof(pick_topic_), MQTT_TOPIC_PICK_FMT, gantry_id_);
}

}  // namespace MqttBridge
