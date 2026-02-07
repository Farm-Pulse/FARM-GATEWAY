#include "mqtt_client.h"
#include "esp_log.h"

static const char *TAG = "MQTT_SERVICE";
static esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(client, "device/cmd", 1);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "RX topic=%.*s data=%.*s",
                 event->topic_len, event->topic,
                 event->data_len, event->data);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    default:
        break;
    }
}

void mqtt_service_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtts://your-broker:8883",
        .credentials.username = "device001",
        .credentials.authentication.password = "secret",
        // .broker.verification.certificate = root_ca_pem,
    };

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler,
                                   NULL);

    esp_mqtt_client_start(client);
}

void mqtt_publish(const char *topic, const char *payload)
{
    if (!client) return;
    esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
}
