#include <string.h>
#include <stdio.h>

#include "mqtt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "mqtt_client.h"

/* ===================== USER CONFIG ===================== */

#define WIFI_SSID       "Airtel_Ajay"
#define WIFI_PASSWORD   "Ajay@6952"

#define MQTT_BROKER_URI "mqtts://d115eaabb2c14526a27e587637759f52.s1.eu.hivemq.cloud:8883"

#define MQTT_USERNAME   "praful"
#define MQTT_PASSWORD   "Praful@123"
#define MQTT_CLIENT_ID  "san_node01"

#define SUBSCRIBE_TOPIC "farmpulse/gateway1/node01/command"
#define PUBLISH_TOPIC   "farmpulse/gateway1/node01/sensor"

/* ======================================================= */

static const char *TAG = "MQTT_APP";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

/* ===================== ROOT CA ===================== */

static const char root_ca_pem[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n";

/* ===================== MQTT PUBLISH TASK ===================== */

static void mqtt_publish_task(void *arg)
{
    while (1)
    {
        if (mqtt_connected && mqtt_client)
        {
            const char *payload =
            "{\"temperature\":29.5,\"humidity\":58,\"soil\":444}";

            int msg_id = esp_mqtt_client_publish(
                mqtt_client,
                PUBLISH_TOPIC,
                payload,
                0,
                1,
                0);

            ESP_LOGI(TAG, "Published msg_id=%d", msg_id);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ===================== MQTT EVENT HANDLER ===================== */

static void mqtt_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {

    case MQTT_EVENT_CONNECTED:

        ESP_LOGI(TAG, "MQTT CONNECTED");

        mqtt_connected = true;
        mqtt_client = event->client;

        esp_mqtt_client_subscribe(
            mqtt_client,
            SUBSCRIBE_TOPIC,
            1);

        ESP_LOGI(TAG, "Subscribed to: %s", SUBSCRIBE_TOPIC);

        xTaskCreate(
            mqtt_publish_task,
            "mqtt_pub",
            4096,
            NULL,
            5,
            NULL);

        break;


    case MQTT_EVENT_DATA:

        ESP_LOGI(TAG, "Message Received");

        printf("Topic: %.*s\n",
               event->topic_len,
               event->topic);

        printf("Data : %.*s\n",
               event->data_len,
               event->data);

        printf("------------\n");

        break;


    case MQTT_EVENT_DISCONNECTED:

        ESP_LOGW(TAG, "MQTT DISCONNECTED");

        mqtt_connected = false;

        break;

    default:
        break;
    }
}

/* ===================== MQTT START ===================== */

static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg =
    {
        .broker.address.uri = MQTT_BROKER_URI,

        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
        .credentials.client_id = MQTT_CLIENT_ID,

        .broker.verification.certificate = root_ca_pem,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL);

    esp_mqtt_client_start(mqtt_client);
}

/* ===================== WIFI EVENT HANDLER ===================== */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }

    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WiFi disconnected... retrying");
        esp_wifi_connect();
    }

    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "WiFi Connected. Starting MQTT...");
        mqtt_start();
    }
}

/* ===================== WIFI INIT ===================== */

static void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        NULL);

    esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_event_handler,
        NULL);

    wifi_config_t wifi_config =
    {
        .sta =
        {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

/* ===================== SYSTEM START ===================== */

void mqtt_system_start(void)
{
    wifi_init();
}