#ifndef MQTT_APP_H
#define MQTT_APP_H
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "mqtt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "mqtt_client.h"
#include "farmpulse_defs.h"

/* ===================== USER CONFIG ===================== */

#define WIFI_SSID "Airtel_Ajay"
#define WIFI_PASSWORD "Ajay@6952"

#define MQTT_BROKER_URI "mqtts://a37f344630334685ad06ea955597f845.s1.eu.hivemq.cloud"

#define MQTT_USERNAME "Prafull"
#define MQTT_PASSWORD "Praful1234"
#define MQTT_CLIENT_ID "gateway01"

#define EXPECTED_PASSWORD "AB1234"
#define EXPECTED_GID "MH-AMT-01"

#define SUBSCRIBE_TOPIC "server/MH-AMT-01"
#define PUBLISH_TOPIC "gateway/MH-AMT-01"
#define ERROR 1
#define NO_ERROR 0

#define GATEWAY_NODE_ID 0

// --- MQTT Function Code ---
typedef enum {
    REBOOT = 0,
    HANDSHAKE_FRAME = 1,
    GATEWAY_CONFIG = 2,
    NODE_THRESHOLD_CONFIG = 3,
    NODE_COUNT_CONFIG = 4,
    NODE_PERIODIC_INTERVAL = 5,
    GATEWAY_PANEL_STATUS = 6,
    NODE_PERIODIC_MESSAGE = 7,
    ALARM_FRAME = 8,
    OTA_FRAME = 9,
    LORA_CONFIG_FRAME = 10,
    NETWORK_CONFIG = 11,
    APN_CONFIG = 12,
    GATEWAY_FIRMWARE_VERSION = 13,
    SIM_INFORMATION = 14
} mqtt_param_id_t;

typedef struct
{
    uint8_t frame_version;
    char password[10];
    char gid[20];
    uint8_t nid;
    uint8_t function_code;
    uint8_t action_code;
    uint8_t data[200]; // GENERIC PAYLOAD

} sensor_frame_t;

extern esp_mqtt_client_handle_t mqtt_client;

void mqtt_system_start(void);

/* makes it easy for other modules (e.g. main) to publish LoRa
 * packets using the configured MQTT client */
void mqtt_publish_lora_data(const char *msg, int rssi, float snr);
extern bool get_next_lora_uplink_string(char *buffer);
extern bool get_next_lora_uplink_string(char *buffer);
extern void push_downlink_command_to_lora(int node_id, int msg_type, uint8_t *data, int data_len);
extern bool validate_mqtt_frame(char *input, sensor_frame_t *frame);
extern int fnMQTT_frame_parser(sensor_frame_t *);

#endif