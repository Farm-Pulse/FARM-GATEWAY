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

#define WIFI_SSID "realme 5 Pro"
#define WIFI_PASSWORD "Praful@123"

#define MQTT_BROKER_URI "mqtts://9c7eb09fd9984a90b5c4419a1cbf7472.s1.eu.hivemq.cloud"

#define MQTT_USERNAME "Parag72"
#define MQTT_PASSWORD "Parag1357"
#define MQTT_CLIENT_ID "gateway01"

#define EXPECTED_PASSWORD "AB1234"
#define EXPECTED_GID "MH-AMT-01"

#define SUBSCRIBE_TOPIC "server/MH-AMT-01"

char topic[100] = "gateway/MH-AMT-01";

/* ======================================================= */

extern bool get_next_lora_uplink_string(char *buffer);
extern void push_downlink_command_to_lora(int node_id, int action_code);

static const char *TAG = "MQTT_GATEWAY";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

typedef struct
{
    char frame_version[5];
    char password[10];
    char gid[20];
    int nid;
    int function_code;
    int action_code;
    char data[200]; // GENERIC PAYLOAD

} sensor_frame_t;

bool validate_mqtt_frame(char *input, sensor_frame_t *frame)
{
    int len = strlen(input);

    // 1. Check start and end delimiter
    if (len < 5 || input[0] != '#' || input[len - 1] != '$')
    {
        printf("Invalid frame format (# or $ missing)\n");
        return false;
    }

    // 2. Copy to temp buffer (strtok modifies string)
    char temp[300];
    strcpy(temp, input);

    // Remove '#' and '$'
    memmove(temp, temp + 1, strlen(temp));
    char *end = strchr(temp, '$');
    if (end)
        *end = '\0';

    // 3. Extract tokens
    char *token;

    token = strtok(temp, " ");
    if (!token)
        return false;
    strcpy(frame->frame_version, token);

    token = strtok(NULL, " ");
    if (!token)
        return false;
    strcpy(frame->password, token);

    token = strtok(NULL, " ");
    if (!token)
        return false;
    strcpy(frame->gid, token);

    token = strtok(NULL, " ");
    if (!token)
        return false;
    frame->nid = atoi(token);

    token = strtok(NULL, " ");
    if (!token)
        return false;
    frame->function_code = atoi(token);

    token = strtok(NULL, " ");
    if (!token)
        return false;
    frame->action_code = atoi(token);

    token = strtok(NULL, "");
    if (!token)
        return false;
    strcpy(frame->data, token);

    // 4. Validate password
    if (strcmp(frame->password, EXPECTED_PASSWORD) != 0)
    {
        printf("Invalid Password\n");
        return false;
    }

    // 5. Validate GID
    if (strcmp(frame->gid, EXPECTED_GID) != 0)
    {
        printf("Invalid GID\n");
        return false;
    }

    return true;
}

/* ===================== SIMULATE LORA DATA ===================== */

void simulate_lora_receive(char *buffer)
{
    int node = (rand() % 3) + 1;

    char nid[5];
    sprintf(nid, "N%02d", node);

    float temp = 25 + (rand() % 10);
    int hum = 40 + (rand() % 30);
    int soil = 20 + (rand() % 40);
    int volt = 220 + (rand() % 10);
    float batt = 3.5 + ((float)(rand() % 10) / 10);

    sprintf(buffer,
            "#01 AB1234 MH-AMT-01 %s 02 01 20260316120000 %.1f %d %d %d %.1f$",
            nid, temp, hum, soil, volt, batt);
}

/* ===================== PARSE LORA FRAME ===================== */

void parse_lora_frame(char *input, sensor_frame_t *frame)
{
    char temp[300];
    strcpy(temp, input); // strtok modifies string

    char *token;

    // Remove starting '#'
    if (temp[0] == '#')
        memmove(temp, temp + 1, strlen(temp));

    // Remove ending '$'
    char *end = strchr(temp, '$');
    if (end)
        *end = '\0';

    token = strtok(temp, " ");
    if (token)
        strcpy(frame->frame_version, token);

    token = strtok(NULL, " ");
    if (token)
        strcpy(frame->password, token);

    token = strtok(NULL, " ");
    if (token)
        strcpy(frame->gid, token);

    token = strtok(NULL, " ");
    if (token)
        frame->nid = atoi(token);

    token = strtok(NULL, " ");
    if (token)
        frame->function_code = atoi(token);

    token = strtok(NULL, " ");
    if (token)
        frame->action_code = atoi(token);

    token = strtok(NULL, " ");
    if (token)
        strcpy(frame->data, token);
}

/* ===================== BUILD MQTT FRAME ===================== */

void build_mqtt_frame(sensor_frame_t *frame, char *payload)
{
    sprintf(payload,
            "#%s %s %s %d %d %d %s $",
            frame->frame_version,
            frame->password,
            frame->gid,
            frame->nid,
            frame->function_code,
            frame->action_code,
            frame->data);
}

/* ===================== MQTT PUBLISH TASK ===================== */

static void mqtt_publish_task(void *arg)
{
    while (1)
    {
        if (mqtt_connected && mqtt_client)
        {
            char lora_data[200];

            /* Simulate LoRa packet */
            // simulate_lora_receive(lora_data);

            // ESP_LOGI(TAG,"LoRa RX: %s",lora_data);
            if (get_next_lora_uplink_string(lora_data))
            {
                sensor_frame_t frame;

                parse_lora_frame(lora_data, &frame);

                char mqtt_payload[200];

                build_mqtt_frame(&frame, mqtt_payload);

                char topic[100];

                sprintf(topic,
                        "gateway/MH-AMT-01"); // frame.gid

                int msg_id = esp_mqtt_client_publish(
                    mqtt_client,
                    topic,
                    mqtt_payload,
                    0,
                    1,
                    0);

                ESP_LOGI(TAG, "MQTT Topic: %s", topic);
                ESP_LOGI(TAG, "MQTT Payload: %s", mqtt_payload);
                ESP_LOGI(TAG, "msg_id=%d", msg_id);
            }
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

        xTaskCreate(
            mqtt_publish_task,
            "mqtt_publish",
            4096,
            NULL,
            5,
            NULL);

        break;

    case MQTT_EVENT_DATA:
    {
        ESP_LOGI(TAG, "MQTT DATA RECEIVED");

        char rx_data[300];

        // Copy payload safely
        snprintf(rx_data, sizeof(rx_data), "%.*s",
                 event->data_len,
                 event->data);

        printf("Received: %s\n", rx_data);

        sensor_frame_t frame;

        if (validate_mqtt_frame(rx_data, &frame))
        {
            ESP_LOGI(TAG, "Frame VALID");

            printf("Version: %s\n", frame.frame_version);
            printf("GID: %s\n", frame.gid);
            printf("Data: %s\n", frame.data);
            if (frame.nid == 0)
            {
                printf("Node ID: Broadcast\n");
            }
            else if (frame.nid >= 1 && frame.nid <= 255)
            {
                printf("Node ID: %d\n", frame.nid);

                // --- NEW: STRICT PROTOCOL PARSING ---
                int sub_function = -1;
                int motor_action = -1;

                // Parse the string "1 0" into two distinct integers
                if (sscanf(frame.data, "%d %d", &sub_function, &motor_action) == 2)
                {

                    // Check if DA-[A] is 1 (Motor Operation)
                    if (sub_function == 1)
                    {
                        uint8_t lora_cmd = CMD_MOTOR_OFF; // Default to safe OFF

                        // Check DA-[B] for ON or OFF
                        if (motor_action == 1)
                        {
                            lora_cmd = CMD_MOTOR_ON;
                            ESP_LOGW(TAG, "Parsed Web Command: MOTOR ON -> Pushing to LoRa");
                        }
                        else if (motor_action == 0)
                        {
                            lora_cmd = CMD_MOTOR_OFF;
                            ESP_LOGW(TAG, "Parsed Web Command: MOTOR OFF -> Pushing to LoRa");
                        }

                        // Push the correctly translated command to your LoRa Queue
                        push_downlink_command_to_lora(frame.nid, lora_cmd);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Ignored: Unsupported Sub-Function Code [%d]", sub_function);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to parse data string: '%s'", frame.data);
                }
                // ------------------------------------

                // Send ACK back to the Cloud
                if (frame.function_code == 10)
                {
                    char ack_payload[200];
                    sprintf(ack_payload, "#1 AB1234 MH-AMT-XX 1 10 3 1 0 $");

                    int msg_id = esp_mqtt_client_publish(
                        mqtt_client,
                        topic,
                        ack_payload,
                        0, 1, 0);
                }
            }
            else
            {
                printf("invalid node id\n");
            }
        }
        else
        {
            ESP_LOGE(TAG, "Frame INVALID - Ignored");
        }

        break;
    }

    case MQTT_EVENT_DISCONNECTED:

        ESP_LOGW(TAG, "MQTT DISCONNECTED");

        mqtt_connected = false;

        break;

    default:
        break;
    }
}

/* ===================== ROOT CA ===================== */

static const char root_ca_pem[] =
    "-----BEGIN CERTIFICATE-----\n"
    "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\n"
    "-----END CERTIFICATE-----\n";

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

    ESP_LOGI(TAG, "Initializing MQTT Client...");

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (mqtt_client != NULL)
    {
        esp_mqtt_client_register_event(
            mqtt_client,
            ESP_EVENT_ANY_ID,
            mqtt_event_handler,
            NULL);

        // Start the client. It will automatically wait in the background for Wi-Fi.
        esp_mqtt_client_start(mqtt_client);
        ESP_LOGI(TAG, "MQTT Client Started successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "FATAL: Failed to initialize MQTT Client");
    }
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
        ESP_LOGW(TAG, "WiFi disconnected...retrying");
        esp_wifi_connect();
    }

    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "WiFi Connected");
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