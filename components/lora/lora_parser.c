#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "farmpulse_defs.h"
#include "mac_layer.h"
#include "network_layer.h"
#include "lora_parser.h"
#include "mqtt.h"

static const char *TAG = "GATEWAY_PARSER";

extern bool alive_table[256];
extern QueueHandle_t lora_to_web_queue;

// ====================================================================
// LORA RX HANDLER (UPLINK TO CLOUD)
// ====================================================================

void app_packet_handler(uint8_t src_id, uint8_t type, uint8_t *data, uint8_t len)
{
    uint8_t mqtt_payload[200] = {0};

    // Any incoming packet means the node is active
    if (src_id !=  0xFF)
    {
        if (!alive_table[src_id])
        {
            ESP_LOGW(TAG, ">>> NODE %d MARKED ALIVE <<<", src_id);
        }
        alive_table[src_id] = true;
    }

    // Parsing Incoming LoRa Frames
    switch (type) // MSG_TYPE
    {
    case PKT_TYPE_STATUS: // 0x02 (Heartbeat / Alive frame)
    {
        ESP_LOGI(TAG, "Status/Heartbeat packet received from Node %d (Len: %d)", src_id, len);
        if (len > 0)
        {
            switch (data[0])
            {
            case STATUS_NORMAL_HB:
                ESP_LOGI(TAG, "Node %d: Normal Heartbeat, Motor State: %d", src_id, (len >= 4) ? data[3] : -1);
                break;
            case STATUS_FIRST_BOOTUP:
                ESP_LOGI(TAG, "Node %d: First Bootup Announcement", src_id);
                break;
            case STATUS_ND_BEACON:
                ESP_LOGD(TAG, "Node %d: Neighbor Discovery Beacon", src_id);
                break;
            default:
                ESP_LOGI(TAG, "Node %d: Status subtype 0x%02X", src_id, data[0]);
                break;
            }
        }
    }
    break;
    case PKT_TYPE_CMD:
    {
        ESP_LOGI(TAG,
                 "Data packet received from Node %d",
                 src_id);

        // Process the LORA Payload based on the CMD_TYPE
        switch (data[0]) // CMD_TYPE
        {
        case CMD_TYPE_CONFIG:
        {
            switch (data[1]) // Direction
            {
            case _TYPE_SEND_CMD:
            {
                ESP_LOGI(TAG,
                         "LoRa Command Packet from Node %d",
                         src_id);

                switch (data[3]) // Function Code
                {
                case PARAM_LORA_MOTOR_CTRL:
                {
                    ESP_LOGI(TAG,
                             "[PARAM_LORA_MOTOR_CTRL] Command Packet from Node %d",
                             src_id);

                    switch (data[2]) // Action Type
                    {
                    case ACTION_GET:
                    {
                        ESP_LOGI(TAG,
                                 "LoRa Command Request from Node %d",
                                 src_id);

                        uint8_t motor_payload[30] = {0};
                        uint8_t index = 0;

                        // Read the motor status from GPIO or internal state
                        // motor_action = GPIO_READ(MOTOR_STATUS_PIN);

                        motor_payload[index++] = CMD_TYPE_CONFIG;
                        motor_payload[index++] = _TYPE_CMD_RESPONSE;
                        motor_payload[index++] = ACTION_DATA;
                        motor_payload[index++] = PARAM_LORA_MOTOR_CTRL;
                        motor_payload[index++] = motor_action;

                        network_send(
                            src_id,
                            PKT_TYPE_CMD,
                            motor_payload,
                            index);
                    }
                    break;

                    case ACTION_SET:
                    {
                        ESP_LOGI(TAG,
                                 "LoRa Command Response from Node %d",
                                 src_id);

                        motor_action = data[4]; // Motor Action (0=OFF, 1=ON)

                        // MOTOR GPIO WORK HERE
                        // GPIO_WRITE(MOTOR_CTRL_PIN, motor_action);

                        ESP_LOGI(TAG,
                                 "Node %d: FC 10 - LORA MOTOR CTRL SET Command - Sub-function 1",
                                 src_id, motor_action);
                    }
                    break;

                    default:
                    {
                        ESP_LOGW(
                            TAG,
                            "Unknown action type 0x%02X in command packet from Node %d",
                            data[2],
                            src_id);
                    }
                    break;
                    }
                }
                break;

                case PARAM_LORA_CONFIG:
                {
                    ESP_LOGI(TAG,
                             "[PARAM_LORA_CONFIG] Command Packet from Node %d",
                             src_id);
                }
                break;

                case PARAM_LORA_PANEL_STATUS:
                {
                    ESP_LOGI(TAG,
                             "[PARAM_LORA_PANEL_STATUS] Command Packet from Node %d",
                             src_id);

                    switch (data[2]) // Action Type
                    {
                    case ACTION_GET:
                    {
                        ESP_LOGI(TAG,
                                 "PARAM_LORA_PANEL_STATUS Get Command Request from Node %d",
                                 src_id);

                        uint8_t panel_status_payload[30] = {0};
                        uint8_t index = 0;

                        // Operation to do for Panel Status
                        // Read the motor status from GPIO or internal state
                        // motor_action = GPIO_READ(MOTOR_STATUS_PIN);
                        // Read the Sensor Data
                        // Read all the Alarm Fault
                        // Firmware Version

                        /*
                        Panel Status Data's List
                            DA- [B]: Node ID
                            DA- [C]: Node Firmware Version (Hex Format)
                            DA- [D]: Node Region (first 4 –bits) and Channel No (second 4 –bits) (Region and Frequency) (Hex Format)
                            DA- [E]: Node Network ID (Range 1000 - 65535)
                            DA- [F]: Node UUID/MAC ID (Hex Format) (8 bytes)
                            DA- [G]: Power on interruption count - this denotes the number of time device was reboot.
                            DA- [H]: Motor Status (1-ON, 0-OFF)
                            DA- [I]: Voltage Value line 1
                            DA- [J]: Voltage Value line 2
                            DA- [K]: Voltage Value line 3
                            DA- [L]: Temperature Sensor Value- Denotes temperature of a Node.
                            DA- [M]: Humidity sensor Value- Denotes sensor value of a Node.
                            DA- [N]: Soil Sensor Value- Denotes sensor value of a Node.
                            DA- [O]: Reserved
                            DA- [P]: Alarm Register 1 (2_bytes) (1-bit for each alarm)
                            DA- [Q]: Alarm Register 2 (2_bytes) (1-bit for each alarm)
                            DA- [R]: Neighbour Count
                            DA- [S]: 1st Neighbour ID
                            DA- [T]: 2nd Neighbour ID
                            DA- [U]: 3rd Neighbour ID
                            :
                            :
                            :
                            :
                            DA- [n]: 20th Neighbour ID
                        */

                        panel_status_payload[index++] = CMD_TYPE_CONFIG;
                        panel_status_payload[index++] = _TYPE_CMD_RESPONSE;
                        panel_status_payload[index++] = ACTION_DATA;
                        panel_status_payload[index++] = PARAM_LORA_PANEL_STATUS;
                        panel_status_payload[index++] = motor_action;

                        network_send(
                            src_id,
                            PKT_TYPE_CMD,
                            panel_status_payload,
                            index);
                    }
                    break;

                    default:
                    {
                        ESP_LOGW(
                            TAG,
                            "Unknown action type 0x%02X in command packet from Node %d",
                            data[2],
                            src_id);
                    }
                    break;
                    }
                }
                break;

                default:
                {
                    ESP_LOGW(
                        TAG,
                        "Unknown function code 0x%02X in command packet from Node %d",
                        data[3],
                        src_id);
                }
                break;
                }
            }
            break;

            case _TYPE_CMD_RESPONSE:
            {
                ESP_LOGI(TAG,
                         "LoRa Data Packet from Node %d",
                         src_id);

                switch (data[3])
                {
                case PARAM_LORA_MOTOR_CTRL:
                {
                    ESP_LOGI(TAG,
                             "Motor Command Packet from Node %d",
                             src_id);

                    switch (data[2]) // Action Type
                    {
                    case ACTION_DATA:
                    {
                        ESP_LOGI(
                            TAG,
                            "LoRa Command Request from Node %d",
                            src_id);

                        // uint8_t motor_payload[30];
                        uint8_t motor_status = data[4]; // Motor Action (0=OFF, 1=ON)

                        // MQTT CLOUD PUBLISH
                        sensor_frame_t data_frame;
                        data_frame.frame_version = _FV_01;
                        memset(data_frame.password, '\0', sizeof(data_frame.password));
                        strcpy(data_frame.password, "AB1234");
                        memset(data_frame.gid, '\0', sizeof(data_frame.gid));
                        strcpy(data_frame.gid, "MH-AMT-01");
                        data_frame.nid = src_id;
                        data_frame.function_code = LORA_CONFIG_FRAME;
                        data_frame.action_code = ACTION_DATA;
                        data_frame.data[0] = PARAM_LORA_MOTOR_CTRL;
                        data_frame.data[1] = motor_status;

                        memset(mqtt_payload, 0, sizeof(mqtt_payload));
                        sprintf((char *)mqtt_payload, "#%d %s %s %d %d %d %d %d $",
                                data_frame.frame_version,
                                data_frame.password,
                                data_frame.gid,
                                data_frame.nid,
                                data_frame.function_code,
                                data_frame.action_code,
                                data_frame.data[0],
                                data_frame.data[1]);
                        
                        if (xQueueSend(lora_to_web_queue, mqtt_payload, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            ESP_LOGI(TAG, "Successfully queued MQTT message: %s", mqtt_payload);
                        }
                        else
                        {
                            ESP_LOGE(TAG, "lora_to_web_queue is FULL! Dropped message: %s", mqtt_payload);
                        }
                    }
                    break;

                    default:

                        ESP_LOGW(
                            TAG,
                            "Unknown action type 0x%02X in command response packet from Node %d",
                            data[2],
                            src_id);
                        break;
                    }
                }
                break;

                case PARAM_LORA_PANEL_STATUS:
                {
                    ESP_LOGI(TAG,
                             "[PARAM_LORA_PANEL_STATUS] Command Packet from Node %d",
                             src_id);

                    switch (data[2]) // Action Type
                    {
                    case ACTION_DATA:
                    {
                        ESP_LOGI(
                            TAG,
                            "PARAM_LORA_PANEL_STATUS Data Request from Node %d",
                            src_id);

                        // uint8_t motor_payload[30];
                        uint8_t motor_status = data[4]; // Motor Action (0=OFF, 1=ON)

                        // MQTT CLOUD PUBLISH
                        sensor_frame_t data_frame;
                        data_frame.frame_version = _FV_01;
                        memset(data_frame.password, '\0', sizeof(data_frame.password));
                        strcpy(data_frame.password, "AB1234");
                        memset(data_frame.gid, '\0', sizeof(data_frame.gid));
                        strcpy(data_frame.gid, "MH-AMT-01");
                        data_frame.nid = src_id;
                        data_frame.function_code = LORA_CONFIG_FRAME;
                        data_frame.action_code = ACTION_DATA;
                        data_frame.data[0] = PARAM_LORA_PANEL_STATUS;
                        data_frame.data[1] = motor_status;

                        memset(mqtt_payload, 0, sizeof(mqtt_payload));
                        sprintf((char *)mqtt_payload, "#%d %s %s %d %d %d %d %d $",
                                data_frame.frame_version,
                                data_frame.password,
                                data_frame.gid,
                                data_frame.nid,
                                data_frame.function_code,
                                data_frame.action_code,
                                data_frame.data[0],
                                data_frame.data[1]);

                        // int stat = esp_mqtt_client_publish(mqtt_client, PUBLISH_TOPIC, (char *)mqtt_payload, strlen((char *) mqtt_payload), 1, 0);
                        // if (stat == 0)
                        // {
                        //     ESP_LOGI(TAG, "[PARAM_LORA_MOTOR_CTRL] Published MQTT message: %s", mqtt_payload);
                        // }
                        // else
                        // {
                        //     ESP_LOGE(TAG, "[PARAM_LORA_MOTOR_CTRL] Failed to publish MQTT message: %s", mqtt_payload);
                        // }

                        if (xQueueSend(lora_to_web_queue, mqtt_payload, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            ESP_LOGI(TAG, "Successfully queued MQTT message: %s", mqtt_payload);
                        }
                        else
                        {
                            ESP_LOGE(TAG, "lora_to_web_queue is FULL! Dropped message: %s", mqtt_payload);
                        }
                    }
                    break;

                    default:

                        ESP_LOGW(
                            TAG,
                            "Unknown action type 0x%02X in command response packet from Node %d",
                            data[2],
                            src_id);
                        break;
                    }
                }
                break;

                default:

                    ESP_LOGW(
                        TAG,
                        "Unknown function code 0x%02X in command response packet from Node %d",
                        data[3],
                        src_id);
                    break;
                }
                break;
            }
            default:
            {
                ESP_LOGW(
                    TAG,
                    "Unknown direction 0x%02X in data packet from Node %d",
                    data[1],
                    src_id);
            }
            break;
            }
        }
        break;

        case CMD_TYPE_OTA_REQ:
        {
            /* OTA request handling */
        }
        break;

        case CMD_TYPE_OTA_DATA:
        {
            /* OTA data handling */
        }
        break;

        default:

            ESP_LOGW(
                TAG,
                "Unknown command type 0x%02X in data packet from Node %d",
                data[0],
                src_id);
            break;
        }
    }
    break;

    case PKT_TYPE_DATA:
    {
        ESP_LOGI(TAG, "Telemetry Data packet received from Node %d", src_id);
    }
    break;

    default:
    {
        ESP_LOGW(TAG,
                 "Unknown packet type 0x%02X from Node %d",
                 type,
                 src_id);
    }
        return;
    }

    // if (type == PKT_TYPE_DATA && len == sizeof(sensor_data_t))
    // {
    //     sensor_data_t *s = (sensor_data_t *)data;

    //     /* Update Alive Table */
    //     if (!alive_table[src_id])
    //     {
    //         ESP_LOGW(TAG,
    //                  ">>> NODE %d ADDED TO ALIVE TABLE <<<",
    //                  src_id);

    //         alive_table[src_id] = true;
    //     }

    //     /* Format data for MQTT */
    //     char lora_str[256];

    //     sprintf(
    //         lora_str,
    //         "#01 AB1234 MH-AMT-01 %d 02 01 "
    //         "V_RYB:%d,%d,%d "
    //         "I_RYB:%d,%d,%d "
    //         "PWR:%ld "
    //         "MTR:%d$",
    //         src_id,
    //         s->voltage_R,
    //         s->voltage_Y,
    //         s->voltage_B,
    //         s->current_R,
    //         s->current_Y,
    //         s->current_B,
    //         s->power_active,
    //         s->motor_status
    //     );

    //         /* Push formatted string to MQTT task */
    //         xQueueSend(
    //             lora_to_web_queue,
    //             lora_str,
    //             pdMS_TO_TICKS(100)
    //         );

    //         ESP_LOGI(TAG,
    //                  "Data from Node %d pushed to MQTT Queue.",
    //                  src_id);
    //     }
}