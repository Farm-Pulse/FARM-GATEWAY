
#include "mqtt.h"

static const char *TAG = "MQTT_GATEWAY";

esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static TaskHandle_t publish_task_handle = NULL;



// static void handle_fc10(const sensor_frame_t *frame)
// {
//     ESP_LOGI(TAG,
//              "Node %d: FC 10 - LORA Configuration",
//              frame->nid);

//     /*
//      * Pass the original FC and DATA
//      * to the LoRa command handler.
//      */
//     push_downlink_command_to_lora(
//     frame->nid,
//     frame->function_code,
//     frame->action_code,
//     frame->data);
// }


bool validate_mqtt_frame(char *input, sensor_frame_t *frame)
{
    if (input == NULL || frame == NULL)
    {
        return false;
    }

    size_t len = strlen(input);

    /* 1. Check start and end delimiter */
    if (len < 5 ||
        input[0] != '#' ||
        input[len - 1] != '$')
    {
        printf("Invalid frame format (# or $ missing)\n");
        return false;
    }

    /* Prevent temp buffer overflow */
    if (len >= 300)
    {
        printf("MQTT frame too long\n");
        return false;
    }

    /* 2. Copy into temporary buffer */
    char temp[300];
    strcpy(temp, input);

    /* Remove leading '#' */
    memmove(temp, temp + 1, strlen(temp));

    /* Remove trailing '$' */
    char *end = strrchr(temp, '$');

    if (end != NULL)
    {
        *end = '\0';
    }

    /* Clear output structure */
    memset(frame, 0, sizeof(sensor_frame_t));

    /* 3. Extract tokens */
    char *token;

    /* Frame Version */
    token = strtok(temp, " ");

    if (token == NULL)
    {
        return false;
    }

    frame->frame_version = (uint8_t)atoi(token);


    /* Password */
    token = strtok(NULL, " ");

    if (token == NULL)
    {
        return false;
    }

    strncpy(frame->password,
            token,
            sizeof(frame->password) - 1);

    frame->password[sizeof(frame->password) - 1] = '\0';


    /* GID */
    token = strtok(NULL, " ");

    if (token == NULL)
    {
        return false;
    }

    strncpy(frame->gid,
            token,
            sizeof(frame->gid) - 1);

    frame->gid[sizeof(frame->gid) - 1] = '\0';


    /* Node ID */
    token = strtok(NULL, " ");

    if (token == NULL)
    {
        return false;
    }

    frame->nid = (uint8_t)atoi(token);


    /* Function Code */
    token = strtok(NULL, " ");

    if (token == NULL)
    {
        return false;
    }

    frame->function_code = (uint8_t)atoi(token);


    /* Action Code */
    token = strtok(NULL, " ");

    if (token == NULL)
    {
        return false;
    }

    frame->action_code = (uint8_t)atoi(token);


    /* Data / remaining payload */
    token = strtok(NULL, "");

    if (token != NULL)
    {
        strncpy((char *)frame->data,
                token,
                sizeof(frame->data) - 1);

        frame->data[sizeof(frame->data) - 1] = '\0';
    }


    /* 4. Validate password */
    if (strcmp(frame->password, EXPECTED_PASSWORD) != 0)
    {
        printf("Invalid Password\n");
        return false;
    }


    /* 5. Validate GID */
    if (strcmp(frame->gid, EXPECTED_GID) != 0)
    {
        printf("Invalid GID\n");
        return false;
    }


    printf("Valid MQTT Frame\n");

    return true;
}

/* ===================== MQTT FRAME PARSER ===================== */
int fnMQTT_frame_parser(sensor_frame_t *frame)
{
    // if (!validate_mqtt_frame(frame->data, frame))
    // {
    //     return ERROR;
    // }
    uint8_t sub_function_code = 0;
    uint8_t data_len = strlen((char *)frame->data);
    uint8_t mqtt_payload[200] = {0};
    uint8_t status = ERROR;
    // uint8_t *temp_ptr;

    switch (frame->function_code)
    {
        case REBOOT:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 0 - REBOOT Command",
                         frame->nid);                                                                                                                                                                                                                                                                   
            }
            break;
        case LORA_CONFIG_FRAME:
            {
                printf("in LORA_CONFIG_FRAME\n");
                // handle_fc10(frame);

                // Check if data length is sufficient to extract sub-function code
                if (data_len >= 2)  // Assuming sub-function code is 1 byte and there's at least 1 byte of data
                {
                    
                    sub_function_code = (uint8_t)atoi((char *)frame->data); // Assuming the first byte of data indicates the sub-function
                    printf("sub_function_code=%d",sub_function_code);
                }
                else
                {
                    ESP_LOGW(TAG,
                             "Node %d: FC 10 - LORA CONFIG Command - Insufficient data length for sub-function code",
                             frame->nid);
                    return ERROR;
                }
                
                switch (sub_function_code)
                {
                    printf("After sub_function_code%d\n",frame->data[0]);
                    case PARAM_LORA_MOTOR_CTRL:
                    {
                        // Note: This frame is only for LORA Nodes not for GATEWAY. The GATEWAY will only receive this frame from the LoRa Nodes.
                        status = NO_ERROR;
                        uint8_t msg_type = PKT_TYPE_CMD;
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_LORA_MOTOR_CTRL]  Command - Sub-function 1",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA MOTOR CTRL SET Command",
                                    frame->nid);
                            if (frame->nid != GATEWAY_NODE_ID) 
                            {
                                ESP_LOGI(TAG,
                                        "Node %d: FC 10 - LORA MOTOR CTRL SET Command - Valid Node ID",
                                        frame->nid);
                                int temp_sub_func = 0;
                                int temp_motor_val = 0;
                                uint8_t motor_status = 0;

                                // #$ convert lora 
                                //uint8_t motor_status = (uint8_t)atoi(strchr((char *)frame->data, ' ') + 1); // Assuming the second byte of data indicates the motor action
                                // 1. Safely extract the motor status (No strchr needed)
                                // If frame->data is "1", atoi safely returns 1. If it's garbage, it returns 0.

                                // Parse both numbers: "1 0" -> sub_func = 1, motor_val = 0
                                if (sscanf((char *)frame->data, "%d %d", &temp_sub_func, &temp_motor_val) == 2) {
                                    motor_status = (temp_motor_val >= 1) ? 1 : 0;
                                } else {
                                    // Fallback if only a single digit was sent
                                    motor_status = (uint8_t)atoi((char *)frame->data);
                                    motor_status = (motor_status >= 1) ? 1 : 0;
                                }

                                ESP_LOGI(TAG, "Node %d: Parsed Motor SET Command -> State: %d (%s)", 
                                        frame->nid, motor_status, motor_status ? "ON" : "OFF");
                                
                                uint8_t lora_payload[5] = {0};
                                uint8_t index = 0;
                                lora_payload[index++] = CMD_TYPE_CONFIG; //0x05
                                lora_payload[index++] = _TYPE_SEND_CMD; //0x01
                                lora_payload[index++] = ACTION_SET; //0x02
                                lora_payload[index++] = PARAM_LORA_MOTOR_CTRL; //0x01
                                lora_payload[index++] = motor_status; //0x01 / 0x00

                                // 3. Push to the LoRa Task safely
                                push_downlink_command_to_lora(frame->nid, PKT_TYPE_CMD, lora_payload, index);
                            }
                            else
                            {
                                ESP_LOGW(TAG,
                                        "Node %d: FC 10 - LORA MOTOR CTRL SET Command - Invalid Node ID",
                                        frame->nid);
                                status = ERROR;

                                memset(mqtt_payload, 0, sizeof(mqtt_payload));
                                sprintf((char *)mqtt_payload, "#%d %s %s %d %d %d %d %d $",
                                        frame->frame_version,
                                        frame->password,
                                        frame->gid,
                                        frame->nid,
                                        frame->function_code,
                                        frame->action_code,
                                        PARAM_LORA_MOTOR_CTRL,
                                        status);
                                int stat = esp_mqtt_client_publish(mqtt_client, PUBLISH_TOPIC, (char *) mqtt_payload, strlen((char *)mqtt_payload), 1, 0);        
                                
                                if (stat >= 0)
                                {
                                    ESP_LOGI(TAG, "[PARAM_LORA_MOTOR_CTRL] Published MQTT message: %s", mqtt_payload);
                                }
                                else
                                {
                                    ESP_LOGE(TAG, "[PARAM_LORA_MOTOR_CTRL] Failed to publish MQTT message: %s", mqtt_payload);
                                }
                            }
                            
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA MOTOR CTRL GET Command",
                                    frame->nid);
                            if (frame->nid != GATEWAY_NODE_ID) 
                            {
                                ESP_LOGI(TAG,
                                        "Node %d: FC 10 - LORA MOTOR CTRL GET Command - Valid Node ID",
                                        frame->nid);

                                // Convert to LoRa Frame 
                                uint8_t lora_payload[5] = {0};
                                uint8_t index = 0;
                                lora_payload[index++] = CMD_TYPE_CONFIG;
                                lora_payload[index++] = _TYPE_SEND_CMD;
                                lora_payload[index++] = ACTION_GET;
                                lora_payload[index++] = PARAM_LORA_MOTOR_CTRL;

                                push_downlink_command_to_lora(
                                frame->nid,
                                msg_type,
                                lora_payload,
                                index);
                            }
                            else
                            {
                                ESP_LOGW(TAG,
                                        "Node %d: FC 10 - LORA MOTOR CTRL GET Command - Invalid Node ID",
                                        frame->nid);
                                status = ERROR;
                            
                                memset(mqtt_payload, 0, sizeof(mqtt_payload));
                                sprintf((char *)mqtt_payload, "#%d %s %s %d %d %d %d %d $",
                                        frame->frame_version,
                                        frame->password,
                                        frame->gid,
                                        frame->nid,
                                        frame->function_code,
                                        frame->action_code,
                                        PARAM_LORA_MOTOR_CTRL,
                                        status);
                                int stat = esp_mqtt_client_publish(mqtt_client, PUBLISH_TOPIC, (char *)mqtt_payload, strlen((char *)mqtt_payload), 1, 0);        
                                
                                if (stat >= 0)
                                {
                                    ESP_LOGI(TAG, "[PARAM_LORA_MOTOR_CTRL] Published MQTT message: %s", mqtt_payload);
                                }
                                else
                                {
                                    ESP_LOGE(TAG, "[PARAM_LORA_MOTOR_CTRL] Failed to publish MQTT message: %s", mqtt_payload);
                                }
                            }
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - LORA MOTOR CTRL Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                            
                            status = ERROR;
                            memset(mqtt_payload, 0, sizeof(mqtt_payload));
                            sprintf((char *)mqtt_payload, "#%d %s %s %d %d %d %d %d $",
                                    frame->frame_version,
                                    frame->password,
                                    frame->gid,
                                    frame->nid,
                                    frame->function_code,
                                    frame->action_code,
                                    PARAM_LORA_MOTOR_CTRL,
                                    status);
                            int stat = esp_mqtt_client_publish(mqtt_client, PUBLISH_TOPIC, (char *)mqtt_payload, strlen((char *)mqtt_payload), 1, 0);        
                            
                            
                            if (stat >= 0)
                            {
                                ESP_LOGI(TAG, "[PARAM_LORA_MOTOR_CTRL] Published MQTT message: %s", mqtt_payload);
                            }
                            else
                            {
                                ESP_LOGE(TAG, "[PARAM_LORA_MOTOR_CTRL] Failed to publish MQTT message: %s", mqtt_payload);
                            }
                            // build_mqtt_frame(&ack_frame, mqtt_payload);
                        }
                        return status;
                    }
                    break;
                    case PARAM_LORA_CONFIG:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_LORA_CONFIG]  Command - Sub-function 2",
                                    frame->nid);
                        
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA CONFIG SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA CONFIG GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - LORA CONFIG Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_LORA_TX_PWR_CONFIG:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_LORA_TX_PWR_CONFIG]  Command - Sub-function 3",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA TX PWR CONFIG SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA TX PWR CONFIG GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - LORA TX PWR CONFIG Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_FV:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_FV]  Command - Sub-function 4",
                                    frame->nid);

                        if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA FRAME VERSION GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - LORA FRAME VERSION Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_SENSOR_DATA:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_SENSOR_DATA]  Command - Sub-function 5",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA TX PWR CONFIG SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA TX PWR CONFIG GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - LORA TX PWR CONFIG Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_LORA_PANEL_STATUS:
                    {
                        uint8_t msg_type = PKT_TYPE_CMD;
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_LORA_PANEL_STATUS] Command - Sub-function 6",
                                    frame->nid);

                        if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA PANEL STATUS GET Command",
                                    frame->nid);
                            if (frame->nid != GATEWAY_NODE_ID) 
                            {
                                ESP_LOGI(TAG,
                                        "Node %d: FC 10 - LORA MOTOR CTRL GET Command - Valid Node ID",
                                        frame->nid);

                                // Convert to LoRa Frame 
                                uint8_t lora_payload[5] = {0};
                                uint8_t index = 0;
                                lora_payload[index++] = CMD_TYPE_CONFIG;
                                lora_payload[index++] = _TYPE_SEND_CMD;
                                lora_payload[index++] = ACTION_GET;
                                lora_payload[index++] = PARAM_LORA_PANEL_STATUS;

                                push_downlink_command_to_lora(
                                frame->nid,
                                msg_type,
                                lora_payload,
                                index);
                            }
                            else
                            {
                                ESP_LOGW(TAG,
                                        "Node %d: FC 10 - LORA PANEL STATUS GET Command - Invalid Node ID",
                                        frame->nid);
                                status = ERROR;
                            
                                memset(mqtt_payload, 0, sizeof(mqtt_payload));
                                sprintf((char *)mqtt_payload, "#%d %s %s %d %d %d %d %d $",
                                        frame->frame_version,
                                        frame->password,
                                        frame->gid,
                                        frame->nid,
                                        frame->function_code,
                                        frame->action_code,
                                        PARAM_LORA_PANEL_STATUS,
                                        status);
                                int stat = esp_mqtt_client_publish(mqtt_client, PUBLISH_TOPIC, (char *)mqtt_payload, strlen((char *)mqtt_payload), 1, 0);        
                                
                                if (stat >= 0)
                                {
                                    ESP_LOGI(TAG, "[PARAM_LORA_MOTOR_CTRL] Published MQTT message: %s", mqtt_payload);
                                }
                                else
                                {
                                    ESP_LOGE(TAG, "[PARAM_LORA_MOTOR_CTRL] Failed to publish MQTT message: %s", mqtt_payload);
                                }
                            }
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - LORA PANEL STATUS Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                            status = ERROR;
                            
                            memset(mqtt_payload, 0, sizeof(mqtt_payload));
                            sprintf((char *)mqtt_payload, "#%d %s %s %d %d %d %d %d $",
                                    frame->frame_version,
                                    frame->password,
                                    frame->gid,
                                    frame->nid,
                                    frame->function_code,
                                    frame->action_code,
                                    PARAM_LORA_PANEL_STATUS,
                                    status);
                            int stat = esp_mqtt_client_publish(mqtt_client, PUBLISH_TOPIC, (char *)mqtt_payload, strlen((char *)mqtt_payload), 1, 0);        
                            
                            if (stat >= 0)
                            {
                                ESP_LOGI(TAG, "[PARAM_LORA_MOTOR_CTRL] Published MQTT message: %s", mqtt_payload);
                            }
                            else
                            {
                                ESP_LOGE(TAG, "[PARAM_LORA_MOTOR_CTRL] Failed to publish MQTT message: %s", mqtt_payload);
                            }
                        }
                        
                        return status;
                    }
                    break;
                    case PARAM_LORA_UUID:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_LORA_UUID] Command - Sub-function 7",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA UUID SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - LORA UUID GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - LORA UUID Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_ALARM_MASK:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_ALARM_MASK] Command - Sub-function 8",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_ALARM_MASK] SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_ALARM_MASK] GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - [PARAM_ALARM_MASK] Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_SIGNATURE:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_SIGNATURE] Command - Sub-function 9",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_SIGNATURE] SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_SIGNATURE] GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - [PARAM_SIGNATURE] Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_DEVICE_ID:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_DEVICE_ID] Command - Sub-function 10",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_DEVICE_ID] SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_DEVICE_ID] GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - [PARAM_DEVICE_ID] Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_RSSI_THRESH:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_RSSI_THRESH] Command - Sub-function 11",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_RSSI_THRESH] SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_RSSI_THRESH] GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - [PARAM_RSSI_THRESH] Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_HB_INTERVAL:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_HB_INTERVAL] Command - Sub-function 12",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_HB_INTERVAL] SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_HB_INTERVAL] GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - [PARAM_HB_INTERVAL] Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_METER_DATA_REQ:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_METER_DATA_REQ] Command - Sub-function 13",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_METER_DATA_REQ] SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_METER_DATA_REQ] GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - [PARAM_METER_DATA_REQ] Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    case PARAM_REBOOT:
                    {
                        ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_REBOOT] Command - Sub-function 14",
                                    frame->nid);
                        if (frame->action_code == ACTION_SET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_REBOOT] SET Command",
                                    frame->nid);
                        
                        }
                        else if (frame->action_code == ACTION_GET)
                        {
                            ESP_LOGI(TAG,
                                    "Node %d: FC 10 - [PARAM_REBOOT] GET Command",
                                    frame->nid);
                        }
                        else
                        {
                            ESP_LOGW(TAG,
                                    "Node %d: FC 10 - [PARAM_REBOOT] Unknown Action Code %d",
                                    frame->nid,
                                    frame->action_code);
                        }
                    }
                    break;
                    default:
                    {

                        ESP_LOGW(TAG,
                            "Node %d: FC 10 - [LORA_CONFIG_FRAME] - Unknown Sub-function %d",
                            frame->nid,
                            sub_function_code);
                    }
                    break;
                }
            }
            break;
        case HANDSHAKE_FRAME:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 1 - HANDSHAKE Command",
                         frame->nid);
            }
            break;
        case GATEWAY_CONFIG:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 2 - GATEWAY CONFIG Command",
                         frame->nid);
            }
            break;
        case NODE_THRESHOLD_CONFIG:
            {
               ESP_LOGI(TAG,
                         "Node %d: FC 3 - NODE THRESHOLD CONFIG Command",
                         frame->nid);
            }
            break;
        case NODE_COUNT_CONFIG:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 4 - NODE COUNT CONFIG Command",
                         frame->nid);
            }
            break;
        case NODE_PERIODIC_INTERVAL:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 5 - NODE PERIODIC INTERVAL Command",
                         frame->nid);
            }
            break;
        case GATEWAY_PANEL_STATUS:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 6 - GATEWAY PANEL STATUS Command",
                         frame->nid);
            }
            break;
        case NODE_PERIODIC_MESSAGE:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 7 - NODE PERIODIC MESSAGE Command",
                         frame->nid);
            }
            break;
        case NETWORK_CONFIG:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 11 - NETWORK CONFIG Command",
                         frame->nid);
            }
            break;
        case APN_CONFIG:
            {
               ESP_LOGI(TAG,
                         "Node %d: FC 12 - APN CONFIG Command",
                         frame->nid);
            }
            break;
        case GATEWAY_FIRMWARE_VERSION:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 13 - GATEWAY FIRMWARE VERSION Command",
                         frame->nid);
            }
            break;
        case SIM_INFORMATION:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 14 - SIM INFORMATION Command",
                         frame->nid);
            }
            break;
        case OTA_FRAME:
            {
               ESP_LOGI(TAG,
                         "Node %d: FC 9 - OTA Command",
                         frame->nid);
            }
            break;
        case ALARM_FRAME:
            {
                ESP_LOGI(TAG,
                         "Node %d: FC 8 - ALARM Command",
                         frame->nid);
            }
            break;
        default:
            ESP_LOGW(TAG,
                    "Node %d: Unknown Function Code %d",
                    frame->nid,
                    frame->function_code);
            break;
    }

    return NO_ERROR;
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
        frame->frame_version = (uint8_t)atoi(token);

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

    token = strtok(NULL, "");
    if (token)
    {
        // strcpy((char *)frame->data, token);
        strncpy((char *)frame->data,
        token,
        sizeof(frame->data) - 1);
        frame->data[sizeof(frame->data) - 1] = '\0';
    }
}

/* ===================== BUILD MQTT FRAME ===================== */

void build_mqtt_frame(sensor_frame_t *frame, char *payload)
{
    sprintf(payload,
            "#%d %s %s %d %d %d %s $",
            frame->frame_version,
            frame->password,
            frame->gid,
            frame->nid,
            frame->function_code,
            frame->action_code,
            frame->data);
}

/* ===================== MQTT PUBLISH TASK ===================== */

// static void mqtt_publish_task(void *arg)
// {
//     while (1)
//     {
//         if (mqtt_connected && mqtt_client)
//         {
//             char lora_data[200];
//             // printf("before LoRa data praful...\n");
//             /* Simulate LoRa packet */
//             // simulate_lora_receive(lora_data);
//             // printf("Simulated LoRa Data: %s\n", lora_data);
//             // ESP_LOGI(TAG,"LoRa RX: %s",lora_data);
//             if (get_next_lora_uplink_string(lora_data))
//             {
//                 printf("Received LoRa Data: %s\n", lora_data);
//                 ESP_LOGI(TAG, "Received LoRa Data: %s", lora_data);
//                 sensor_frame_t frame;

//                 parse_lora_frame(lora_data, &frame);

//                 char mqtt_payload[200];

//                 build_mqtt_frame(&frame, mqtt_payload);

//                 char topic[100];

//                 sprintf(topic,"gateway/MH-AMT-01"); // frame.gid

//                 // int msg_id = esp_mqtt_client_publish( mqtt_client, topic, mqtt_payload, 0,1, 0);
                
//                 // ESP_LOGI(TAG, "MQTT Topic: %s", topic);
//                 // ESP_LOGI(TAG, "MQTT Payload: %s", mqtt_payload);
//                 // ESP_LOGI(TAG, "msg_id=%d", msg_id);
                 
//                 // if (frame.function_code == 10) {
//                 //     char ack_payload[200];
//                 //     sprintf(ack_payload, "#1 AB1234 MH-AMT-XX 1 10 3 1 0 $");
//                 //     esp_mqtt_client_publish(mqtt_client, topic, ack_payload, 0, 1, 0);
//                 // }
//             }
//         }
//         else
//         vTaskDelay(pdMS_TO_TICKS(5000)); // Only delay if Wi-Fi/MQTT is disconnected so we don't spam CPU
//     }
// }

static void mqtt_publish_task(void *arg)
{
    while (1)
    {
        if (mqtt_connected && mqtt_client)
        {
            char lora_data[200];
            
            // 1. Wait for the perfectly formatted string from main.c
            if (get_next_lora_uplink_string(lora_data))
            {
                ESP_LOGI(TAG, "Pulled from Queue: %s", lora_data);
                
                char topic[100];
                sprintf(topic, "gateway/MH-AMT-01");

                // 2. DIRECT PUBLISH: Skip parse_lora_frame and build_mqtt_frame entirely!
                int msg_id = esp_mqtt_client_publish(mqtt_client, topic, lora_data, 0, 1, 0);
                
                if (msg_id >= 0) {
                    ESP_LOGI(TAG, "Successfully published to HiveMQ (msg_id=%d)", msg_id);
                } else {
                    ESP_LOGE(TAG, "Failed to publish to HiveMQ");
                }
            }
        }
        else
        {
            // Only delay if disconnected to prevent CPU spam
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

/* ===================== MQTT EVENT HANDLER ===================== */

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {
        case MQTT_EVENT_CONNECTED:
        {    
                ESP_LOGI(TAG, "MQTT CONNECTED");

                mqtt_connected = true;
                mqtt_client = event->client;

                esp_mqtt_client_subscribe(
                    mqtt_client,
                    SUBSCRIBE_TOPIC,
                    1);

                // ONLY create the task if it doesn't already exist
            if (publish_task_handle == NULL) {
                BaseType_t ret = xTaskCreate(
                    mqtt_publish_task,
                    "mqtt_publish",
                    4096,
                    NULL,
                    5,
                    &publish_task_handle); // Save the handle
                    ESP_LOGI(TAG, "xTaskCreate returned %d", ret);
            }
        }
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
                printf("Version: %d\n", frame.frame_version);
                printf("GID: %s\n", frame.gid);
                printf("Data: %s\n", frame.data);

                if (fnMQTT_frame_parser(&frame) == NO_ERROR)
                {
                    ESP_LOGI(TAG, "Frame Processed Successfully");
                }
                else
                {
                    ESP_LOGE(TAG, "Frame Processing Failed");
                }
            }
            else
            {
                ESP_LOGE(TAG, "Frame INVALID - Ignored");
            }


                // if (frame.nid == 0)
                // {
                //     /*
                //     * Only Gateway functions are allowed here.
                //     */

                //     switch (frame.function_code)
                //     {
                //         case 0:
                //             /* Gateway Reboot */
                //             break;

                //         case 1:
                //             /* Handshake */
                //             break;

                //         case 2:
                //             /* Gateway Configuration */
                //             break;

                //         case 4:
                //             /* Number of Nodes */
                //             break;

                //         case 5:
                //             /* Periodic Interval */
                //             break;

                //         case 6:
                //             /* Gateway Panel Status */
                //             break;

                //         case 9:
                //             /* Gateway OTA */
                //             break;

                //         case 11:
                //             /* MQTT Network Configuration */
                //             break;

                //         case 12:
                //             /* APN */
                //             break;

                //         case 13:
                //             /* Gateway Firmware */
                //             break;

                //         case 14:
                //             /* SIM Information */
                //             break;

                //         default:
                //             ESP_LOGE(TAG,"Invalid Gateway Function Code: %d",frame.function_code);
                //             // mqtt_send_nack(&frame, "Invalid Gateway Function Code");
                //             break;
                //     }
                // }
                // else if (frame.nid >= 1 && frame.nid <= 255)
                // {
                //     ESP_LOGI(TAG, "Valid Node ID: %d", frame.nid);

                //     switch (frame.function_code)
                //     {
                //         case 10:
                //             handle_fc10(&frame);
                //             break;

                //         // case 0:
                //         //     handle_node_reboot(&frame);
                //         //     break;

                //         // case 3:
                //         //     handle_node_threshold(&frame);
                //         //     break;

                //         // case 7:
                //         //     handle_node_periodic(&frame);
                //         //     break;

                //         // case 8:
                //         //     handle_node_alarm(&frame);
                //         //     break;

                //         // case 9:
                //         //     handle_node_ota(&frame);
                //         //     break;

                //         default:
                //             ESP_LOGW(TAG,"Unsupported Node Function Code: %d", frame.function_code);
                //             break;
                //     }
                // }
                // else
                // {
                //     ESP_LOGE(TAG, "Invalid Node ID: %d", frame.nid);
                // }
            
            
            // }
            // else
            // {
            //     ESP_LOGE(TAG, "Frame INVALID - Ignored");

            // }

            break;
        }

        case MQTT_EVENT_DISCONNECTED:
        {

            
            ESP_LOGW(TAG, "MQTT DISCONNECTED");
            
            mqtt_connected = false;
            
            break;
        }

        default:
        {

            break;
        }
    }
}

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