#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mac_layer.h"
#include "network_layer.h"
#include "farmpulse_defs.h"
#include "farmpulse_config.h"
#include "lora_parser.h"
#include "mqtt.h"

static const char *TAG = "GATEWAY_MAIN";
//#define MY_NODE_ID 0

// --- EMSAVE ALIVE TABLE ---
bool alive_table[256] = {false};
static QueueHandle_t web_to_lora_queue; // Holds commands from Cloud
QueueHandle_t lora_to_web_queue; // Holds formatted strings for Cloud
uint8_t motor_action = 0;

// --- IPC QUEUES (The Bridge between LoRa and MQTT) ---
typedef struct
{
    uint8_t target_node_id;
    uint8_t msg_type;
    uint8_t data[MAX_PAYLOAD_SIZE];
    uint8_t data_len;
} lora_cmd_t;

// ====================================================================
// PUBLIC API FOR MQTT.C TO USE
// ====================================================================

// 1. MQTT Team calls this when a cloud command arrives
void push_downlink_command_to_lora(int node_id, int msg_type, uint8_t *data, int data_len) {
    lora_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.target_node_id = (uint8_t)node_id;
    cmd.msg_type = (uint8_t)msg_type;
    cmd.data_len = (uint8_t)data_len;

    if (data_len > 0 && data_len <= MAX_PAYLOAD_SIZE) {
        memcpy(cmd.data, data, data_len);
    }

    xQueueSend(web_to_lora_queue, &cmd, 0);
}


// 2. MQTT Team calls this to get real data instead of simulating
bool get_next_lora_uplink_string(char* buffer) {
    // Waits up to 1 second for real data from the LoRa mesh
    return (xQueueReceive(lora_to_web_queue, buffer, pdMS_TO_TICKS(1000)) == pdTRUE);
}


// ====================================================================
// LORA TX MANAGER (DOWNLINK TO NODES)
// ====================================================================

void lora_gateway_task(void *arg)
{
    uint32_t seconds_since_discovery = 0;
    // gateway_cmd_t incoming_cmd;
    lora_cmd_t incoming_cmd;

    while (1)
    {
        /* =====================================================
         * 1. Check for commands from Cloud / MQTT
         * ===================================================== */

        if (xQueueReceive(
                web_to_lora_queue,
                &incoming_cmd,
                pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            ESP_LOGW(
                TAG,
                ">>> WEB COMMAND RECEIVED for Node %d <<<", incoming_cmd.target_node_id);

            ESP_LOGI(
                TAG,
                "Msg_type: %d, Data_len: %d",
                incoming_cmd.msg_type,
                incoming_cmd.data_len);

            /* =================================================
             * Check if Node is alive
             * ================================================= */

            if (alive_table[incoming_cmd.target_node_id])
            {
                ESP_LOGI(
                    TAG,
                    "Node %d is alive",
                    incoming_cmd.target_node_id);

                bool success = false;
                /*
                    * Send payload through
                    * existing LoRa network layer.
                    */
                success = network_send(
                    incoming_cmd.target_node_id,
                    incoming_cmd.msg_type,
                    incoming_cmd.data,
                    incoming_cmd.data_len);

                /* =================================================
                 * Transmission Result
                 * ================================================= */

                // if (incoming_cmd.function_code == 10 &&
                //     incoming_cmd.action_code == 2 &&
                //     success == false)
                if (success == false)
                {
                    ESP_LOGE(
                        TAG,
                        "!!! COMM ERROR !!! "
                        "Failed to reach Node %d",
                        incoming_cmd.target_node_id);

                    alive_table[
                        incoming_cmd.target_node_id
                    ] = false;
                }
                else if (success == true)
                {
                    ESP_LOGI(
                        TAG,
                        "LoRa command successfully sent "
                        "to Node %d",
                        incoming_cmd.target_node_id);
                }
            }

            /* =================================================
             * Node is not alive
             * ================================================= */

            else
            {
                ESP_LOGE(
                    TAG,
                    "Cannot send command. "
                    "Node %d is DEAD/OFFLINE.",
                    incoming_cmd.target_node_id);
            }
        }

        /* =====================================================
         * 2. Background Discovery
         * ===================================================== */

        seconds_since_discovery++;

        if (seconds_since_discovery >= 30)
        {
            ESP_LOGI(
                TAG,
                "Gateway: Sending Discovery Broadcast...");

            uint8_t dummy = 0;

            network_send(
                0xFF,
                PKT_TYPE_STATUS,
                &dummy,
                1);

            seconds_since_discovery = 0;
        }

    }
}


void app_main(void) {
    //Versioning
    const esp_app_desc_t *app_desc = esp_app_get_description();
    
    ESP_LOGI(TAG, "FarmPulse Firmware Version: %s", app_desc->version);
    ESP_LOGI(TAG, "Project Name: %s", app_desc->project_name);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }

    // 3. Signature & Identity Check
    farmpulse_config_init();
    // Force this board to be the Gateway (Node 0)
    farmpulse_save_node_id(0);

    // FORCE GATEWAY IDENTITY: If NVS defaulted to 1, overwrite it to 0.
    if (system_config.node_id != 0) {
        ESP_LOGW(TAG, "Incorrect Gateway ID detected in NVS. Forcing to 0...");
        farmpulse_save_node_id(0);
    }
    
    // Initialize the IPC Queues
    web_to_lora_queue = xQueueCreate(10, sizeof(lora_cmd_t));
    lora_to_web_queue = xQueueCreate(10, 256); // Holds strings up to 256 chars

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "   FARMPULSE GATEWAY [TD: %d] - STARTING...",system_config.node_id);
    ESP_LOGI(TAG, "==========================================");

    mac_init();     
    network_init(); 
    network_register_cb(app_packet_handler);
    
    // 1. Start the LoRa Task
    xTaskCreate(lora_gateway_task, "lora_task", 4096, NULL, 5, NULL);
    
    // 2. Start your Teammate's MQTT & WiFi System
    mqtt_system_start();
}