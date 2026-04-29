#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mac_layer.h"
#include "network_layer.h"
#include "farmpulse_defs.h"
#include "mqtt.h"

static const char *TAG = "GATEWAY_MAIN";
#define MY_NODE_ID 0

// --- EMSAVE ALIVE TABLE ---
static bool alive_table[256] = {false};

// --- IPC QUEUES (The Bridge between LoRa and MQTT) ---
typedef struct {
    uint8_t target_node_id;
    uint8_t command_type;
} gateway_cmd_t;

static QueueHandle_t web_to_lora_queue; // Holds commands from Cloud
static QueueHandle_t lora_to_web_queue; // Holds formatted strings for Cloud

// ====================================================================
// PUBLIC API FOR MQTT.C TO USE
// ====================================================================

// 1. MQTT Team calls this when a cloud command arrives
void push_downlink_command_to_lora(int node_id, int action_code) {
    gateway_cmd_t cmd;
    cmd.target_node_id = (uint8_t)node_id;
    cmd.command_type = (uint8_t)action_code; // 1 = ON, 0 = OFF (Modify based on your spec)
    xQueueSend(web_to_lora_queue, &cmd, 0);
}

// 2. MQTT Team calls this to get real data instead of simulating
bool get_next_lora_uplink_string(char* buffer) {
    // Waits up to 1 second for real data from the LoRa mesh
    return (xQueueReceive(lora_to_web_queue, buffer, pdMS_TO_TICKS(1000)) == pdTRUE);
}

// ====================================================================
// LORA RX HANDLER (UPLINK TO CLOUD)
// ====================================================================

void app_packet_handler(uint8_t src_id, uint8_t type, uint8_t *data, uint8_t len) {
    if (type == PKT_TYPE_DATA && len == sizeof(sensor_data_t)) {
        sensor_data_t *s = (sensor_data_t *)data;
        
        // Update Alive Table
        if (!alive_table[src_id]) {
            ESP_LOGW(TAG, ">>> NODE %d ADDED TO ALIVE TABLE <<<", src_id);
            alive_table[src_id] = true;
        }

        // Format the real data into your teammate's expected string protocol!
        // Format: #<version> <pwd> <gid> <nid> <func> <action> <data>$
        char lora_str[256];
        sprintf(lora_str, "#01 AB1234 MH-AMT-01 %d 02 01 V_RYB:%d,%d,%d I_RYB:%d,%d,%d PWR:%ld MTR:%d$", 
                src_id, 
                s->voltage_R, s->voltage_Y, s->voltage_B,
                s->current_R, s->current_Y, s->current_B,
                s->power_active, s->motor_status);
        
        // Push the formatted string to the MQTT task
        xQueueSend(lora_to_web_queue, lora_str, pdMS_TO_TICKS(100));
        
        ESP_LOGI(TAG, "Data from Node %d pushed to MQTT Queue.", src_id);
    }
}

// ====================================================================
// LORA TX MANAGER (DOWNLINK TO NODES)
// ====================================================================

void lora_gateway_task(void *arg) {
    uint32_t seconds_since_discovery = 0;
    gateway_cmd_t incoming_cmd;

    while (1) {
        // 1. Check for commands from the Cloud/MQTT
        if (xQueueReceive(web_to_lora_queue, &incoming_cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            
            ESP_LOGW(TAG, ">>> WEB COMMAND RECEIVED for Node %d <<<", incoming_cmd.target_node_id);

            // Check if the Node is alive before transmitting
            if (alive_table[incoming_cmd.target_node_id]) {
                ESP_LOGI(TAG, "Transmitting Command over LoRa Mesh...");
                bool success = network_send(incoming_cmd.target_node_id, PKT_TYPE_CMD, &incoming_cmd.command_type, 1);
                
                if (!success) {
                    // DECLARE DEAD LOGIC
                    ESP_LOGE(TAG, "!!! COMM ERROR !!! Failed to reach Node %d.", incoming_cmd.target_node_id);
                    alive_table[incoming_cmd.target_node_id] = false; 
                }
            } else {
                ESP_LOGE(TAG, "Cannot send command. Node %d is DEAD/OFFLINE.", incoming_cmd.target_node_id);
            }
        }

        // 2. Background Maintenance: Discovery Broadcasts every 30 seconds
        seconds_since_discovery++;
        if (seconds_since_discovery >= 30) {
            ESP_LOGI(TAG, "Gateway: Sending Discovery Broadcast...");
            uint8_t dummy = 0;
            network_send(0xFF, PKT_TYPE_STATUS, &dummy, 1);
            seconds_since_discovery = 0;
        }
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }

    // Initialize the IPC Queues
    web_to_lora_queue = xQueueCreate(10, sizeof(gateway_cmd_t));
    lora_to_web_queue = xQueueCreate(10, 256); // Holds strings up to 256 chars

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "   FARMPULSE GATEWAY - STARTING...");
    ESP_LOGI(TAG, "==========================================");

    mac_init();     
    network_init(); 
    network_register_cb(app_packet_handler);
    
    // 1. Start the LoRa Task
    xTaskCreate(lora_gateway_task, "lora_task", 4096, NULL, 5, NULL);
    
    // 2. Start your Teammate's MQTT & WiFi System
    mqtt_system_start();
}