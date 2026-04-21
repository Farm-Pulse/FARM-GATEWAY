#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mac_layer.h"
#include "network_layer.h"
#include "farmpulse_defs.h"
#include "sx127x.h"
#include "esp_utils.h"
#include "ssd1306.h"
#include "lora.h"
#include "mqtt.h"
#include "nvs_flash.h"

static const char *TAG = "GATEWAY_MAIN";

#define MY_NODE_ID       CONFIG_FARMPULSE_NODE_ID // 0

static uint8_t motor_state = 0; 

// --- EMSAVE ALIVE TABLE ---
// Tracks the status of up to 256 possible nodes in the network
static bool alive_table[256] = {false};

void app_packet_handler(uint8_t src_id, uint8_t type, uint8_t *data, uint8_t len) {
    if (type == PKT_TYPE_DATA && len == sizeof(sensor_data_t)) {
        sensor_data_t *s = (sensor_data_t *)data;
        
        // --- ALIVE TABLE LOGIC ---
        // If we receive data from a node, mark it as Alive!
        if (!alive_table[src_id]) {
            ESP_LOGW(TAG, ">>> NODE %d ADDED TO ALIVE TABLE <<<", src_id);
            alive_table[src_id] = true;
        }

        ESP_LOGI(TAG, "--- 3-PHASE DATA FROM NODE %d ---", src_id);
        ESP_LOGI(TAG, "   Voltage: R=%d V, Y=%d V, B=%d V", s->voltage_R, s->voltage_Y, s->voltage_B);
        ESP_LOGI(TAG, "   Current: R=%d A, Y=%d A, B=%d A (x10)", s->current_R, s->current_Y, s->current_B);
        ESP_LOGI(TAG, "   Power:   %ld Watts", s->power_active);
        ESP_LOGI(TAG, "   Motor:   %s", s->motor_status ? "ON" : "OFF");
        ESP_LOGI(TAG, "------------------------------------");
    }
}

void application_task(void *arg) {
    uint32_t counter = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 Second loop

        if (counter % 4 == 0) {
            uint8_t cmd = (motor_state == 0) ? CMD_MOTOR_ON : CMD_MOTOR_OFF;
            motor_state = !motor_state; 
            
            bool command_sent = false;
            for (int i = 1; i < 256; i++) {
                if (alive_table[i]) {
                    ESP_LOGI(TAG, "Sending CMD_MOTOR_%s to Node %d", cmd ? "ON" : "OFF", i);
                    
                    // Send the command and capture the result
                    bool success = network_send(i, PKT_TYPE_CMD, &cmd, 1);
                    
                    if (success) {
                        ESP_LOGI(TAG, "Command delivered to Node %d successfully.", i);
                    } else {
                        // --- NEW: EMSAVE DECLARE DEAD LOGIC ---
                        ESP_LOGE(TAG, "!!! COMM ERROR !!! Failed to reach Node %d.", i);
                        ESP_LOGE(TAG, ">>> DECLARING NODE %d DEAD <<<", i);
                        
                        // Remove from Alive Table so we don't keep spamming it
                        alive_table[i] = false; 
                    }
                    
                    command_sent = true;
                    vTaskDelay(pdMS_TO_TICKS(500)); 
                }
            }

            if (!command_sent) {
                ESP_LOGW(TAG, "No nodes in Alive Table to send command to.");
            }
        } 
        else {
            ESP_LOGI(TAG, "Sending Discovery Broadcast...");
            uint8_t dummy = 0;
            network_send(0xFF, PKT_TYPE_STATUS, &dummy, 1);
        }
        counter++;
    }
}

/* ================== APP MAIN ================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Farm-Gateway");
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }
        
    mqtt_system_start();

    /* ---------- OLED INIT ---------- */
    i2c_master_init(&oled, GPIO_NUM_18, GPIO_NUM_17, -1);
    ssd1306_init(&oled, 128, 64);
    ssd1306_clear_screen(&oled, false);
    ssd1306_contrast(&oled, 0xFF);
    ssd1306_display_text(&oled, 0, "LoRa RX Ready", 13, false);

    vTaskDelay(pdMS_TO_TICKS(500));

    /* ---------- SX127X RESET ---------- */
    sx127x_util_reset();

    /* ---------- SPI INIT ---------- */
    spi_device_handle_t spi_device;
    sx127x_init_spi(&spi_device);

    /* ---------- RADIO INIT ---------- */
    ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
    ESP_ERROR_CHECK(sx127x_set_opmod(
        SX127X_MODE_STANDBY,
        SX127X_MODULATION_LORA,
        &device));

    ESP_ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&device));

    ESP_ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, &device));
    ESP_ERROR_CHECK(sx127x_rx_set_lna_gain(SX127X_LNA_GAIN_G4, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127X_BW_125000, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(SX127X_SF_9, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(0x12, &device));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &device));

    /* ---------- LORA MODULE INIT ---------- */
    lora_init(&device);

    /* ---------- INTERRUPT TASK (LIBRARY) ---------- */
    ESP_ERROR_CHECK(setup_task(&device));

    gpio_install_isr_service(0);
    setup_gpio_interrupts(
        (gpio_num_t)DIO0,
        &device,
        GPIO_INTR_POSEDGE);

    /* ---------- OLED TASK ---------- */
    xTaskCreatePinnedToCore(
        oled_task,
        "oled_task",
        4096,
        NULL,
        1,
        NULL,
        xPortGetCoreID());

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "   FARMPULSE GATEWAY - Node ID: %d", MY_NODE_ID);
    ESP_LOGI(TAG, "==========================================");

    ESP_LOGI(TAG, "LoRa RX continuous mode started");
}