#include <inttypes.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sx127x.h"
#include "esp_utils.h"
#include "ssd1306.h"
#include "lora.h"

/* ================== GLOBALS ================== */

static const char *TAG = "gateway_main";

SSD1306_t oled;
sx127x device;

/* ================== OLED TASK ================== */

static void oled_task(void *arg)
{
    lora_rx_packet_t pkt;

    char line0[22];
    char line1[22];
    char line2[22];
    char line3[22];
    char line4[22];

    while (1) {
        if (xQueueReceive(lora_get_rx_queue(), &pkt, portMAX_DELAY)) {

            char msg[32];
            uint16_t len = pkt.len < sizeof(msg) - 1 ? pkt.len : sizeof(msg) - 1;
            memcpy(msg, pkt.data, len);
            msg[len] = '\0';

            snprintf(line0, sizeof(line0), "RX: %.16s", msg);
            snprintf(line1, sizeof(line1), "RSSI: %d dBm", pkt.rssi);
            snprintf(line2, sizeof(line2), "SNR : %.2f dB", pkt.snr);
            snprintf(line3, sizeof(line3), "FERR: %" PRId32, pkt.freq_error);
            snprintf(line4, sizeof(line4), "PKT : %" PRIu32, pkt.pkt_count);

            ssd1306_clear_screen(&oled, false);
            ssd1306_display_text(&oled, 0, line0, strlen(line0), false);
            ssd1306_display_text(&oled, 3, line1, strlen(line1), false);
            ssd1306_display_text(&oled, 4, line2, strlen(line2), false);
            ssd1306_display_text(&oled, 5, line3, strlen(line3), false);
            ssd1306_display_text(&oled, 6, line4, strlen(line4), false);
            ssd1306_show_buffer(&oled);
        }
    }
}

/* ================== APP MAIN ================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Farm-Gateway");

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

    /* ---------- RX MODE ---------- */
    ESP_ERROR_CHECK(sx127x_set_opmod(
        SX127X_MODE_RX_CONT,
        SX127X_MODULATION_LORA,
        &device));

    ESP_LOGI(TAG, "LoRa RX continuous mode started");
}
