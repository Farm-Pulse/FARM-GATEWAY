#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "lora.h"
#include "esp_utils.h"
#include "sx127x.h"
#include "ssd1306"

static const char *TAG = "LoRa";

extern SSD1306_t oled;
sx127x device;
static QueueHandle_t rx_queue;
volatile int total_packets_received = 0;

/* ========= RX CALLBACK (called from SX127x interrupt context) ========= */
void IRAM_ATTR lora_isr_handler(void *ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveIndexedFromISR(device.handle_interrupt, 0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// RX Task handles OLED/log updates
static void lora_rx_task(void *arg) {
    rx_packet_t packet;

    while (1) {
        if (xQueueReceive(rx_queue, &packet, portMAX_DELAY) == pdTRUE) {
            // OLED update
            char line0[22], line1[22], line2[22], line3[22], line4[22];

            snprintf(line0, sizeof(line0), "RX: %.16s", packet.data);
            snprintf(line1, sizeof(line1), "RSSI: %d dBm", packet.rssi);
            snprintf(line2, sizeof(line2), "SNR : %.2f dB", packet.snr);
            snprintf(line3, sizeof(line3), "FERR: %" PRId32, packet.ferr);
            snprintf(line4, sizeof(line4), "PKT : %d", total_packets_received);

            ssd1306_clear_screen(&oled, false);
            ssd1306_display_text(&oled, 0, line0, strlen(line0), false);
            ssd1306_display_text(&oled, 3, line1, strlen(line1), false);
            ssd1306_display_text(&oled, 4, line2, strlen(line2), false);
            ssd1306_display_text(&oled, 5, line3, strlen(line3), false);
            ssd1306_display_text(&oled, 6, line4, strlen(line4), false);
            ssd1306_show_buffer(&oled);

            // Logging HEX
            char buf[3*packet.len];
            for (int i=0; i<packet.len; i++) sprintf(&buf[i*2], "%02X", packet.data[i]);
            ESP_LOGI(TAG, "RX [%d bytes]: %s", packet.len, buf);
        }
    }
}

// RX callback: copy payload and push to queue
void mylora_rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
    sx127x *dev = (sx127x *)ctx;
    rx_packet_t packet;

    uint16_t len = (data_length < sizeof(packet.data)-1) ? data_length : sizeof(packet.data)-1;
    memcpy(packet.data, data, len);
    packet.data[len] = '\0';
    packet.len = len;

    ESP_ERROR_CHECK(sx127x_rx_get_packet_rssi(dev, &packet.rssi));
    ESP_ERROR_CHECK(sx127x_lora_rx_get_packet_snr(dev, &packet.snr));
    ESP_ERROR_CHECK(sx127x_rx_get_frequency_error(dev, &packet.ferr));

    total_packets_received++;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(rx_queue, &packet, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}


/* ========= INIT ========= */
void lora_init(void)
{
    ESP_LOGI(TAG, "Starting SX127x");

    /* Reset radio */
    sx127x_util_reset();

    /* SPI */
    spi_device_handle_t spi;
    sx127x_init_spi(&spi);

    /* SX127x device */
    ESP_ERROR_CHECK(sx127x_create(spi, &device));

    /* Radio config */
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &device));
    ESP_ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&device));
    ESP_ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, &device));
    ESP_ERROR_CHECK(sx127x_rx_set_lna_gain(SX127X_LNA_GAIN_G4, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127X_BW_125000, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_spreading_factor(SX127X_SF_9, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(0x12, &device));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &device));

    
    /* ===== CRITICAL ORDERING FIX ===== */

    /* 1. Create library interrupt task FIRST */
    //sx127x_rx_set_callback(lora_rx_callback, &device, &device);
    //sx127x_lora_cad_set_callback(cad_callback, &device, &device);
    //ESP_ERROR_CHECK(setup_task(&device));

    // Setup RX queue & task
    rx_queue = xQueueCreate(10, sizeof(rx_packet_t));
    xTaskCreatePinnedToCore(lora_rx_task, "lora_rx_task", 4096, NULL, 5, NULL, xPortGetCoreID());

    /* 2. Install GPIO ISR service */
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    /* 3. Attach DIO0 interrupt */
    //setup_gpio_interrupts(DIO0, &device, GPIO_INTR_POSEDGE);

    //gpio_install_isr_service(0);
    setup_gpio_interrupts((gpio_num_t)DIO0, &device, GPIO_INTR_POSEDGE);
    sx127x_rx_set_callback(mylora_rx_callback, &device, &device);
    sx127x_lora_cad_set_callback(cad_callback, &device, &device);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, &device));
    ESP_LOGI(TAG, "LoRa RX continuous mode");
}
