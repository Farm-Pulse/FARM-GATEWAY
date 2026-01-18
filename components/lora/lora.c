#include "lora.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "lora";
static QueueHandle_t rx_queue;
static uint32_t packet_counter = 0;

void lora_init(sx127x *device)
{
    rx_queue = xQueueCreate(8, sizeof(lora_rx_packet_t));
    configASSERT(rx_queue);

    /* Register callback EXACTLY as library expects */
    sx127x_rx_set_callback(my_lora_rx_callback, device, device);

    ESP_LOGI(TAG, "LoRa RX queue initialized");
}

QueueHandle_t lora_get_rx_queue(void)
{
    return rx_queue;
}

/* ===== RX CALLBACK (NO OLED, NO LOGGING) ===== */
void my_lora_rx_callback(void *ctx, uint8_t *data, uint16_t len)
{
    sx127x *device = (sx127x *)ctx;
    lora_rx_packet_t pkt;

    if (len > LORA_MAX_PAYLOAD) {
        len = LORA_MAX_PAYLOAD;
    }

    pkt.len = len;
    memcpy(pkt.data, data, len);
    pkt.pkt_count = ++packet_counter;

    /* Metadata (fast, non-blocking) */
    sx127x_rx_get_packet_rssi(device, &pkt.rssi);
    sx127x_lora_rx_get_packet_snr(device, &pkt.snr);
    sx127x_rx_get_frequency_error(device, &pkt.freq_error);

    xQueueSend(rx_queue, &pkt, 0);
}
