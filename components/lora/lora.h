#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sx127x.h"

#define LORA_MAX_PAYLOAD 255

typedef struct {
    uint8_t  data[LORA_MAX_PAYLOAD];
    uint16_t len;
    int16_t  rssi;
    float    snr;
    int32_t  freq_error;
    uint32_t pkt_count;
} lora_rx_packet_t;

void lora_init(sx127x *device);
QueueHandle_t lora_get_rx_queue(void);

/* This is the callback you register with sx127x */
void my_lora_rx_callback(void *ctx, uint8_t *data, uint16_t len);
