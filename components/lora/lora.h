#pragma once

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "sx127x.h"
#include "ssd1306.h"

extern SSD1306_t oled;

// RX packet structure
typedef struct {
    uint8_t data[256];
    uint16_t len;
    int16_t rssi;
    float snr;
    int32_t ferr;
} rx_packet_t;

// Functions
void lora_init(void);
void mylora_rx_callback(void *ctx, uint8_t *data, uint16_t data_length);
