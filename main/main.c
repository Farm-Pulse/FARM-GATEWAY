#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "esp_utils.h"
#include "lora.h"
#include "ssd1306.h"
#include "sx127x.h"

SSD1306_t oled;
static const char *TAG = "sx127x_rx";
sx127x device;

void app_main(void)
{
    ESP_LOGI(TAG, "Farm-Gateway starting...");

    // OLED init
    i2c_master_init(&oled, GPIO_NUM_18, GPIO_NUM_17, -1);
    ssd1306_init(&oled, 128, 64);
    ssd1306_clear_screen(&oled, false);
    ssd1306_contrast(&oled, 0xff);
    ssd1306_display_text(&oled, 0, "LoRa RX Ready", 13, false);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    lora_init();
}
