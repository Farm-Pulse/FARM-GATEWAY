idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        esp_app_format
        nvs_flash
        mqtt
        mqtt_service
        lora
        sx127x
)
