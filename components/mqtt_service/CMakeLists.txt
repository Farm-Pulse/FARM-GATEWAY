idf_component_register(
    SRCS "mqtt.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event esp_netif mqtt nvs_flash
)