#ifndef MQTT_APP_H
#define MQTT_APP_H

void mqtt_system_start(void);

/* makes it easy for other modules (e.g. main) to publish LoRa
 * packets using the configured MQTT client */
void mqtt_publish_lora_data(const char *msg, int rssi, float snr);

#endif