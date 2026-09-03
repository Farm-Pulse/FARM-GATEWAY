#ifndef GATEWAY_PARSER_H
#define GATEWAY_PARSER_H

#include <stdint.h>

void app_packet_handler(uint8_t src_id, uint8_t type, uint8_t *data, uint8_t len);

#endif // GATEWAY_PARSER_H