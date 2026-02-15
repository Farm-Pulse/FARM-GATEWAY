/**
 * @file MAC-layer.h
 * @brief MAC (Media Access Configuration) Layer Implementation - Base Level RF Format Packet
 * @author Shahid  
 * @date Feb 2026
 */

#ifndef MAC_LAYER_H
#define MAC_LAYER_H

#include <stdbool.h>
#include "farmpulse_defs.h"

// Initialize the LoRa hardware
void mac_init(void);

// Send a packet immediately (Low level)
bool mac_tx(farm_packet_t *packet);

#endif