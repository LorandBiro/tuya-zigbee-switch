#ifndef _COVER_CLUSTER_H_
#define _COVER_CLUSTER_H_

#include "base_components/relay.h"
#include "hal/zigbee.h"
#include <stdint.h>

// WindowCovering operational status values (bitmap - NOT USED)
// These were from ZCL spec but we use simpler enum values instead
// #define WINDOW_COVERING_STATUS_OPERATIONAL 0x01
// #define WINDOW_COVERING_STATUS_ONLINE 0x02
// #define WINDOW_COVERING_STATUS_OPEN_UP_COMMANDS_REVERSED 0x04
// #define WINDOW_COVERING_STATUS_LIFT_CONTROL_CLOSED_LOOP 0x08
// #define WINDOW_COVERING_STATUS_LIFT_POSITION_ENCODER 0x10
// #define WINDOW_COVERING_STATUS_TILT_POSITION_ENCODER 0x20
// #define WINDOW_COVERING_STATUS_LIFT_MOVING_UP 0x40
// #define WINDOW_COVERING_STATUS_LIFT_MOVING_DOWN 0x80

// Simplified operational status enum values (manufacturer-specific attribute 0xff06)
#define COVER_STATUS_STOPPED 0x00
#define COVER_STATUS_OPENING 0x01
#define COVER_STATUS_CLOSING 0x02

typedef struct {
  uint8_t output_idx;
  uint8_t endpoint;
  
  // Physical relays
  relay_t *up_relay;
  relay_t *down_relay;
  
  // Configuration
  uint8_t reversal;  // Motor reversal (0 = normal, 1 = reversed)
  
  // State reporting
  uint8_t status;  // Custom attribute: 0=stopped, 1=opening, 2=closing
  
  // Zigbee attributes
  uint8_t window_covering_type;  // 0x08 = Rollershade
  uint8_t position;              // 0-100 (0=closed, 100=open)
  
  hal_zigbee_attribute attr_infos[4];  // WindowCovering attributes
} zigbee_cover_cluster;

void cover_cluster_add_to_endpoint(
    zigbee_cover_cluster *cluster,
    hal_zigbee_endpoint *endpoint);

void cover_up(zigbee_cover_cluster *cluster);
void cover_down(zigbee_cover_cluster *cluster);
void cover_stop(zigbee_cover_cluster *cluster);

void cover_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                         uint16_t attribute_id);

#endif


