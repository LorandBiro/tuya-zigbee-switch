#ifndef _COVER_SWITCH_CLUSTER_H_
#define _COVER_SWITCH_CLUSTER_H_

#include "base_components/button.h"
#include "hal/zigbee.h"
#include <stdint.h>

// Cover switch press actions (MultiStateInput values)
#define COVER_SWITCH_RELEASED 0
#define COVER_SWITCH_OPEN_PRESS 1
#define COVER_SWITCH_CLOSE_PRESS 2
#define COVER_SWITCH_STOP_PRESS 3
#define COVER_SWITCH_OPEN_LONG_PRESS 4
#define COVER_SWITCH_CLOSE_LONG_PRESS 5

// Local mode options
#define COVER_LOCAL_MODE_DETACHED 0
#define COVER_LOCAL_MODE_PRESS_START 1
#define COVER_LOCAL_MODE_SHORT_PRESS 3
#define COVER_LOCAL_MODE_LONG_PRESS 2
#define COVER_LOCAL_MODE_SHORT_AND_LONG_PRESS 4

// Binded mode options
#define COVER_BINDED_MODE_PRESS_START 1
#define COVER_BINDED_MODE_SHORT_PRESS 3
#define COVER_BINDED_MODE_LONG_PRESS 2
#define COVER_BINDED_MODE_SHORT_AND_LONG_PRESS 4

typedef struct {
  uint8_t output_index;
  uint8_t reversal;
  uint8_t local_mode;
  uint8_t binded_mode;
  uint8_t switch_type;
} zigbee_cover_switch_cluster_config;

typedef struct {
  // Parameters
  uint8_t cover_switch_idx;
  uint8_t endpoint;
  button_t *open_button;
  button_t *close_button;
  
  // On/Off Switch Configuration Attributes
  uint8_t switch_type;
  uint8_t output_index;
  uint8_t reversal;
  uint8_t local_mode;
  uint8_t binded_mode;
  hal_zigbee_attribute config_attr_infos[6];
  
  // Multistate Input Attributes
  uint16_t multistate_state;
  hal_zigbee_attribute multistate_attr_infos[4];
} zigbee_cover_switch_cluster;

void cover_switch_cluster_add_to_endpoint(
    zigbee_cover_switch_cluster *cluster,
    hal_zigbee_endpoint *endpoint);

void cover_switch_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                        uint16_t attribute_id);

#endif
