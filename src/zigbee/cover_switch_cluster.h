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

// Configuration attributes (similar to switch_cluster)
typedef struct {
  uint8_t input_idx;
  uint8_t endpoint;
  
  // Physical buttons
  button_t *open_button;
  button_t *close_button;
  
  // Configuration attributes
  uint8_t switch_type;             // Switch type (always 0x02 = multifunction)
  uint8_t output_index;            // Which cover to control locally
  uint8_t reversal;                // Swap OPEN/CLOSE (0=normal, 1=reversed)
  uint8_t local_mode;              // Detached/press_start/short/long/both
  uint8_t binded_mode;             // When to send bind commands
  
  // State reporting
  uint16_t multistate_state;       // Current press action
  
  hal_zigbee_attribute config_attr_infos[6];      // OnOffSwitchConfig attributes (configuration)
  hal_zigbee_attribute multistate_attr_infos[4];  // MultiStateInput attributes (state reporting)
} zigbee_cover_switch_cluster;

void cover_switch_cluster_add_to_endpoint(
    zigbee_cover_switch_cluster *cluster,
    hal_zigbee_endpoint *endpoint);

void cover_switch_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                        uint16_t attribute_id);

#endif


