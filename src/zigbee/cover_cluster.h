#ifndef _COVER_CLUSTER_H_
#define _COVER_CLUSTER_H_

#include "base_components/relay.h"
#include "hal/zigbee.h"
#include "hal/tasks.h"
#include <stdint.h>

typedef struct {
  uint8_t reversal;
  uint16_t calibration_time;
  uint16_t closed_slack;
  uint16_t open_slack;
  uint16_t motor_position_bp;
} zigbee_cover_cluster_config;

typedef struct {
  // Parameters
  uint8_t cover_idx;
  uint8_t endpoint;
  relay_t *open_relay;
  relay_t *close_relay;

  // Attributes
  uint8_t window_covering_type;
  uint8_t position;
  uint8_t moving;
  uint8_t reversal;
  uint8_t calibration;
  uint16_t calibration_time;
  uint16_t closed_slack;
  uint16_t open_slack;
  hal_zigbee_attribute attr_infos[8];

  // State
  uint16_t motor_position_bp;
  uint32_t movement_start_time;
  uint16_t start_motor_position_bp;
  uint8_t calibration_direction;
  hal_task_t stop_task;
  hal_task_t position_update_task;
  hal_task_t calibration_timeout_task;

} zigbee_cover_cluster;

void cover_cluster_add_to_endpoint(
    zigbee_cover_cluster *cluster,
    hal_zigbee_endpoint *endpoint);

void cover_open(zigbee_cover_cluster *cluster);
void cover_close(zigbee_cover_cluster *cluster);
void cover_stop(zigbee_cover_cluster *cluster);
void cover_goto_position(zigbee_cover_cluster *cluster, uint8_t target_position);

void cover_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                  uint16_t attribute_id);

#endif
