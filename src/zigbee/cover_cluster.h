#ifndef _COVER_CLUSTER_H_
#define _COVER_CLUSTER_H_

#include "base_components/relay.h"
#include "hal/zigbee.h"
#include "hal/tasks.h"
#include <stdint.h>

typedef struct {
  uint8_t output_idx;
  uint8_t endpoint;

  relay_t *open_relay;
  relay_t *close_relay;

  uint8_t window_covering_type;
  uint8_t position;
  uint8_t moving;
  uint8_t reversal;
  uint8_t calibration;
  uint16_t calibration_time;
  uint16_t closed_slack;
  uint16_t open_slack;

  // Motor position tracking
  // ----------------------
  // motor_position_bp: Physical motor position as basis points (0.01% precision)
  //   - Range: 0 (fully closed = 0.00%) to 10000 (fully open = 100.00%)
  //   - Stored as basis points for high precision without floating point
  //   - Works with separate opening/closing calibration times
  //
  // Relationship to cover position (0-100%):
  //   - Motor position is converted to cover position by accounting for slack zones
  //   - Slack zones are time-based, so conversion depends on calibration_time
  //
  // Example with closed_slack=5s, open_slack=3s, calibration_time=30s:
  //   Closed slack:     0ms - 5000ms    = 16.67% of time = motor_position_bp 0 - 1667
  //   Effective travel: 5000ms - 27000ms = 73.33% of time = motor_position_bp 1667 - 9000
  //   Open slack:       27000ms - 30000ms = 10.00% of time = motor_position_bp 9000 - 10000
  //
  //   motor_position_bp = 0      → position = 0%   (fully closed)
  //   motor_position_bp = 1000   → position = 0%   (in closed slack zone)
  //   motor_position_bp = 1667   → position = 0%   (end of closed slack)
  //   motor_position_bp = 5333   → position = 50%  (middle of effective travel)
  //   motor_position_bp = 9000   → position = 100% (start of open slack)
  //   motor_position_bp = 10000  → position = 100% (fully open)
  uint16_t motor_position_bp;

  // Timing state
  uint32_t movement_start_time;      // When current movement started (hal_millis)
  uint16_t start_motor_position_bp;  // Motor position when movement started
  uint8_t calibration_direction;     // Direction during calibration (OPENING/CLOSING)
  hal_task_t stop_task;              // Scheduled task for auto-stop
  hal_task_t position_update_task;   // Periodic position updates during movement
  hal_task_t calibration_timeout_task; // Safety timeout during calibration

  hal_zigbee_attribute attr_infos[8];
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
