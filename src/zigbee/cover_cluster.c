#include "cover_cluster.h"
#include "base_components/relay.h"
#include "cluster_common.h"
#include "consts.h"
#include "device_config/nvm_items.h"
#include "hal/nvm.h"
#include "hal/printf_selector.h"
#include "hal/system.h"
#include "hal/tasks.h"
#include "hal/timer.h"
#include "hal/zigbee.h"

#define MAX_MOTOR_POSITION 10000

// ============================================================================
// Section 1: Forward declarations
// ============================================================================

hal_zigbee_cmd_result_t cover_cluster_callback(zigbee_cover_cluster *cluster,
    uint8_t command_id,
    void *cmd_payload);
hal_zigbee_cmd_result_t cover_cluster_callback_trampoline(uint8_t endpoint,
    uint8_t cluster_id,
    uint8_t command_id,
    void *cmd_payload);

void cover_cluster_on_write_attr(zigbee_cover_cluster *cluster,
                                          uint16_t attribute_id);

void cover_cluster_store_attrs_to_nv(zigbee_cover_cluster *cluster);
void cover_cluster_load_attrs_from_nv(zigbee_cover_cluster *cluster);
void cover_cluster_init(zigbee_cover_cluster *cluster);

// Timing & position helpers
uint8_t motor_to_cover_position(zigbee_cover_cluster *cluster);
uint16_t cover_to_motor_position(zigbee_cover_cluster *cluster, uint8_t cover_pos);
void cover_update_position(zigbee_cover_cluster *cluster);
void cover_schedule_next_position_update(zigbee_cover_cluster *cluster);
void cover_position_update_handler(void *arg);
void cover_auto_stop_handler(void *arg);
void cover_cancel_movement_tasks(zigbee_cover_cluster *cluster);

// Calibration functions
void cover_start_calibration_movement(zigbee_cover_cluster *cluster,
                                      uint8_t direction);
void cover_complete_calibration(zigbee_cover_cluster *cluster);
void cover_calibration_timeout_handler(void *arg);

// Movement control
void cover_goto_position(zigbee_cover_cluster *cluster, uint8_t target_position);
void cover_execute_movement(zigbee_cover_cluster *cluster, uint8_t target_position);
void cover_safety_delay_handler(void *arg);

// ============================================================================
// Section 2: Static arrays & trampoline functions
// ============================================================================

zigbee_cover_cluster *cover_cluster_by_endpoint[10];

void cover_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                           uint16_t attribute_id) {
  cover_cluster_on_write_attr(cover_cluster_by_endpoint[endpoint],
                                       attribute_id);
}

// ============================================================================
// Section 2.5: Position conversion functions
// ============================================================================
// motor_position: Physical motor position as basis points (0.01% precision)
//   - Range: 0 (fully closed = 0.00%) to 10000 (fully open = 100.00%)
//   - Stored as basis points for high precision without floating point
//   - Works with separate opening/closing calibration times
//
// Relationship to cover position (0-100%):
//   - Motor position is converted to cover position by accounting for slack zones
//   - Slack zones are time-based, so conversion depends on calibration_time
//
// Example with closed_slack=5s, open_slack=3s, calibration_time=30s:
//   Closed slack:     0ms - 5000ms    = 16.67% of time = motor_position 0 - 1667
//   Effective travel: 5000ms - 27000ms = 73.33% of time = motor_position 1667 - 9000
//   Open slack:       27000ms - 30000ms = 10.00% of time = motor_position 9000 - 10000
//
//   motor_position = 0      → position = 0%   (fully closed)
//   motor_position = 1000   → position = 0%   (in closed slack zone)
//   motor_position = 1667   → position = 0%   (end of closed slack)
//   motor_position = 5333   → position = 50%  (middle of effective travel)
//   motor_position = 9000   → position = 100% (start of open slack)
//   motor_position = 10000  → position = 100% (fully open)

uint8_t motor_to_cover_position(zigbee_cover_cluster *cluster) {
  uint32_t calibration_ms = cluster->calibration_time * 100;
  uint32_t closed_slack_ms = cluster->closed_slack * 100;
  uint32_t open_slack_ms = cluster->open_slack * 100;
  
  // Calculate motor position in milliseconds
  uint32_t motor_ms = (cluster->motor_position * calibration_ms) / MAX_MOTOR_POSITION;
  
  // In closed slack zone: motor is running but cover hasn't started moving
  if (motor_ms <= closed_slack_ms) {
    return 0;
  }
  
  // In open slack zone: cover has reached end but motor still running
  if (motor_ms >= calibration_ms - open_slack_ms) {
    return 100;
  }
  
  // In effective travel zone: motor movement translates to cover movement
  uint32_t effective_ms = calibration_ms - closed_slack_ms - open_slack_ms;
  if (effective_ms == 0) {
    effective_ms = 1;  // Prevent division by zero
  }
  uint32_t travel_ms = motor_ms - closed_slack_ms;
  return (travel_ms * 100) / effective_ms;
}

uint16_t cover_to_motor_position(zigbee_cover_cluster *cluster, uint8_t cover_pos) {
  if (cover_pos == 0) {
    return 0;  // Fully closed = 0.00%
  }
  
  if (cover_pos == 100) {
    return 10000;  // Fully open = 100.00%
  }
  
  // Map to effective travel zone (between slack zones)
  uint32_t calibration_ms = cluster->calibration_time * 100;
  uint32_t closed_slack_ms = cluster->closed_slack * 100;
  uint32_t open_slack_ms = cluster->open_slack * 100;
  uint32_t effective_ms = calibration_ms - closed_slack_ms - open_slack_ms;
  
  // Calculate target position in milliseconds
  uint32_t target_ms = closed_slack_ms + (cover_pos * effective_ms) / 100;
  
  // Convert to percentage (basis points)
  return (target_ms * 10000) / calibration_ms;
}

// ============================================================================
// Section 3: Main API functions (cover_open, cover_close, cover_stop)
// ============================================================================

void cover_open(zigbee_cover_cluster *cluster) {
  printf("Cover OPEN command\r\n");

  // If in calibration mode, handle specially
  if (cluster->calibration) {
    cover_start_calibration_movement(cluster,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING);
    return;
  }

  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    printf("Already opening\r\n");
    return;
  }

  cover_goto_position(cluster, 100);
}

void cover_close(zigbee_cover_cluster *cluster) {
  printf("Cover CLOSE command\r\n");

  // If in calibration mode, handle specially
  if (cluster->calibration) {
    cover_start_calibration_movement(cluster,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING_CLOSING);
    return;
  }

  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_CLOSING) {
    printf("Already closing\r\n");
    return;
  }

  cover_goto_position(cluster, 0);
}

void cover_stop(zigbee_cover_cluster *cluster) {
  printf("Cover STOP command\r\n");

  // If in calibration mode and moving, complete calibration
  if (cluster->calibration && cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED) {
    cover_complete_calibration(cluster);
    return;
  }

  // Cancel any pending tasks
  cover_cancel_movement_tasks(cluster);

  // Stop both relays
  relay_off(cluster->open_relay);
  relay_off(cluster->close_relay);

  cluster->last_relay_off_time = hal_millis();
  cluster->has_pending_movement = 0;

  // Update current position before stopping
  if (cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED) {
    cover_update_position(cluster);
  }

  cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
  
  // Send unsolicited position report when stopping
  hal_zigbee_send_report_attr(cluster->endpoint,
                              ZCL_CLUSTER_WINDOW_COVERING,
                              ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE,
                              ZCL_DATA_TYPE_UINT8,
                              &cluster->position,
                              1);
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

// ============================================================================
// Section 4: Timing helpers
// ============================================================================

void cover_cancel_movement_tasks(zigbee_cover_cluster *cluster) {
  hal_tasks_unschedule(&cluster->stop_task);
  hal_tasks_unschedule(&cluster->position_update_task);
  hal_tasks_unschedule(&cluster->calibration_timeout_task);
  hal_tasks_unschedule(&cluster->safety_delay_task);
}

void cover_update_position(zigbee_cover_cluster *cluster) {
  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED || cluster->calibration_time == 0) {
    return;
  }

  uint32_t elapsed_ms = hal_millis() - cluster->movement_start_time;
  uint32_t calibration_ms = cluster->calibration_time * 100;
  uint32_t motor_position_delta = (elapsed_ms * MAX_MOTOR_POSITION) / calibration_ms;
  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    uint32_t new_motor_position = cluster->start_motor_position + motor_position_delta;
    if (new_motor_position > MAX_MOTOR_POSITION) {
      new_motor_position = MAX_MOTOR_POSITION;
    }
    cluster->motor_position = (uint16_t)new_motor_position;
  } else {
    if (motor_position_delta > cluster->start_motor_position) {
      cluster->motor_position = 0;
    } else {
      cluster->motor_position = cluster->start_motor_position - (uint16_t)motor_position_delta;
    }
  }
  
  uint8_t new_cover_pos = motor_to_cover_position(cluster);
  if (new_cover_pos != cluster->position) {
    cluster->position = new_cover_pos;
    printf("Position updated: %d%% (motor: %u.%02u%%)\r\n",
           new_cover_pos,
           cluster->motor_position / 100,
           cluster->motor_position % 100);
    hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                       ZCL_CLUSTER_WINDOW_COVERING,
                                       ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE);
  }
}

void cover_schedule_next_position_update(zigbee_cover_cluster *cluster) {
  uint32_t effective_travel_ms = (cluster->calibration_time - cluster->closed_slack - cluster->open_slack) * 100;
  uint32_t update_interval_ms = effective_travel_ms / 100;
  if (update_interval_ms < 100) {
    update_interval_ms = 100;
  }

  hal_tasks_schedule(&cluster->position_update_task, update_interval_ms);
}

void cover_position_update_handler(void *arg) {
  zigbee_cover_cluster *cluster = (zigbee_cover_cluster *)arg;
  cover_update_position(cluster);

  // Reschedule if still moving - update every 1% of effective travel
  if (cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED) {
    cover_schedule_next_position_update(cluster);
  }
}

void cover_auto_stop_handler(void *arg) {
  zigbee_cover_cluster *cluster = (zigbee_cover_cluster *)arg;
  printf("Auto-stop triggered\r\n");

  // Stop the motor
  relay_off(cluster->open_relay);
  relay_off(cluster->close_relay);

  cluster->last_relay_off_time = hal_millis();

  // Update motor position to final position
  uint32_t elapsed_ms = hal_millis() - cluster->movement_start_time;
  uint32_t calibration_ms = cluster->calibration_time * 100;
  uint32_t motor_position_delta = (elapsed_ms * MAX_MOTOR_POSITION) / calibration_ms;

  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    uint32_t new_motor_position = cluster->start_motor_position + motor_position_delta;
    if (new_motor_position > MAX_MOTOR_POSITION) {
      new_motor_position = MAX_MOTOR_POSITION;
    }
    cluster->motor_position = (uint16_t)new_motor_position;
  } else {
    if (motor_position_delta > cluster->start_motor_position) {
      cluster->motor_position = 0;
    } else {
      cluster->motor_position = cluster->start_motor_position - (uint16_t)motor_position_delta;
    }
  }

  cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
  cluster->position = motor_to_cover_position(cluster);

  // Cancel position update task
  hal_tasks_unschedule(&cluster->position_update_task);

  // Send reports
  hal_zigbee_send_report_attr(cluster->endpoint,
                              ZCL_CLUSTER_WINDOW_COVERING,
                              ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE,
                              ZCL_DATA_TYPE_UINT8,
                              &cluster->position,
                              1);
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

void cover_execute_movement(zigbee_cover_cluster *cluster, uint8_t target_cover_pos) {
  uint16_t target_motor_position = cover_to_motor_position(cluster, target_cover_pos);
  
  cover_cancel_movement_tasks(cluster);

  uint32_t duration_ms;
  uint32_t calibration_ms = cluster->calibration_time * 100;
  if (target_motor_position > cluster->motor_position) {
    cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING;
    duration_ms = ((target_motor_position - cluster->motor_position) * calibration_ms) / MAX_MOTOR_POSITION;
  } else {
    cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_CLOSING;
    duration_ms = ((cluster->motor_position - target_motor_position) * calibration_ms) / MAX_MOTOR_POSITION;
  }

  hal_zigbee_notify_attribute_changed(cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING);
  cluster->movement_start_time = hal_millis();
  cluster->start_motor_position = cluster->motor_position;

  relay_t *open_relay = cluster->motor_reversal ? cluster->close_relay : cluster->open_relay;
  relay_t *close_relay = cluster->motor_reversal ? cluster->open_relay : cluster->close_relay;
  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    printf("Moving OPEN: %d%% -> %d%% (motor: %u.%02u%% -> %u.%02u%%, duration: %ums)\r\n",
           cluster->position, target_cover_pos,
           cluster->motor_position / 100, cluster->motor_position % 100,
           target_motor_position / 100, target_motor_position % 100,
           duration_ms);
    relay_on(open_relay);
    relay_off(close_relay);
  } else {
    printf("Moving CLOSE: %d%% -> %d%% (motor: %u.%02u%% -> %u.%02u%%, duration: %ums)\r\n",
           cluster->position, target_cover_pos,
           cluster->motor_position / 100, cluster->motor_position % 100,
           target_motor_position / 100, target_motor_position % 100,
           duration_ms);
    relay_off(open_relay);
    relay_on(close_relay);
  }

  cover_schedule_next_position_update(cluster);
  hal_tasks_schedule(&cluster->stop_task, duration_ms);
}

void cover_safety_delay_handler(void *arg) {
  zigbee_cover_cluster *cluster = (zigbee_cover_cluster *)arg;
  printf("Safety delay expired\r\n");
  
  if (cluster->has_pending_movement) {
    printf("Executing pending movement to %d%%\r\n", cluster->pending_target_position);
    cluster->has_pending_movement = 0;
    cover_execute_movement(cluster, cluster->pending_target_position);
  }
}

void cover_goto_position(zigbee_cover_cluster *cluster, uint8_t target_cover_pos) {
  if (cluster->calibration_time == 0) {
    printf("ERROR: Cannot go-to-position - not calibrated\r\n");
    return;
  }

  if (target_cover_pos > 100) {
    printf("ERROR: Cannot go above 100%%\r\n");
    return;
  }

  uint16_t target_motor_position = cover_to_motor_position(cluster, target_cover_pos);
  if (cluster->motor_position == target_motor_position) {
    printf("Already at target motor position\r\n");
    return;
  }

  // Check if we need to apply safety delay
  uint8_t relays_currently_active = (cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED);
  
  if (relays_currently_active) {
    // Stop relays immediately
    printf("Stopping for safety delay before direction change\r\n");
    cover_cancel_movement_tasks(cluster);
    relay_off(cluster->open_relay);
    relay_off(cluster->close_relay);
    
    if (cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED) {
      cover_update_position(cluster);
    }
    
    cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
    cluster->last_relay_off_time = hal_millis();
    hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                       ZCL_CLUSTER_WINDOW_COVERING,
                                       ZCL_ATTR_WINDOW_COVERING_MOVING);
    
    // Queue the new movement
    cluster->pending_target_position = target_cover_pos;
    cluster->has_pending_movement = 1;
    hal_tasks_schedule(&cluster->safety_delay_task, 500);
    return;
  }
  
  // Check if we're within safety delay period from last relay-off
  if (cluster->last_relay_off_time > 0) {
    uint32_t elapsed_since_off = hal_millis() - cluster->last_relay_off_time;
    if (elapsed_since_off < 500) {
      printf("Within safety delay period, queuing movement (elapsed: %ums)\r\n", elapsed_since_off);
      cluster->pending_target_position = target_cover_pos;
      cluster->has_pending_movement = 1;
      uint32_t remaining_delay = 500 - elapsed_since_off;
      hal_tasks_schedule(&cluster->safety_delay_task, remaining_delay);
      return;
    }
  }
  
  // Safe to execute immediately
  cover_execute_movement(cluster, target_cover_pos);
}

// ============================================================================
// Section 5: Calibration logic
// ============================================================================

void cover_start_calibration_movement(zigbee_cover_cluster *cluster,
                                      uint8_t direction) {
  uint8_t reversed = cluster->motor_reversal;
  relay_t *open_relay = reversed ? cluster->close_relay : cluster->open_relay;
  relay_t *close_relay = reversed ? cluster->open_relay : cluster->close_relay;

  printf("Starting calibration movement: direction=%d\r\n", direction);

  // Cancel any existing tasks
  cover_cancel_movement_tasks(cluster);

  // SAFETY FIRST: Stop everything
  relay_off(close_relay);
  relay_off(open_relay);

  // Record calibration start
  cluster->movement_start_time = hal_millis();
  cluster->calibration_direction = direction;

  // Start movement (no auto-stop in calibration)
  if (direction == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    relay_on(open_relay);
  } else {
    relay_on(close_relay);
  }

  cluster->moving = direction;

  // Schedule safety timeout (120 seconds)
  hal_tasks_schedule(&cluster->calibration_timeout_task, 120000);

  // Notify moving state changed
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

void cover_complete_calibration(zigbee_cover_cluster *cluster) {
  uint32_t measured_ms = hal_millis() - cluster->movement_start_time;
  printf("Calibration complete: measured %u ms\r\n", measured_ms);

  // Convert to 100ms units with rounding
  uint16_t measured_time = (measured_ms + 50) / 100;

  // Validate measurement (minimum 5.0 seconds = 50 units)
  if (measured_time < 50) {
    printf("WARNING: Calibration too short (%u units), rejecting\r\n",
           measured_time);
    // Just stop without saving
    relay_off(cluster->open_relay);
    relay_off(cluster->close_relay);
    cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
    cluster->calibration = 0;
    
    // Send unsolicited report for calibration state
    uint8_t calib_val = 0;
    hal_zigbee_send_report_attr(cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING,
                                ZCL_ATTR_WINDOW_COVERING_CALIBRATION,
                                ZCL_DATA_TYPE_BOOLEAN, &calib_val, 1);
    return;
  }

  if (measured_time > 1200) {
    printf("WARNING: Calibration very long (%u units = %u.%u s)\r\n",
           measured_time, measured_time / 10, measured_time % 10);
  }

  // Cancel timeout
  hal_tasks_unschedule(&cluster->calibration_timeout_task);

  // Stop the motor
  relay_off(cluster->open_relay);
  relay_off(cluster->close_relay);

  cluster->last_relay_off_time = hal_millis();

  // Save calibration time
  cluster->calibration_time = measured_time;
  printf("Calibration saved: %u units (%u.%u seconds)\r\n", measured_time,
         measured_time / 10, measured_time % 10);

  // Set motor position to endpoint based on calibration direction
  if (cluster->calibration_direction ==
      ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    cluster->motor_position = 10000;  // Fully open = 100.00%
    cluster->position = 100;
  } else {
    cluster->motor_position = 0;  // Fully closed = 0.00%
    cluster->position = 0;
  }

  // Exit calibration mode
  cluster->calibration = 0;
  cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;

  // Store to NVM
  cover_cluster_store_attrs_to_nv(cluster);

  // Send unsolicited reports for calibration attributes
  uint8_t calib_val = 0;
  hal_zigbee_send_report_attr(cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING,
                              ZCL_ATTR_WINDOW_COVERING_CALIBRATION,
                              ZCL_DATA_TYPE_BOOLEAN, &calib_val, 1);
  hal_zigbee_send_report_attr(cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING,
                              ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
                              ZCL_DATA_TYPE_UINT16, &cluster->calibration_time,
                              2);

  // Send configured reports for position and moving
  hal_zigbee_notify_attribute_changed(
      cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING,
      ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE);
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

void cover_calibration_timeout_handler(void *arg) {
  zigbee_cover_cluster *cluster = (zigbee_cover_cluster *)arg;
  printf("Calibration TIMEOUT - forcing stop\r\n");

  // Force stop
  relay_off(cluster->open_relay);
  relay_off(cluster->close_relay);

  cluster->last_relay_off_time = hal_millis();
  cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
  cluster->calibration = 0;

  // Send unsolicited report for calibration state
  uint8_t calib_val = 0;
  hal_zigbee_send_report_attr(cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING,
                              ZCL_ATTR_WINDOW_COVERING_CALIBRATION,
                              ZCL_DATA_TYPE_BOOLEAN, &calib_val, 1);
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

// ============================================================================
// Section 6: NVM storage/loading
// ============================================================================

static zigbee_cover_cluster_config nv_config_buffer;

void cover_cluster_store_attrs_to_nv(zigbee_cover_cluster *cluster) {
  nv_config_buffer.motor_reversal = cluster->motor_reversal;
  nv_config_buffer.calibration_time = cluster->calibration_time;
  nv_config_buffer.closed_slack = cluster->closed_slack;
  nv_config_buffer.open_slack = cluster->open_slack;
  nv_config_buffer.motor_position = cluster->motor_position;

  hal_nvm_write(NV_ITEM_COVER_CONFIG(cluster->cover_idx),
                sizeof(zigbee_cover_cluster_config),
                (uint8_t *)&nv_config_buffer);
  printf("Config saved to NVM: calib_time=%u (%u.%us), motor_pos=%u.%02u%%\r\n",
         cluster->calibration_time, cluster->calibration_time / 10,
         cluster->calibration_time % 10,
         cluster->motor_position / 100, cluster->motor_position % 100);
}

void cover_cluster_load_attrs_from_nv(zigbee_cover_cluster *cluster) {
  hal_nvm_status_t st = hal_nvm_read(
      NV_ITEM_COVER_CONFIG(cluster->cover_idx),
      sizeof(zigbee_cover_cluster_config),
      (uint8_t *)&nv_config_buffer);

  if (st != HAL_NVM_SUCCESS) {
    printf("No cover config in NV, using defaults\r\n");
    return;
  }

  cluster->motor_reversal = nv_config_buffer.motor_reversal;
  cluster->calibration_time = nv_config_buffer.calibration_time;
  cluster->closed_slack = nv_config_buffer.closed_slack;
  cluster->open_slack = nv_config_buffer.open_slack;
  cluster->motor_position = nv_config_buffer.motor_position;
  cluster->position = motor_to_cover_position(cluster);
}

void cover_cluster_init(zigbee_cover_cluster *cluster) {
  // Attributes
  cluster->window_covering_type = 0;
  cluster->position = 50; // 50%
  cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
  cluster->motor_reversal = 0;
  cluster->calibration = 0;
  cluster->calibration_time = 300;
  cluster->closed_slack = 0;
  cluster->open_slack = 0;

  // State
  cluster->motor_position = MAX_MOTOR_POSITION / 2;
  cluster->movement_start_time = 0;
  cluster->start_motor_position = 0;
  cluster->calibration_direction = 0;
  cluster->last_relay_off_time = 0;
  cluster->pending_target_position = 0;
  cluster->has_pending_movement = 0;

  hal_tasks_init(&cluster->stop_task);
  cluster->stop_task.handler = cover_auto_stop_handler;
  cluster->stop_task.arg = cluster;

  hal_tasks_init(&cluster->position_update_task);
  cluster->position_update_task.handler = cover_position_update_handler;
  cluster->position_update_task.arg = cluster;

  hal_tasks_init(&cluster->calibration_timeout_task);
  cluster->calibration_timeout_task.handler = cover_calibration_timeout_handler;
  cluster->calibration_timeout_task.arg = cluster;
  
  hal_tasks_init(&cluster->safety_delay_task);
  cluster->safety_delay_task.handler = cover_safety_delay_handler;
  cluster->safety_delay_task.arg = cluster;
}

// ============================================================================
// Section 7: Attribute write handler
// ============================================================================

void cover_cluster_on_write_attr(zigbee_cover_cluster *cluster,
                                 uint16_t attribute_id) {
  switch (attribute_id) {
  case ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL:
  case ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME:
  case ZCL_ATTR_WINDOW_COVERING_CLOSED_SLACK:
  case ZCL_ATTR_WINDOW_COVERING_OPEN_SLACK:
    cover_cluster_store_attrs_to_nv(cluster);
    // Send unsolicited reports for these config changes
    if (attribute_id == ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL) {
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL,
                                  ZCL_DATA_TYPE_BOOLEAN, &cluster->motor_reversal, 1);
    } else if (attribute_id == ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME) {
      hal_zigbee_send_report_attr(
          cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING,
          ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME, ZCL_DATA_TYPE_UINT16,
          &cluster->calibration_time, 2);
    } else if (attribute_id == ZCL_ATTR_WINDOW_COVERING_CLOSED_SLACK) {
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_CLOSED_SLACK,
                                  ZCL_DATA_TYPE_UINT16, &cluster->closed_slack, 2);
    } else if (attribute_id == ZCL_ATTR_WINDOW_COVERING_OPEN_SLACK) {
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_OPEN_SLACK,
                                  ZCL_DATA_TYPE_UINT16, &cluster->open_slack,
                                  2);
    }
    break;

  case ZCL_ATTR_WINDOW_COVERING_CALIBRATION:
    // Calibration attribute written - enter/exit calibration mode
    if (cluster->calibration) {
      printf("Entering calibration mode\r\n");
      // Send unsolicited report
      uint8_t calib_val = 1;
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_CALIBRATION,
                                  ZCL_DATA_TYPE_BOOLEAN, &calib_val, 1);
    } else {
      printf("Exiting calibration mode\r\n");
      // If currently moving, stop
      if (cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED) {
        cover_cancel_movement_tasks(cluster);
        relay_off(cluster->open_relay);
        relay_off(cluster->close_relay);
        cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
        hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                           ZCL_CLUSTER_WINDOW_COVERING,
                                           ZCL_ATTR_WINDOW_COVERING_MOVING);
      }
      // Send unsolicited report
      uint8_t calib_val = 0;
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_CALIBRATION,
                                  ZCL_DATA_TYPE_BOOLEAN, &calib_val, 1);
    }
    break;
  }
}

// ============================================================================
// Section 8: Cluster registration
// ============================================================================

void cover_cluster_add_to_endpoint(zigbee_cover_cluster *cluster,
    hal_zigbee_endpoint *endpoint) {
  cover_cluster_by_endpoint[endpoint->endpoint] = cluster;
  cluster->endpoint = endpoint->endpoint;
  cover_cluster_init(cluster);
  cover_cluster_load_attrs_from_nv(cluster);

  SETUP_ATTR(0, ZCL_ATTR_WINDOW_COVERING_TYPE, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->window_covering_type);
  SETUP_ATTR(1, ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE,
             ZCL_DATA_TYPE_UINT8, ATTR_READONLY, cluster->position);
  SETUP_ATTR(2, ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL, ZCL_DATA_TYPE_BOOLEAN,
             ATTR_WRITABLE, cluster->motor_reversal);
  SETUP_ATTR(3, ZCL_ATTR_WINDOW_COVERING_MOVING, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->moving);
  SETUP_ATTR(4, ZCL_ATTR_WINDOW_COVERING_CALIBRATION, ZCL_DATA_TYPE_BOOLEAN,
             ATTR_WRITABLE, cluster->calibration);
  SETUP_ATTR(5, ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->calibration_time);
  SETUP_ATTR(6, ZCL_ATTR_WINDOW_COVERING_CLOSED_SLACK, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->closed_slack);
  SETUP_ATTR(7, ZCL_ATTR_WINDOW_COVERING_OPEN_SLACK, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->open_slack);

  endpoint->clusters[endpoint->cluster_count].cluster_id =
      ZCL_CLUSTER_WINDOW_COVERING;
  endpoint->clusters[endpoint->cluster_count].attribute_count = 8;
  endpoint->clusters[endpoint->cluster_count].attributes = cluster->attr_infos;
  endpoint->clusters[endpoint->cluster_count].is_server = 1;
  endpoint->clusters[endpoint->cluster_count].cmd_callback =
      cover_cluster_callback_trampoline;
  endpoint->cluster_count++;
}

hal_zigbee_cmd_result_t cover_cluster_callback_trampoline(uint8_t endpoint,
    uint8_t cluster_id,
    uint8_t command_id,
    void *cmd_payload) {
  return cover_cluster_callback(cover_cluster_by_endpoint[endpoint], command_id,
                                cmd_payload);
}

hal_zigbee_cmd_result_t cover_cluster_callback(zigbee_cover_cluster *cluster,
    uint8_t command_id,
    void *cmd_payload) {
  
  printf("Cover command: %d\r\n", command_id);
  
  switch (command_id) {
  case ZCL_CMD_WINDOW_COVERING_UP_OPEN:
    cover_open(cluster);
    break;
  case ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
    cover_close(cluster);
    break;
  case ZCL_CMD_WINDOW_COVERING_STOP:
    cover_stop(cluster);
    break;
  case ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
    if (cmd_payload != NULL) {
      // Payload is a single uint8_t representing percentage (0-100)
      uint8_t target_percentage = *((uint8_t *)cmd_payload);
      printf("Go-to-percentage command: %d%%\r\n", target_percentage);
      cover_goto_position(cluster, target_percentage);
    } else {
      printf("ERROR: Go-to-percentage command with NULL payload\r\n");
      return HAL_ZIGBEE_CMD_SKIPPED;
    }
    break;
  default:
    return HAL_ZIGBEE_CMD_SKIPPED;
  }
  return HAL_ZIGBEE_CMD_PROCESSED;
}
