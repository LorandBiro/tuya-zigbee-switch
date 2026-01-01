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

// Timing & position helpers
void cover_update_position(zigbee_cover_cluster *cluster);
void cover_position_update_handler(void *arg);
void cover_auto_stop_handler(void *arg);
void cover_cancel_movement_tasks(zigbee_cover_cluster *cluster);

// Calibration functions
void cover_start_calibration_movement(zigbee_cover_cluster *cluster,
                                      uint8_t direction);
void cover_complete_calibration(zigbee_cover_cluster *cluster);
void cover_calibration_timeout_handler(void *arg);

// Movement control
void cover_start_movement(zigbee_cover_cluster *cluster, uint8_t direction);
void cover_goto_position(zigbee_cover_cluster *cluster, uint8_t target_position);

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

  cover_start_movement(cluster, ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING);
}

void cover_close(zigbee_cover_cluster *cluster) {
  printf("Cover CLOSE command\r\n");

  // If in calibration mode, handle specially
  if (cluster->calibration) {
    cover_start_calibration_movement(cluster,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING_CLOSING);
    return;
  }

  cover_start_movement(cluster, ZCL_ATTR_WINDOW_COVERING_MOVING_CLOSING);
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
}

void cover_update_position(zigbee_cover_cluster *cluster) {
  // Don't update if not moving or not calibrated
  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED ||
      cluster->calibration_time == 0) {
    return;
  }

  // All timing values are in 100ms units, convert to milliseconds
  uint32_t calibration_ms = (uint32_t)cluster->calibration_time * 100;
  uint32_t open_delay_ms = (uint32_t)cluster->open_delay * 100;
  uint32_t close_delay_ms = (uint32_t)cluster->close_delay * 100;

  // Effective travel time excludes delays
  uint32_t effective_time_ms = calibration_ms - open_delay_ms - close_delay_ms;
  if (effective_time_ms == 0) {
    effective_time_ms = 1; // Prevent division by zero
  }

  uint32_t elapsed_ms = hal_millis() - cluster->movement_start_time;
  uint8_t new_position = cluster->position;

  if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    // Opening: 0% (closed) -> 100% (open)
    if (elapsed_ms < open_delay_ms) {
      // Still in delay phase
      new_position = cluster->start_position;
    } else {
      uint32_t travel_ms = elapsed_ms - open_delay_ms;
      // Calculate position: start + (progress * distance / effective_time)
      uint32_t distance = 100 - cluster->start_position;
      uint32_t progress = (travel_ms * distance) / effective_time_ms;
      new_position = cluster->start_position + progress;
      if (new_position > 100)
        new_position = 100;
    }
  } else if (cluster->moving == ZCL_ATTR_WINDOW_COVERING_MOVING_CLOSING) {
    // Closing: 100% (open) -> 0% (closed)
    if (elapsed_ms < close_delay_ms) {
      // Still in delay phase
      new_position = cluster->start_position;
    } else {
      uint32_t travel_ms = elapsed_ms - close_delay_ms;
      // Calculate position: start - (progress * distance / effective_time)
      uint32_t distance = cluster->start_position;
      uint32_t progress = (travel_ms * distance) / effective_time_ms;
      if (progress > cluster->start_position) {
        new_position = 0;
      } else {
        new_position = cluster->start_position - progress;
      }
    }
  }

  // Only update if position changed
  if (new_position != cluster->position) {
    cluster->position = new_position;
    printf("Position updated: %d%%\r\n", new_position);
    hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                       ZCL_CLUSTER_WINDOW_COVERING,
                                       ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE);
  }
}

void cover_position_update_handler(void *arg) {
  zigbee_cover_cluster *cluster = (zigbee_cover_cluster *)arg;
  cover_update_position(cluster);

  // Reschedule if still moving
  if (cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED) {
    hal_tasks_schedule(&cluster->position_update_task, 1000); // Update every 1 second
  }
}

void cover_auto_stop_handler(void *arg) {
  zigbee_cover_cluster *cluster = (zigbee_cover_cluster *)arg;
  printf("Auto-stop triggered\r\n");

  // Stop the motor
  relay_off(cluster->open_relay);
  relay_off(cluster->close_relay);

  // Update to target position
  cluster->position = cluster->target_position;
  cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;

  // Cancel position update task
  hal_tasks_unschedule(&cluster->position_update_task);

  // Send unsolicited position report when auto-stopping
  hal_zigbee_send_report_attr(cluster->endpoint,
                              ZCL_CLUSTER_WINDOW_COVERING,
                              ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE,
                              ZCL_DATA_TYPE_UINT8,
                              &cluster->position,
                              1);
  // Notify moving state changed (for configured reporting)
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

void cover_start_movement(zigbee_cover_cluster *cluster, uint8_t direction) {
  uint8_t reversed = cluster->reversal;
  relay_t *open_relay = reversed ? cluster->close_relay : cluster->open_relay;
  relay_t *close_relay = reversed ? cluster->open_relay : cluster->close_relay;

  // Cancel any existing movement tasks
  cover_cancel_movement_tasks(cluster);

  // SAFETY FIRST: Stop everything
  relay_off(close_relay);
  relay_off(open_relay);

  // Record starting state
  cluster->movement_start_time = hal_millis();
  cluster->start_position = cluster->position;

  if (direction == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    printf("Starting OPEN movement (reversed=%d, pos=%d)\r\n", reversed,
           cluster->position);
    cluster->target_position = 100;
    relay_on(open_relay);
  } else {
    printf("Starting CLOSE movement (reversed=%d, pos=%d)\r\n", reversed,
           cluster->position);
    cluster->target_position = 0;
    relay_on(close_relay);
  }

  cluster->moving = direction;

  // Calculate duration and schedule auto-stop
  if (cluster->calibration_time > 0) {
    uint32_t duration_ms = (uint32_t)cluster->calibration_time * 100;
    
    // Adjust duration based on current position
    if (direction == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
      // Opening: only travel the remaining distance
      duration_ms = (duration_ms * (100 - cluster->start_position)) / 100;
    } else {
      // Closing: only travel from current position to 0
      duration_ms = (duration_ms * cluster->start_position) / 100;
    }

    printf("Auto-stop scheduled in %u ms\r\n", duration_ms);
    cluster->stop_task.handler = cover_auto_stop_handler;
    cluster->stop_task.arg = cluster;
    hal_tasks_init(&cluster->stop_task);
    hal_tasks_schedule(&cluster->stop_task, duration_ms);

    // Schedule periodic position updates
    cluster->position_update_task.handler = cover_position_update_handler;
    cluster->position_update_task.arg = cluster;
    hal_tasks_init(&cluster->position_update_task);
    hal_tasks_schedule(&cluster->position_update_task, 1000); // Every 1 second
  }

  // Notify moving state changed
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

void cover_goto_position(zigbee_cover_cluster *cluster, uint8_t target_position) {
  printf("Go-to-position: target=%d%%, current=%d%%\r\n", target_position, cluster->position);

  // Validate calibration
  if (cluster->calibration_time == 0) {
    printf("ERROR: Cannot go-to-position - not calibrated\r\n");
    return;
  }

  // Clamp target to valid range
  if (target_position > 100) {
    target_position = 100;
  }

  // Check if already at target position
  if (cluster->position == target_position) {
    printf("Already at target position\r\n");
    // If currently moving, stop it
    if (cluster->moving != ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED) {
      cover_stop(cluster);
    }
    return;
  }

  // Determine direction based on current vs target position
  uint8_t direction;
  if (target_position > cluster->position) {
    // Need to open
    direction = ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING;
  } else {
    // Need to close
    direction = ZCL_ATTR_WINDOW_COVERING_MOVING_CLOSING;
  }

  uint8_t reversed = cluster->reversal;
  relay_t *open_relay = reversed ? cluster->close_relay : cluster->open_relay;
  relay_t *close_relay = reversed ? cluster->open_relay : cluster->close_relay;

  // Cancel any existing movement tasks
  cover_cancel_movement_tasks(cluster);

  // SAFETY FIRST: Stop everything
  relay_off(close_relay);
  relay_off(open_relay);

  // Record starting state
  cluster->movement_start_time = hal_millis();
  cluster->start_position = cluster->position;
  cluster->target_position = target_position;

  // Start movement in the appropriate direction
  if (direction == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    printf("Moving OPEN: %d%% -> %d%%\r\n", cluster->position, target_position);
    relay_on(open_relay);
  } else {
    printf("Moving CLOSE: %d%% -> %d%%\r\n", cluster->position, target_position);
    relay_on(close_relay);
  }

  cluster->moving = direction;

  // Calculate duration based on distance to travel
  // All timing values are in 100ms units, convert to milliseconds
  uint32_t calibration_ms = (uint32_t)cluster->calibration_time * 100;
  uint32_t open_delay_ms = (uint32_t)cluster->open_delay * 100;
  uint32_t close_delay_ms = (uint32_t)cluster->close_delay * 100;

  // Effective travel time (excludes delays)
  uint32_t effective_time_ms = calibration_ms - open_delay_ms - close_delay_ms;
  if (effective_time_ms == 0) {
    effective_time_ms = 1; // Prevent division by zero
  }

  // Calculate distance to travel as percentage
  uint8_t distance;
  if (target_position > cluster->start_position) {
    distance = target_position - cluster->start_position;
  } else {
    distance = cluster->start_position - target_position;
  }

  // Duration = (distance / 100) * effective_time + delay
  uint32_t duration_ms = (effective_time_ms * distance) / 100;
  
  // Add appropriate delay
  if (direction == ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    duration_ms += open_delay_ms;
  } else {
    duration_ms += close_delay_ms;
  }

  printf("Go-to-position scheduled: %u ms to reach %d%%\r\n", duration_ms, target_position);

  // Schedule auto-stop at target position
  cluster->stop_task.handler = cover_auto_stop_handler;
  cluster->stop_task.arg = cluster;
  hal_tasks_init(&cluster->stop_task);
  hal_tasks_schedule(&cluster->stop_task, duration_ms);

  // Schedule periodic position updates
  cluster->position_update_task.handler = cover_position_update_handler;
  cluster->position_update_task.arg = cluster;
  hal_tasks_init(&cluster->position_update_task);
  hal_tasks_schedule(&cluster->position_update_task, 1000); // Every 1 second

  // Notify moving state changed
  hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                     ZCL_CLUSTER_WINDOW_COVERING,
                                     ZCL_ATTR_WINDOW_COVERING_MOVING);
}

// ============================================================================
// Section 5: Calibration logic
// ============================================================================

void cover_start_calibration_movement(zigbee_cover_cluster *cluster,
                                      uint8_t direction) {
  uint8_t reversed = cluster->reversal;
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
  cluster->calibration_timeout_task.handler = cover_calibration_timeout_handler;
  cluster->calibration_timeout_task.arg = cluster;
  hal_tasks_init(&cluster->calibration_timeout_task);
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
    printf("WARNING: Calibration very long (%u units = %.1f s)\r\n",
           measured_time, measured_time / 10.0f);
  }

  // Cancel timeout
  hal_tasks_unschedule(&cluster->calibration_timeout_task);

  // Stop the motor
  relay_off(cluster->open_relay);
  relay_off(cluster->close_relay);

  // Save calibration time
  cluster->calibration_time = measured_time;
  printf("Calibration saved: %u units (%.1f seconds)\r\n", measured_time,
         measured_time / 10.0f);

  // Set position to endpoint based on direction
  if (cluster->calibration_direction ==
      ZCL_ATTR_WINDOW_COVERING_MOVING_OPENING) {
    cluster->position = 100; // Fully open
  } else {
    cluster->position = 0; // Fully closed
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

void cover_cluster_store_attrs_to_nv(zigbee_cover_cluster *cluster) {
  // Store reversal (1 byte) + calibration_time (2 bytes) + open_delay (2
  // bytes) + close_delay (2 bytes) = 7 bytes
  uint8_t data[7];
  data[0] = cluster->reversal;
  data[1] = cluster->calibration_time & 0xFF;
  data[2] = (cluster->calibration_time >> 8) & 0xFF;
  data[3] = cluster->open_delay & 0xFF;
  data[4] = (cluster->open_delay >> 8) & 0xFF;
  data[5] = cluster->close_delay & 0xFF;
  data[6] = (cluster->close_delay >> 8) & 0xFF;

  hal_nvm_write(NVM_COVER_0_CONFIG + cluster->output_idx, 7, data);
  printf("Config saved to NVM: calib_time=%u (%.1fs)\r\n",
         cluster->calibration_time, cluster->calibration_time / 10.0f);
}

void cover_cluster_load_attrs_from_nv(zigbee_cover_cluster *cluster) {
  uint8_t data[7];
  uint8_t read_status =
      hal_nvm_read(NVM_COVER_0_CONFIG + cluster->output_idx, 7, data);
  if (read_status != 0) {
    // Default values (all timing values are in 100ms units)
    cluster->reversal = 0;         // No reversal
    cluster->calibration_time = 0; // Not calibrated
    cluster->open_delay = 0;       // No delay
    cluster->close_delay = 0;      // No delay
  } else {
    cluster->reversal = data[0];
    cluster->calibration_time = data[1] | (data[2] << 8);
    cluster->open_delay = data[3] | (data[4] << 8);
    cluster->close_delay = data[5] | (data[6] << 8);
    printf("Config loaded from NVM: calib_time=%u (%.1fs)\r\n",
           cluster->calibration_time, cluster->calibration_time / 10.0f);
  }

  // Initialize runtime state
  cluster->moving = ZCL_ATTR_WINDOW_COVERING_MOVING_STOPPED;
  cluster->calibration = 0;
  cluster->movement_start_time = 0;
  cluster->start_position = 0;
  cluster->target_position = 0;
  cluster->calibration_direction = 0;

  // Initialize tasks
  hal_tasks_init(&cluster->stop_task);
  hal_tasks_init(&cluster->position_update_task);
  hal_tasks_init(&cluster->calibration_timeout_task);
}

// ============================================================================
// Section 7: Attribute write handler
// ============================================================================

void cover_cluster_on_write_attr(zigbee_cover_cluster *cluster,
                                 uint16_t attribute_id) {
  switch (attribute_id) {
  case ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL:
  case ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME:
  case ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY:
  case ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY:
    cover_cluster_store_attrs_to_nv(cluster);
    // Send unsolicited reports for these config changes
    if (attribute_id == ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL) {
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL,
                                  ZCL_DATA_TYPE_BOOLEAN, &cluster->reversal, 1);
    } else if (attribute_id == ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME) {
      hal_zigbee_send_report_attr(
          cluster->endpoint, ZCL_CLUSTER_WINDOW_COVERING,
          ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME, ZCL_DATA_TYPE_UINT16,
          &cluster->calibration_time, 2);
    } else if (attribute_id == ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY) {
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY,
                                  ZCL_DATA_TYPE_UINT16, &cluster->open_delay, 2);
    } else if (attribute_id == ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY) {
      hal_zigbee_send_report_attr(cluster->endpoint,
                                  ZCL_CLUSTER_WINDOW_COVERING,
                                  ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY,
                                  ZCL_DATA_TYPE_UINT16, &cluster->close_delay,
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
  cover_cluster_load_attrs_from_nv(cluster);

  SETUP_ATTR(0, ZCL_ATTR_WINDOW_COVERING_TYPE, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->window_covering_type);
  SETUP_ATTR(1, ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE,
             ZCL_DATA_TYPE_UINT8, ATTR_READONLY, cluster->position);
  SETUP_ATTR(2, ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL, ZCL_DATA_TYPE_BOOLEAN,
             ATTR_WRITABLE, cluster->reversal);
  SETUP_ATTR(3, ZCL_ATTR_WINDOW_COVERING_MOVING, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->moving);
  SETUP_ATTR(4, ZCL_ATTR_WINDOW_COVERING_CALIBRATION, ZCL_DATA_TYPE_BOOLEAN,
             ATTR_WRITABLE, cluster->calibration);
  SETUP_ATTR(5, ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->calibration_time);
  SETUP_ATTR(6, ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->open_delay);
  SETUP_ATTR(7, ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->close_delay);

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
