#include "cover_cluster.h"
#include "cluster_common.h"
#include "consts.h"
#include "device_config/nvm_items.h"
#include "hal/nvm.h"
#include "hal/printf_selector.h"
#include "hal/system.h"
#include "hal/zigbee.h"
#include "base_components/relay.h"

hal_zigbee_cmd_result_t cover_cluster_callback(
    zigbee_cover_cluster *cluster,
    uint8_t command_id,
    void *cmd_payload);
hal_zigbee_cmd_result_t cover_cluster_callback_trampoline(
    uint8_t endpoint,
    uint8_t cluster_id,
    uint8_t command_id,
    void *cmd_payload);

void cover_cluster_on_write_attr(zigbee_cover_cluster *cluster,
                                          uint16_t attribute_id);

void cover_cluster_store_attrs_to_nv(zigbee_cover_cluster *cluster);
void cover_cluster_load_attrs_from_nv(zigbee_cover_cluster *cluster);

zigbee_cover_cluster *cover_cluster_by_endpoint[10];

void cover_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                           uint16_t attribute_id) {
  cover_cluster_on_write_attr(cover_cluster_by_endpoint[endpoint],
                                       attribute_id);
}

void cover_cluster_add_to_endpoint(
    zigbee_cover_cluster *cluster,
    hal_zigbee_endpoint *endpoint) {
  cover_cluster_by_endpoint[endpoint->endpoint] = cluster;
  cluster->endpoint = endpoint->endpoint;
  cover_cluster_load_attrs_from_nv(cluster);

  // Initialize state
  cluster->status = 0;  // stopped
  cluster->window_covering_type = ZCL_WINDOW_COVERING_TYPE_ROLLERSHADE;
  cluster->position = 50;  // Unknown position initially
  
  // Initialize calibration attributes with defaults
  cluster->calibration = 0;        // Not calibrating
  cluster->calibration_time = 0;   // Not calibrated yet
  cluster->open_delay = 0;         // No delay
  cluster->close_delay = 0;        // No delay

  // WindowCovering server cluster
  SETUP_ATTR(0, ZCL_ATTR_WINDOW_COVERING_TYPE, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->window_covering_type);
  SETUP_ATTR(1, ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE, ZCL_DATA_TYPE_UINT8,
             ATTR_READONLY, cluster->position);
  SETUP_ATTR(2, ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL, ZCL_DATA_TYPE_BOOLEAN,
             ATTR_WRITABLE, cluster->reversal);
  SETUP_ATTR(3, ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->status);
  SETUP_ATTR(4, ZCL_ATTR_WINDOW_COVERING_CALIBRATION, ZCL_DATA_TYPE_BOOLEAN,
             ATTR_WRITABLE, cluster->calibration);
  SETUP_ATTR(5, ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->calibration_time);
  SETUP_ATTR(6, ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->open_delay);
  SETUP_ATTR(7, ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY, ZCL_DATA_TYPE_UINT16,
             ATTR_WRITABLE, cluster->close_delay);

  endpoint->clusters[endpoint->cluster_count].cluster_id = ZCL_CLUSTER_WINDOW_COVERING;
  endpoint->clusters[endpoint->cluster_count].attribute_count = 8;
  endpoint->clusters[endpoint->cluster_count].attributes = cluster->attr_infos;
  endpoint->clusters[endpoint->cluster_count].is_server = 1;
  endpoint->clusters[endpoint->cluster_count].cmd_callback =
      cover_cluster_callback_trampoline;
  endpoint->cluster_count++;
}

hal_zigbee_cmd_result_t cover_cluster_callback_trampoline(
    uint8_t endpoint,
    uint8_t cluster_id,
    uint8_t command_id,
    void *cmd_payload) {
  return cover_cluster_callback(cover_cluster_by_endpoint[endpoint],
                                         command_id, cmd_payload);
}

hal_zigbee_cmd_result_t cover_cluster_callback(
    zigbee_cover_cluster *cluster,
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
  case ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE:
  case ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
    // Not implemented in MVP - just stop
    printf("Go-to-position not implemented, stopping\r\n");
    cover_stop(cluster);
    break;
  default:
    return HAL_ZIGBEE_CMD_SKIPPED;
  }
  return HAL_ZIGBEE_CMD_PROCESSED;
}

void cover_open(zigbee_cover_cluster *cluster) {
  uint8_t reversed = cluster->reversal;
  relay_t *open = reversed ? cluster->close_relay : cluster->open_relay;
  relay_t *close = reversed ? cluster->open_relay : cluster->close_relay;
  
  printf("Cover OPEN (reversed=%d)\r\n", reversed);
  
  // SAFETY FIRST: Stop everything
  relay_off(close);
  relay_off(open);
  
  // Now safe to activate open relay
  relay_on(open);
  
  // Update status to OPENING (1)
  cluster->status = 1;
  
  // Report status change
  hal_zigbee_send_report_attr(cluster->endpoint, 
                         ZCL_CLUSTER_WINDOW_COVERING,
                         ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, 0, 0, 0);
}

void cover_close(zigbee_cover_cluster *cluster) {
  uint8_t reversed = cluster->reversal;
  relay_t *open = reversed ? cluster->close_relay : cluster->open_relay;
  relay_t *close = reversed ? cluster->open_relay : cluster->close_relay;
  
  printf("Cover CLOSE (reversed=%d)\r\n", reversed);
  
  // SAFETY FIRST: Stop everything
  relay_off(open);
  relay_off(close);
  
  // Now safe to activate close relay
  relay_on(close);
  
  // Update status to CLOSING (2)
  cluster->status = 2;
  
  // Report status change
  hal_zigbee_send_report_attr(cluster->endpoint, 
                         ZCL_CLUSTER_WINDOW_COVERING,
                         ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, 0, 0, 0);
}

void cover_stop(zigbee_cover_cluster *cluster) {
  printf("Cover STOP\r\n");
  
  // Stop both relays
  relay_off(cluster->open_relay);
  relay_off(cluster->close_relay);
  
  // Update status to STOPPED (0)
  cluster->status = 0;
  
  // Report status change
  hal_zigbee_send_report_attr(cluster->endpoint, 
                         ZCL_CLUSTER_WINDOW_COVERING,
                         ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, 0, 0, 0);
}

void cover_cluster_store_attrs_to_nv(zigbee_cover_cluster *cluster) {
  // Store reversal (1 byte) + calibration_time (2 bytes) + open_delay (2 bytes) + close_delay (2 bytes) = 7 bytes
  uint8_t data[7];
  data[0] = cluster->reversal;
  data[1] = cluster->calibration_time & 0xFF;
  data[2] = (cluster->calibration_time >> 8) & 0xFF;
  data[3] = cluster->open_delay & 0xFF;
  data[4] = (cluster->open_delay >> 8) & 0xFF;
  data[5] = cluster->close_delay & 0xFF;
  data[6] = (cluster->close_delay >> 8) & 0xFF;
  
  hal_nvm_write(NVM_COVER_0_CONFIG + cluster->output_idx, 7, data);
}

void cover_cluster_load_attrs_from_nv(zigbee_cover_cluster *cluster) {
  uint8_t data[7];
  uint8_t read_status = hal_nvm_read(
      NVM_COVER_0_CONFIG + cluster->output_idx, 7, data);
  if (read_status != 0) {
    // Default values
    cluster->reversal = 0;           // No reversal
    cluster->calibration_time = 0;   // Not calibrated
    cluster->open_delay = 0;         // No delay
    cluster->close_delay = 0;        // No delay
  } else {
    cluster->reversal = data[0];
    cluster->calibration_time = data[1] | (data[2] << 8);
    cluster->open_delay = data[3] | (data[4] << 8);
    cluster->close_delay = data[5] | (data[6] << 8);
  }
}

void cover_cluster_on_write_attr(zigbee_cover_cluster *cluster,
                                          uint16_t attribute_id) {
  switch (attribute_id) {
  case ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL:
  case ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME:
  case ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY:
  case ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY:
    cover_cluster_store_attrs_to_nv(cluster);
    break;
  case ZCL_ATTR_WINDOW_COVERING_CALIBRATION:
    // Calibration attribute is not persisted - it's a command to start calibration
    // TODO: Implement calibration logic
    break;
  }
}


