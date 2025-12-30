#include "cover_output_cluster.h"
#include "cluster_common.h"
#include "consts.h"
#include "device_config/nvm_items.h"
#include "hal/nvm.h"
#include "hal/printf_selector.h"
#include "hal/system.h"
#include "hal/zigbee.h"
#include "base_components/relay.h"

hal_zigbee_cmd_result_t cover_output_cluster_callback(
    zigbee_cover_output_cluster *cluster,
    uint8_t command_id,
    void *cmd_payload);
hal_zigbee_cmd_result_t cover_output_cluster_callback_trampoline(
    uint8_t endpoint,
    uint8_t cluster_id,
    uint8_t command_id,
    void *cmd_payload);

void cover_output_cluster_on_write_attr(zigbee_cover_output_cluster *cluster,
                                          uint16_t attribute_id);

void cover_output_cluster_store_attrs_to_nv(zigbee_cover_output_cluster *cluster);
void cover_output_cluster_load_attrs_from_nv(zigbee_cover_output_cluster *cluster);

zigbee_cover_output_cluster *cover_output_cluster_by_endpoint[10];

void cover_output_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                           uint16_t attribute_id) {
  cover_output_cluster_on_write_attr(cover_output_cluster_by_endpoint[endpoint],
                                       attribute_id);
}

void cover_output_cluster_add_to_endpoint(
    zigbee_cover_output_cluster *cluster,
    hal_zigbee_endpoint *endpoint) {
  cover_output_cluster_by_endpoint[endpoint->endpoint] = cluster;
  cluster->endpoint = endpoint->endpoint;
  cover_output_cluster_load_attrs_from_nv(cluster);

  // Initialize state
  cluster->status = 0;  // stopped
  cluster->window_covering_type = ZCL_WINDOW_COVERING_TYPE_ROLLERSHADE;
  cluster->position = 50;  // Unknown position initially

  // WindowCovering server cluster
  SETUP_ATTR(0, ZCL_ATTR_WINDOW_COVERING_TYPE, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->window_covering_type);
  SETUP_ATTR(1, ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE, ZCL_DATA_TYPE_UINT8,
             ATTR_READONLY, cluster->position);
  SETUP_ATTR(2, ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL, ZCL_DATA_TYPE_BOOLEAN,
             ATTR_WRITABLE, cluster->reversal);
  SETUP_ATTR(3, ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, ZCL_DATA_TYPE_ENUM8,
             ATTR_READONLY, cluster->status);

  endpoint->clusters[endpoint->cluster_count].cluster_id = ZCL_CLUSTER_WINDOW_COVERING;
  endpoint->clusters[endpoint->cluster_count].attribute_count = 4;
  endpoint->clusters[endpoint->cluster_count].attributes = cluster->attr_infos;
  endpoint->clusters[endpoint->cluster_count].is_server = 1;
  endpoint->clusters[endpoint->cluster_count].cmd_callback =
      cover_output_cluster_callback_trampoline;
  endpoint->cluster_count++;
}

hal_zigbee_cmd_result_t cover_output_cluster_callback_trampoline(
    uint8_t endpoint,
    uint8_t cluster_id,
    uint8_t command_id,
    void *cmd_payload) {
  return cover_output_cluster_callback(cover_output_cluster_by_endpoint[endpoint],
                                         command_id, cmd_payload);
}

hal_zigbee_cmd_result_t cover_output_cluster_callback(
    zigbee_cover_output_cluster *cluster,
    uint8_t command_id,
    void *cmd_payload) {
  
  printf("Cover output command: %d\r\n", command_id);
  
  switch (command_id) {
  case ZCL_CMD_WINDOW_COVERING_UP_OPEN:
    cover_output_up(cluster);
    break;
  case ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
    cover_output_down(cluster);
    break;
  case ZCL_CMD_WINDOW_COVERING_STOP:
    cover_output_stop(cluster);
    break;
  case ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE:
  case ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
    // Not implemented in MVP - just stop
    printf("Go-to-position not implemented, stopping\r\n");
    cover_output_stop(cluster);
    break;
  default:
    return HAL_ZIGBEE_CMD_SKIPPED;
  }
  return HAL_ZIGBEE_CMD_PROCESSED;
}

void cover_output_up(zigbee_cover_output_cluster *cluster) {
  uint8_t reversed = cluster->reversal;
  relay_t *up = reversed ? cluster->down_relay : cluster->up_relay;
  relay_t *down = reversed ? cluster->up_relay : cluster->down_relay;
  
  printf("Cover UP (reversed=%d)\r\n", reversed);
  
  // SAFETY FIRST: Stop everything
  relay_off(down);
  relay_off(up);
  
  // Now safe to activate up relay
  relay_on(up);
  
  // Update status to OPENING (1)
  cluster->status = 1;
  
  // Report status change
  hal_zigbee_send_report_attr(cluster->endpoint, 
                         ZCL_CLUSTER_WINDOW_COVERING,
                         ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, 0, 0, 0);
}

void cover_output_down(zigbee_cover_output_cluster *cluster) {
  uint8_t reversed = cluster->reversal;
  relay_t *up = reversed ? cluster->down_relay : cluster->up_relay;
  relay_t *down = reversed ? cluster->up_relay : cluster->down_relay;
  
  printf("Cover DOWN (reversed=%d)\r\n", reversed);
  
  // SAFETY FIRST: Stop everything
  relay_off(up);
  relay_off(down);
  
  // Now safe to activate down relay
  relay_on(down);
  
  // Update status to CLOSING (2)
  cluster->status = 2;
  
  // Report status change
  hal_zigbee_send_report_attr(cluster->endpoint, 
                         ZCL_CLUSTER_WINDOW_COVERING,
                         ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, 0, 0, 0);
}

void cover_output_stop(zigbee_cover_output_cluster *cluster) {
  printf("Cover STOP\r\n");
  
  // Stop both relays
  relay_off(cluster->up_relay);
  relay_off(cluster->down_relay);
  
  // Update status to STOPPED (0)
  cluster->status = 0;
  
  // Report status change
  hal_zigbee_send_report_attr(cluster->endpoint, 
                         ZCL_CLUSTER_WINDOW_COVERING,
                         ZCL_ATTR_WINDOW_COVERING_OPERATIONAL_STATUS, 0, 0, 0);
}

void cover_output_cluster_store_attrs_to_nv(zigbee_cover_output_cluster *cluster) {
  hal_nvm_write(NVM_COVER_OUTPUT_0_CONFIG + cluster->output_idx,
                1, (uint8_t *)&cluster->reversal);
}

void cover_output_cluster_load_attrs_from_nv(zigbee_cover_output_cluster *cluster) {
  uint8_t read_status = hal_nvm_read(
      NVM_COVER_OUTPUT_0_CONFIG + cluster->output_idx, 
      1, (uint8_t *)&cluster->reversal);
  if (read_status != 0) {
    // Default values
    cluster->reversal = 0;  // No reversal
  }
}

void cover_output_cluster_on_write_attr(zigbee_cover_output_cluster *cluster,
                                          uint16_t attribute_id) {
  switch (attribute_id) {
  case ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL:
    cover_output_cluster_store_attrs_to_nv(cluster);
    break;
  }
}


