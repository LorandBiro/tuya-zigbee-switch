#include "cover_switch_cluster.h"
#include "basic_cluster.h"
#include "cluster_common.h"
#include "consts.h"
#include "device_config/nvm_items.h"
#include "hal/nvm.h"
#include "hal/printf_selector.h"
#include "hal/system.h"
#include "hal/tasks.h"
#include "hal/zigbee.h"
#include "cover_cluster.h"
#include "zigbee_commands.h"

static const uint8_t multistate_out_of_service = 0;
static const uint8_t multistate_flags = 0;
static const uint16_t multistate_num_of_states = 6;

extern zigbee_basic_cluster basic_cluster;
extern zigbee_cover_cluster cover_clusters[];
extern uint8_t cover_clusters_cnt;
extern uint8_t switch_clusters_cnt;
extern uint8_t relay_clusters_cnt;

void cover_switch_cluster_on_open_button_press(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_open_button_release(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_open_button_long_press(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_close_button_press(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_close_button_release(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_close_button_long_press(zigbee_cover_switch_cluster *cluster);

zigbee_cover_switch_cluster *cover_switch_cluster_by_endpoint[10];

static zigbee_cover_switch_cluster_config nv_config_buffer;

void cover_switch_cluster_init(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_store_attrs_to_nv(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_load_attrs_from_nv(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_write_attr(zigbee_cover_switch_cluster *cluster,
                                         uint16_t attribute_id);

void cover_switch_cluster_report_action(zigbee_cover_switch_cluster *cluster);
void cover_switch_trigger_output(zigbee_cover_switch_cluster *cluster, uint8_t command);

void cover_switch_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                          uint16_t attribute_id) {
  cover_switch_cluster_on_write_attr(cover_switch_cluster_by_endpoint[endpoint],
                                      attribute_id);
}

void cover_switch_cluster_add_to_endpoint(zigbee_cover_switch_cluster *cluster,
                                           hal_zigbee_endpoint *endpoint) {
  cover_switch_cluster_by_endpoint[endpoint->endpoint] = cluster;
  cluster->endpoint = endpoint->endpoint;
  cover_switch_cluster_init(cluster);
  cover_switch_cluster_load_attrs_from_nv(cluster);

  cluster->open_button->on_press =
      (ev_button_callback_t)cover_switch_cluster_on_open_button_press;
  cluster->open_button->on_release =
      (ev_button_callback_t)cover_switch_cluster_on_open_button_release;
  cluster->open_button->on_long_press =
      (ev_button_callback_t)cover_switch_cluster_on_open_button_long_press;
  cluster->open_button->callback_param = cluster;

  cluster->close_button->on_press =
      (ev_button_callback_t)cover_switch_cluster_on_close_button_press;
  cluster->close_button->on_release =
      (ev_button_callback_t)cover_switch_cluster_on_close_button_release;
  cluster->close_button->on_long_press =
      (ev_button_callback_t)cover_switch_cluster_on_close_button_long_press;
  cluster->close_button->callback_param = cluster;

  // Configuration attributes on CoverSwitchConfig SERVER cluster (manufacturer-specific)
  SETUP_ATTR_FOR_TABLE(cluster->config_attr_infos, 0,
                      ZCL_ATTR_COVER_SWITCH_CONFIG_SWITCH_TYPE, ZCL_DATA_TYPE_ENUM8,
                      ATTR_WRITABLE, cluster->switch_type);
  SETUP_ATTR_FOR_TABLE(cluster->config_attr_infos, 1,
                      ZCL_ATTR_COVER_SWITCH_CONFIG_OUTPUT_INDEX, ZCL_DATA_TYPE_UINT8,
                      ATTR_WRITABLE, cluster->output_index);
  SETUP_ATTR_FOR_TABLE(cluster->config_attr_infos, 2,
                      ZCL_ATTR_COVER_SWITCH_CONFIG_REVERSAL, ZCL_DATA_TYPE_BOOLEAN,
                      ATTR_WRITABLE, cluster->reversal);
  SETUP_ATTR_FOR_TABLE(cluster->config_attr_infos, 3,
                      ZCL_ATTR_COVER_SWITCH_CONFIG_LOCAL_MODE, ZCL_DATA_TYPE_ENUM8,
                      ATTR_WRITABLE, cluster->local_mode);
  SETUP_ATTR_FOR_TABLE(cluster->config_attr_infos, 4,
                      ZCL_ATTR_COVER_SWITCH_CONFIG_BINDED_MODE, ZCL_DATA_TYPE_ENUM8,
                      ATTR_WRITABLE, cluster->binded_mode);
  SETUP_ATTR_FOR_TABLE(cluster->config_attr_infos, 5,
                      ZCL_ATTR_COVER_SWITCH_CONFIG_LONG_PRESS_DUR, ZCL_DATA_TYPE_UINT16,
                      ATTR_WRITABLE, cluster->open_button->long_press_duration_ms);

  // CoverSwitchConfig SERVER cluster (manufacturer-specific)
  endpoint->clusters[endpoint->cluster_count].cluster_id = ZCL_CLUSTER_COVER_SWITCH_CONFIG;
  endpoint->clusters[endpoint->cluster_count].attribute_count = 6;
  endpoint->clusters[endpoint->cluster_count].attributes = cluster->config_attr_infos;
  endpoint->clusters[endpoint->cluster_count].is_server = 1;
  endpoint->cluster_count++;

  // WindowCovering CLIENT cluster (for binding to other devices)
  endpoint->clusters[endpoint->cluster_count].cluster_id = ZCL_CLUSTER_WINDOW_COVERING;
  endpoint->clusters[endpoint->cluster_count].attribute_count = 0;
  endpoint->clusters[endpoint->cluster_count].attributes = NULL;
  endpoint->clusters[endpoint->cluster_count].is_server = 0;  // CLIENT
  endpoint->cluster_count++;

  // MultiStateInput for button press action reporting
  SETUP_ATTR_FOR_TABLE(cluster->multistate_attr_infos, 0,
                      ZCL_ATTR_MULTISTATE_INPUT_NUM_OF_STATES, ZCL_DATA_TYPE_UINT16,
                      ATTR_READONLY, multistate_num_of_states);
  SETUP_ATTR_FOR_TABLE(cluster->multistate_attr_infos, 1,
                      ZCL_ATTR_MULTISTATE_INPUT_OUT_OF_SERVICE, ZCL_DATA_TYPE_BOOLEAN,
                      ATTR_READONLY, multistate_out_of_service);
  SETUP_ATTR_FOR_TABLE(cluster->multistate_attr_infos, 2,
                      ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE, ZCL_DATA_TYPE_UINT16,
                      ATTR_READONLY, cluster->multistate_state);
  SETUP_ATTR_FOR_TABLE(cluster->multistate_attr_infos, 3,
                      ZCL_ATTR_MULTISTATE_INPUT_STATUS_FLAGS, ZCL_DATA_TYPE_BITMAP8,
                      ATTR_READONLY, multistate_flags);

  endpoint->clusters[endpoint->cluster_count].cluster_id = ZCL_CLUSTER_MULTISTATE_INPUT_BASIC;
  endpoint->clusters[endpoint->cluster_count].attribute_count = 4;
  endpoint->clusters[endpoint->cluster_count].attributes = cluster->multistate_attr_infos;
  endpoint->clusters[endpoint->cluster_count].is_server = 1;
  endpoint->cluster_count++;
}

void cover_switch_cluster_report_action(zigbee_cover_switch_cluster *cluster) {
  hal_zigbee_send_report_attr(cluster->endpoint, 
                         ZCL_CLUSTER_MULTISTATE_INPUT_BASIC,
                         ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE, 0, 0, 0);
}

void cover_switch_trigger_output(zigbee_cover_switch_cluster *cluster, uint8_t command) {
  if (cluster->local_mode == COVER_LOCAL_MODE_DETACHED) {
    return;
  }

  // Find the target output cluster
  if (cluster->output_index == 0 || cluster->output_index > cover_clusters_cnt) {
    return;
  }

  zigbee_cover_cluster *output = 
      &cover_clusters[cluster->output_index - 1];

  switch (command) {
  case ZCL_CMD_WINDOW_COVERING_UP_OPEN:
    cover_open(output);
    break;
  case ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
    cover_close(output);
    break;
  case ZCL_CMD_WINDOW_COVERING_STOP:
    cover_stop(output);
    break;
  }
}

void cover_switch_cluster_on_open_button_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_CLOSE_PRESS : COVER_SWITCH_OPEN_PRESS;
  uint8_t command = cluster->reversal ? ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE : ZCL_CMD_WINDOW_COVERING_UP_OPEN;

  cluster->multistate_state = action;
  cover_switch_cluster_report_action(cluster);

  // Trigger local output if configured for press_start or short_and_long
  if (cluster->local_mode == COVER_LOCAL_MODE_PRESS_START ||
      cluster->local_mode == COVER_LOCAL_MODE_SHORT_AND_LONG_PRESS) {
    cover_switch_trigger_output(cluster, command);
  }

  // TODO: Send bind command if configured
}

void cover_switch_cluster_on_open_button_release(zigbee_cover_switch_cluster *cluster) {
  cluster->multistate_state = COVER_SWITCH_RELEASED;
  cover_switch_cluster_report_action(cluster);

  // Trigger stop on release for press_start mode
  if (cluster->local_mode == COVER_LOCAL_MODE_PRESS_START) {
    cover_switch_trigger_output(cluster, ZCL_CMD_WINDOW_COVERING_STOP);
  }
}

void cover_switch_cluster_on_open_button_long_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_CLOSE_LONG_PRESS : COVER_SWITCH_OPEN_LONG_PRESS;
  uint8_t command = cluster->reversal ? ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE : ZCL_CMD_WINDOW_COVERING_UP_OPEN;

  cluster->multistate_state = action;
  cover_switch_cluster_report_action(cluster);

  // Trigger local output if configured for long_press or short_and_long
  if (cluster->local_mode == COVER_LOCAL_MODE_LONG_PRESS ||
      cluster->local_mode == COVER_LOCAL_MODE_SHORT_AND_LONG_PRESS) {
    cover_switch_trigger_output(cluster, command);
  }

  // TODO: Send bind command if configured
}

void cover_switch_cluster_on_close_button_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_OPEN_PRESS : COVER_SWITCH_CLOSE_PRESS;
  uint8_t command = cluster->reversal ? ZCL_CMD_WINDOW_COVERING_UP_OPEN : ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE;

  cluster->multistate_state = action;
  cover_switch_cluster_report_action(cluster);

  // Trigger local output if configured for press_start or short_and_long
  if (cluster->local_mode == COVER_LOCAL_MODE_PRESS_START ||
      cluster->local_mode == COVER_LOCAL_MODE_SHORT_AND_LONG_PRESS) {
    cover_switch_trigger_output(cluster, command);
  }

  // TODO: Send bind command if configured
}

void cover_switch_cluster_on_close_button_release(zigbee_cover_switch_cluster *cluster) {
  cluster->multistate_state = COVER_SWITCH_RELEASED;
  cover_switch_cluster_report_action(cluster);

  // Trigger stop on release for press_start mode
  if (cluster->local_mode == COVER_LOCAL_MODE_PRESS_START) {
    cover_switch_trigger_output(cluster, ZCL_CMD_WINDOW_COVERING_STOP);
  }
}

void cover_switch_cluster_on_close_button_long_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_OPEN_LONG_PRESS : COVER_SWITCH_CLOSE_LONG_PRESS;
  uint8_t command = cluster->reversal ? ZCL_CMD_WINDOW_COVERING_UP_OPEN : ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE;

  cluster->multistate_state = action;
  cover_switch_cluster_report_action(cluster);

  // Trigger local output if configured for long_press or short_and_long
  if (cluster->local_mode == COVER_LOCAL_MODE_LONG_PRESS ||
      cluster->local_mode == COVER_LOCAL_MODE_SHORT_AND_LONG_PRESS) {
    cover_switch_trigger_output(cluster, command);
  }

  // TODO: Send bind command if configured
}

void cover_switch_cluster_init(zigbee_cover_switch_cluster *cluster) {
  cluster->switch_type = 0x02; // Multifunction (momentary)
  cluster->output_index = cluster->cover_switch_idx + 1;
  cluster->reversal = 0;
  cluster->local_mode = COVER_LOCAL_MODE_SHORT_AND_LONG_PRESS;
  cluster->binded_mode = COVER_BINDED_MODE_SHORT_AND_LONG_PRESS;
  cluster->multistate_state = COVER_SWITCH_RELEASED;
}

void cover_switch_cluster_store_attrs_to_nv(zigbee_cover_switch_cluster *cluster) {
  nv_config_buffer.output_index = cluster->output_index;
  nv_config_buffer.reversal = cluster->reversal;
  nv_config_buffer.local_mode = cluster->local_mode;
  nv_config_buffer.binded_mode = cluster->binded_mode;
  nv_config_buffer.switch_type = cluster->switch_type;
  
  hal_nvm_write(NV_ITEM_COVER_SWITCH_CONFIG(cluster->cover_switch_idx),
                sizeof(zigbee_cover_switch_cluster_config),
                (uint8_t *)&nv_config_buffer);
}

void cover_switch_cluster_load_attrs_from_nv(zigbee_cover_switch_cluster *cluster) {
  hal_nvm_status_t st = hal_nvm_read(
      NV_ITEM_COVER_SWITCH_CONFIG(cluster->cover_switch_idx),
      sizeof(zigbee_cover_switch_cluster_config),
      (uint8_t *)&nv_config_buffer);
  
  if (st != HAL_NVM_SUCCESS) {
    printf("No cover switch config in NV, using defaults\r\n");
    return;
  }
  
  cluster->output_index = nv_config_buffer.output_index;
  cluster->reversal = nv_config_buffer.reversal;
  cluster->local_mode = nv_config_buffer.local_mode;
  cluster->binded_mode = nv_config_buffer.binded_mode;
  cluster->switch_type = nv_config_buffer.switch_type;
}

void cover_switch_cluster_on_write_attr(zigbee_cover_switch_cluster *cluster,
                                         uint16_t attribute_id) {
  switch (attribute_id) {
  case ZCL_ATTR_COVER_SWITCH_CONFIG_SWITCH_TYPE:
  case ZCL_ATTR_COVER_SWITCH_CONFIG_OUTPUT_INDEX:
  case ZCL_ATTR_COVER_SWITCH_CONFIG_REVERSAL:
  case ZCL_ATTR_COVER_SWITCH_CONFIG_LOCAL_MODE:
  case ZCL_ATTR_COVER_SWITCH_CONFIG_BINDED_MODE:
    cover_switch_cluster_store_attrs_to_nv(cluster);
    break;
  case ZCL_ATTR_COVER_SWITCH_CONFIG_LONG_PRESS_DUR:
    // Long press duration is shared between open and close buttons
    cluster->close_button->long_press_duration_ms = 
        cluster->open_button->long_press_duration_ms;
    cover_switch_cluster_store_attrs_to_nv(cluster);
    break;
  }
}

