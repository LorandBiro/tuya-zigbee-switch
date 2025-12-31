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

void cover_switch_cluster_on_up_button_press(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_up_button_release(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_up_button_long_press(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_down_button_press(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_down_button_release(zigbee_cover_switch_cluster *cluster);
void cover_switch_cluster_on_down_button_long_press(zigbee_cover_switch_cluster *cluster);

zigbee_cover_switch_cluster *cover_switch_cluster_by_endpoint[10];

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
  cover_switch_cluster_load_attrs_from_nv(cluster);

  cluster->up_button->on_press =
      (ev_button_callback_t)cover_switch_cluster_on_up_button_press;
  cluster->up_button->on_release =
      (ev_button_callback_t)cover_switch_cluster_on_up_button_release;
  cluster->up_button->on_long_press =
      (ev_button_callback_t)cover_switch_cluster_on_up_button_long_press;
  cluster->up_button->callback_param = cluster;

  cluster->down_button->on_press =
      (ev_button_callback_t)cover_switch_cluster_on_down_button_press;
  cluster->down_button->on_release =
      (ev_button_callback_t)cover_switch_cluster_on_down_button_release;
  cluster->down_button->on_long_press =
      (ev_button_callback_t)cover_switch_cluster_on_down_button_long_press;
  cluster->down_button->callback_param = cluster;

  // WindowCovering client cluster FIRST (for binding and configuration)
  SETUP_ATTR_FOR_TABLE(cluster->windowcovering_attr_infos, 0,
                      ZCL_ATTR_WINDOW_COVERING_INPUT_OUTPUT_INDEX, ZCL_DATA_TYPE_UINT8,
                      ATTR_WRITABLE, cluster->output_index);
  SETUP_ATTR_FOR_TABLE(cluster->windowcovering_attr_infos, 1,
                      ZCL_ATTR_WINDOW_COVERING_INPUT_REVERSAL, ZCL_DATA_TYPE_BOOLEAN,
                      ATTR_WRITABLE, cluster->reversal);
  SETUP_ATTR_FOR_TABLE(cluster->windowcovering_attr_infos, 2,
                      ZCL_ATTR_WINDOW_COVERING_INPUT_LOCAL_MODE, ZCL_DATA_TYPE_ENUM8,
                      ATTR_WRITABLE, cluster->local_mode);
  SETUP_ATTR_FOR_TABLE(cluster->windowcovering_attr_infos, 3,
                      ZCL_ATTR_WINDOW_COVERING_INPUT_BINDED_MODE, ZCL_DATA_TYPE_ENUM8,
                      ATTR_WRITABLE, cluster->binded_mode);
  SETUP_ATTR_FOR_TABLE(cluster->windowcovering_attr_infos, 4,
                      ZCL_ATTR_WINDOW_COVERING_INPUT_LONG_PRESS_DUR, ZCL_DATA_TYPE_UINT16,
                      ATTR_WRITABLE, cluster->up_button->long_press_duration_ms);

  endpoint->clusters[endpoint->cluster_count].cluster_id = ZCL_CLUSTER_WINDOW_COVERING;
  endpoint->clusters[endpoint->cluster_count].attribute_count = 5;
  endpoint->clusters[endpoint->cluster_count].attributes = cluster->windowcovering_attr_infos;
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
    cover_up(output);
    break;
  case ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
    cover_down(output);
    break;
  case ZCL_CMD_WINDOW_COVERING_STOP:
    cover_stop(output);
    break;
  }
}

void cover_switch_cluster_on_up_button_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_DOWN_PRESS : COVER_SWITCH_UP_PRESS;
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

void cover_switch_cluster_on_up_button_release(zigbee_cover_switch_cluster *cluster) {
  cluster->multistate_state = COVER_SWITCH_RELEASED;
  cover_switch_cluster_report_action(cluster);

  // Trigger stop on release for press_start mode
  if (cluster->local_mode == COVER_LOCAL_MODE_PRESS_START) {
    cover_switch_trigger_output(cluster, ZCL_CMD_WINDOW_COVERING_STOP);
  }
}

void cover_switch_cluster_on_up_button_long_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_DOWN_LONG_PRESS : COVER_SWITCH_UP_LONG_PRESS;
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

void cover_switch_cluster_on_down_button_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_UP_PRESS : COVER_SWITCH_DOWN_PRESS;
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

void cover_switch_cluster_on_down_button_release(zigbee_cover_switch_cluster *cluster) {
  cluster->multistate_state = COVER_SWITCH_RELEASED;
  cover_switch_cluster_report_action(cluster);

  // Trigger stop on release for press_start mode
  if (cluster->local_mode == COVER_LOCAL_MODE_PRESS_START) {
    cover_switch_trigger_output(cluster, ZCL_CMD_WINDOW_COVERING_STOP);
  }
}

void cover_switch_cluster_on_down_button_long_press(zigbee_cover_switch_cluster *cluster) {
  uint8_t action = cluster->reversal ? COVER_SWITCH_UP_LONG_PRESS : COVER_SWITCH_DOWN_LONG_PRESS;
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

void cover_switch_cluster_store_attrs_to_nv(zigbee_cover_switch_cluster *cluster) {
  hal_nvm_write(NVM_COVER_SWITCH_0_CONFIG + cluster->input_idx,
                4, (uint8_t *)&cluster->output_index);
}

void cover_switch_cluster_load_attrs_from_nv(zigbee_cover_switch_cluster *cluster) {
  uint8_t read_status = hal_nvm_read(
      NVM_COVER_SWITCH_0_CONFIG + cluster->input_idx, 4, (uint8_t *)&cluster->output_index);
  if (read_status != 0) {
    // Default values
    cluster->output_index = cluster->input_idx + 1;  // Default to same index + 1
    cluster->reversal = 0;
    cluster->local_mode = COVER_LOCAL_MODE_SHORT_AND_LONG_PRESS;
    cluster->binded_mode = COVER_BINDED_MODE_SHORT_AND_LONG_PRESS;
  }
}

void cover_switch_cluster_on_write_attr(zigbee_cover_switch_cluster *cluster,
                                         uint16_t attribute_id) {
  switch (attribute_id) {
  case ZCL_ATTR_WINDOW_COVERING_INPUT_OUTPUT_INDEX:
  case ZCL_ATTR_WINDOW_COVERING_INPUT_REVERSAL:
  case ZCL_ATTR_WINDOW_COVERING_INPUT_LOCAL_MODE:
  case ZCL_ATTR_WINDOW_COVERING_INPUT_BINDED_MODE:
    cover_switch_cluster_store_attrs_to_nv(cluster);
    break;
  case ZCL_ATTR_WINDOW_COVERING_INPUT_LONG_PRESS_DUR:
    // Long press duration is shared between up and down buttons
    cluster->down_button->long_press_duration_ms = 
        cluster->up_button->long_press_duration_ms;
    cover_switch_cluster_store_attrs_to_nv(cluster);
    break;
  }
}

