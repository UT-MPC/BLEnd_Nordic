/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file
 *
 * @brief    Stacon application main file.
 *
 * Source code of the percom demo application based on the BLEnd template. 
 */
#define BLEND_THINGY_SDK
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_button.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_uis.h"
#include "blend.h"
#include "context.h"
#include "drv_ext_gpio.h"
#include "drv_ext_light.h"
#include "light_control.h"
#include "m_batt_meas.h"
#include "m_ble.h"
#include "m_ui.h"
#include "matching.h"
#include "node.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "pca20020.h"
#include "sensor.h"
#include "softdevice_handler.h"
#include "support_func.h"
#include "twi_manager.h"

//! Value used as error code on stack dump, can be used to identify stack location on stack unwind.
#define DEAD_BEEF 0xDEADBEEF
//! Maximum size of scheduler events.
#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE)
//! Maximum number of events in the scheduler queue.
#define SCHED_QUEUE_SIZE 60

#define PROTOCOL_ID 0x8B
#define DEVICE_ID 0x01
#define MAX_DEVICE 10
#define DATA_LENGTH 15
#define NUM_ENABLED_SENSOR 8
#define LOSING_PERIOD 2.5

//! BLEnd parameters {Epoch, Adv. interval, mode}.
const uint16_t lambda_ms = 2000;
const uint16_t epoch_length_ms = 2000;
const uint16_t adv_interval_ms = 77;

//! {protocol_id, node_id, cap_vec, demand_vec, shared_type, value1, value2}.
uint8_t payload[DATA_LENGTH];

blend_data_t m_blend_data;		

static m_ble_service_handle_t  m_ble_service_handles[THINGY_SERVICES_MAX];

static const ble_uis_led_t led_colors[5] = {LED_CONFIG_WHITE, LED_CONFIG_GREEN, LED_CONFIG_PURPLE, LED_CONFIG_BLUE, LED_CONFIG_RED};

uint8_t on_scan_flag  = 0;
uint8_t discovered = 0;
bool discover_mode = true;

static const nrf_drv_twi_t m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);

/*!< List of nodes in the neighborhood(self-exclusive). */
node_t* node_lst_head;
/*!< Local node. */
node_t* localhost;
/*!< Snapshot of the neighborhood. */
node_t* snapshot;
/*!< Container for the context values shared and received in the neighborhood(indexed by context type). */
context_t context_pool[NUM_CONTEXT_TYPES];
/*!< Context type of the current sharing task. Use TASK_OFFSET in beacons (<OFFSET is invalid). */
uint8_t current_task_type;
/*!< Current task result. */
context_t saved_reading;

/* === Section (Function Prototypes) === */

/*!< AfterScan as in Stacon */
bool compare_snapshots(void);
uint8_t take_snapshot(void);

/*!< Helper function prototypes. */
node_t* merge_sorted(node_t* lst1, node_t* lst2);
void front_back_split(node_t* source, node_t** front_ref, node_t** back_ref);
void mergesort(node_t** head_ref);

/*!< Unit test prototypes. */
void unittest_sorting();

/* === End of Section (Function Prototypes) === */

/* === Section (Board Functions) === */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
#if NRF_LOG_ENABLED
  error_info_t * err_info = (error_info_t*)info;
  NRF_LOG_ERROR(" id = %d, pc = %d, file = %s, line number: %d, error code = %d = %s \r\n", \
		id, pc, nrf_log_push((char*)err_info->p_file_name), err_info->line_num, err_info->err_code, nrf_log_push((char*)nrf_strerror_find(err_info->err_code)));
#endif
    
  (void)m_ui_led_set_event(M_UI_ERROR);
  NRF_LOG_FINAL_FLUSH();
  nrf_delay_ms(5);
    
  // On assert, the system can only recover with a reset.
#ifndef DEBUG
  NVIC_SystemReset();
#endif

  app_error_save_and_stop(id, pc, info);
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
#define FPU_EXCEPTION_MASK 0x0000009F
static void power_manage(void) {
  __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
  (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);

  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

static void thingy_init(void) {
  uint32_t                 err_code;
  m_ui_init_t              ui_params;
  m_ble_init_t             ble_params;
  batt_meas_init_t         batt_meas_init = BATT_MEAS_PARAM_CFG;

  /**@brief Initialize the TWI manager. */
  err_code = twi_manager_init(APP_IRQ_PRIORITY_THREAD);
  APP_ERROR_CHECK(err_code);

  /**@brief Initialize LED and button UI module. */
  ui_params.p_twi_instance = &m_twi_sensors;
  err_code = m_ui_init(&m_ble_service_handles[THINGY_SERVICE_UI],
		       &ui_params);
  APP_ERROR_CHECK(err_code);
}

static void board_init(void) {
  uint32_t            err_code;
  drv_ext_gpio_init_t ext_gpio_init;

#if defined(THINGY_HW_v0_7_0)
#error   "HW version v0.7.0 not supported."
#elif defined(THINGY_HW_v0_8_0)
  NRF_LOG_WARNING("FW compiled for depricated Thingy HW v0.8.0 \r\n");
#elif defined(THINGY_HW_v0_9_0)
  NRF_LOG_WARNING("FW compiled for depricated Thingy HW v0.9.0 \r\n");
#endif

  static const nrf_drv_twi_config_t twi_config =
    {
      .scl                = TWI_SCL,
      .sda                = TWI_SDA,
      .frequency          = NRF_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

  static const drv_sx1509_cfg_t sx1509_cfg =
    {
      .twi_addr       = SX1509_ADDR,
      .p_twi_instance = &m_twi_sensors,
      .p_twi_cfg      = &twi_config
    };

  ext_gpio_init.p_cfg = &sx1509_cfg;
    
  err_code = support_func_configure_io_startup(&ext_gpio_init);
  APP_ERROR_CHECK(err_code);

  nrf_delay_ms(100);
}

static void timer_init(void)
{
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

static void run_test(){
  ret_code_t err_code;
  blend_sched_start();
  unittest_sorting();
}

//! Start inclusive, end exclusive.
uint32_t rng_rand(int start,int end) {
  uint8_t rand[4];
  ret_code_t err_code;
  uint8_t pool_avail = 0;

  while (pool_avail <4) {
    err_code = sd_rand_application_bytes_available_get(&pool_avail);
    APP_ERROR_CHECK(err_code);       
  }

  err_code = sd_rand_application_vector_get(rand, 4);
  APP_ERROR_CHECK(err_code);
  uint32_t dice = abs(rand[0] + (rand[1]<<8) + (rand[2]<<16) + (rand[3]<<24));
  return start + (dice % (end - start));
}

/**@brief Function for updating the beacon payload.
 *
 * @details This function will be called at the beginning of each epoch and update the payload accordingly.
 *
 * @param[in] context_t Context that the host is sharing (Invalid type identified by 0xFF).
 * @param[out] payload Pointer to the beacon payload array.
 *
 * @return Return status.
 */
uint32_t update_payload(context_t context_in, uint8_t* payload) {
  // For debug purpose
  /* char* x = malloc(sizeof(char) * 30); */
  /* context2str(context_in, x); */
  /* NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN "Encoding context: %s.\r\n", (uint32_t)x); */
  /* free(x); */
  
  //uint8_t sharing_type, uint32_t ctx_val1, uint32_t ctx_val2, uint8_t* payload
  payload[0] = PROTOCOL_ID;
  payload[1] = DEVICE_ID;
  payload[2] = (localhost->cap_vec >> 8); // Big endian
  payload[3] = localhost->cap_vec & 0xFF;
  payload[4] = (localhost->demand_vec >> 8);
  payload[5] = localhost->demand_vec & 0xFF;
  if (current_task_type >= TASK_OFFSET) {
    payload[6] = context_in.ctx_type;
    uint8_t* vp = (uint8_t*) &(context_in.value1);
    for (int i = 0; i < 4; ++i) {
      payload[7+i] = vp[i];
    }
    vp = (uint8_t*) &(context_in.value2);
    for (int i = 0; i < 4; ++i) {
      payload[11+i] = vp[i];
    }
  } else {
    memset(&payload[6], 0, 9*sizeof(uint8_t));
  }


  m_blend_data.data_length = DATA_LENGTH;
  m_blend_data.data = payload;
  if (blend_advdata_set(&m_blend_data) != BLEND_NO_ERROR) {
    NRF_LOG_ERROR("Blend data set error");
    return 1;
  }
  return 0;
}

/**@brief Initialize the node's sensing equipment and tasks.
*/
uint32_t middleware_init(void) {
  localhost = malloc(sizeof(node_t));
  localhost->node_id = DEVICE_ID;
  // TODO(liuchg): randomized init. for capabilities.
  localhost->cap_vec = 0;
  SetBit(localhost->cap_vec, 1);
  SetBit(localhost->cap_vec, 2);
  localhost->demand_vec = 0xFFFF;
  localhost->next = NULL;

  node_lst_head = localhost;

  // Randomly assign a capapble task.
  current_task_type = rng_rand(0, 2) + TASK_OFFSET;
  
  return 0;
}

/**@brief Update the sensing task.
 *
 * Reselect the sensing task based on the current neighborhood. Output to global variable current_task_type.
 */
uint32_t update_sensing_task(void) {
  if (compare_snapshots()) {
    return 0;
  }
  uint8_t l_id = take_snapshot();
  current_task_type = get_new_assignment(snapshot, l_id);
  if (current_task_type >= TASK_OFFSET && TestBit(localhost->cap_vec, current_task_type - TASK_OFFSET)) {
    NRF_LOG_DEBUG("Context task selected: %d.\r\n", current_task_type - TASK_OFFSET);
  } else {
    NRF_LOG_DEBUG("Context task selected: Idle.\r\n");
  }
  return 0;
}

/**@brief Query the sensor and update the shared context, if current task is valid.
 */
uint32_t execute_sensing_task(void) {
  if (current_task_type < TASK_OFFSET) {
    return 0;
  }
  // JH: The code below is only a testbed for Christine to create the Android code.
  context_sample(current_task_type - TASK_OFFSET);
  saved_reading = context_read(current_task_type - TASK_OFFSET);
  return 0;
}

/**@brief Context type visualization using the lightwell.
*/
uint32_t update_light(void) {
  if (current_task_type - TASK_OFFSET >= 5) { // TODO(liuchg): enable real checking below.
  //  if (current_task_type - TASK_OFFSET >= NUM_SENSOR_TYPE) {
    NRF_LOG_ERROR("Update light error (task context type out of range.)");
    return 1;
  }
  
  ret_code_t err_code = led_set(&led_colors[current_task_type],NULL);
  //ret_code_t err_code = led_set(&led_colors[2],NULL);
  APP_ERROR_CHECK(err_code);
  return 0;
}

/**@brief Decode the context exchange packet.
*/
decoded_packet_t decode(uint8_t * bytes) {
    uint8_t ngbr_id = bytes[1];
    uint16_t ngbr_cap = bytes[3] + (bytes[2] << 8);
    uint16_t ngbr_demand = bytes[5] + (bytes[4] << 8);
    uint8_t ctx_type = bytes[6];
    bool ctx_valid = (ctx_type >= TASK_OFFSET);
    uint16_t val1 = 0;
    uint16_t val2 = 0;
    val1 = bytes[7] + (bytes[8] << 8) + (bytes[9] << 16) + (bytes[10] << 24);
    //TODO(liuchg): Process the extended field for rich types.
    uint32_t cur_time_ms = app_timer_cnt_get();
    context_t cur_context = {ngbr_id, ctx_type, val1, val2, cur_time_ms};
    decoded_packet_t decoded = {ngbr_id, ngbr_cap, ngbr_demand, ctx_valid, cur_context, cur_time_ms};
    return decoded;
}

/**@brief Update the neighbor list from received packet.
*/
uint32_t update_neighbor_list(decoded_packet_t* packet) {
  node_t* cur = node_lst_head;
  node_t* prev = NULL;
  while(cur && cur->node_id != packet->node_id) {
    prev = cur;
    cur = cur->next;
  }
  if(!cur) {
    node_t* append_ngbr = malloc(sizeof(node_t));
    append_ngbr->node_id = packet->node_id;
    append_ngbr->cap_vec = packet->cap_vec;
    append_ngbr->demand_vec = packet->demand_vec;
    append_ngbr->last_sync_ms = packet->timestamp_ms;
    append_ngbr->next = NULL;
    if (prev) {
      prev->next = append_ngbr;
    } else {
      node_lst_head = append_ngbr;
    }
  } else {
    cur->last_sync_ms = packet->timestamp_ms;
    // TODO(liuchg): Check cap and demand changes if the env. is dynamic
  }

  cur = node_lst_head;
  prev = NULL;
  uint32_t cur_time_ms = app_timer_cnt_get();
  while(cur) {
    if (cur->last_sync_ms + LOSING_PERIOD*lambda_ms < cur_time_ms) {
      if (prev) {
	prev->next = cur->next;
	free(cur);
	cur = prev->next;
      } else {
	node_lst_head = cur->next;
	free(cur);
	cur = node_lst_head;
      }
    } else {
      prev = cur;
      cur = cur->next;
    }
  }

  return 0;
}

/**@brief Update the local context pool from received packet.
*/
uint32_t udpate_context_pool(decoded_packet_t* packet) {
  context_t cur_context = packet->context;

  uint8_t ctype = cur_context.ctx_type;
  if (ctype && ctype < NUM_CONTEXT_TYPES) {
    context_pool[ctype].ctx_type = ctype;
    context_pool[ctype].source_id = cur_context.source_id;
    context_pool[ctype].value1 = cur_context.value1;
    context_pool[ctype].value2 = cur_context.value2;
    context_pool[ctype].timestamp_ms = cur_context.timestamp_ms;
  }
  return 0;
}

static void m_blend_handler(blend_evt_t * p_blend_evt)
{
  switch (p_blend_evt->evt_id) {
  case BLEND_EVT_ADV_REPORT: {
    uint8_t * p_data = p_blend_evt->evt_data.data;
    uint8_t plen = p_blend_evt->evt_data.data_length;
    // JH: plen here is not the actual length of the data. Since BLEnd does not have a 
    // byte to denote length, all the data length is the maximum length which is 26 here.
    //if (p_data[0] != PROTOCOL_ID || plen != DATA_LENGTH) {
    if (p_data[0] != PROTOCOL_ID) {
      break;
    }
    decoded_packet_t decoded_packet = decode(p_data);
    update_neighbor_list(&decoded_packet);
    if (decoded_packet.is_ctx_valid){
      udpate_context_pool(&decoded_packet);
    }
    // For debug purpose
    char* x = malloc(sizeof(char) * 30);
    context2str(decoded_packet.context, x);
    NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN "Read context: %s from node %d\r\n", (uint32_t)x, decoded_packet.context.source_id);
    free(x);
    break;
    }
  case BLEND_EVT_EPOCH_START: {
    break;
  }
  case BLEND_EVT_AFTER_SCAN: {

    // JH: sample code for sample, read, and to_string the context type.
    /* context_sample(TEMP_CTX); */
    /* context_sample(HUMID_CTX); */
    /* context_sample(PRESS_CTX); */
    /* context_sample(COLOR_CTX); */
    /* context_t temp = context_read(TEMP_CTX); */
    /* context_t humid = context_read(HUMID_CTX); */
    /* context_t press = context_read(PRESS_CTX); */
    /* context_t color = context_read(COLOR_CTX); */
    /* char* x = malloc(sizeof(char) * 30); */
    /* context2str(temp, x); */
    /* NRF_LOG_INFO("Read context: %s\r\n", (uint32_t)x); */
    /* context2str(humid, x); */
    /* NRF_LOG_INFO("Read context: %s\r\n", (uint32_t)x); */
    /* context2str(press, x); */
    /* NRF_LOG_INFO("Read context: %s\r\n", (uint32_t)x); */
    /* context2str(color, x); */
    /* NRF_LOG_INFO("Read context: %s\r\n", (uint32_t)x); */
    /* free(x); */
    // Update sensing task
    update_sensing_task();
    // Execute sensing task
    execute_sensing_task();
    // Update lightwell
    update_light();

    // Update beacon payload
    if (update_payload(saved_reading, payload)) {
      NRF_LOG_ERROR("Error when updating beacon payload.");
    }
    break;
  }
  default: {
    NRF_LOG_ERROR("Event handler (invalid event type).");
  }
  }
}

void sensor_init()
{
  humidity_sensor_init(&m_twi_sensors);
  pressure_sensor_init(&m_twi_sensors);
  color_sensor_init(&m_twi_sensors);
}
/* === End of Section (Board Functions) === */


int main(void) {
  uint32_t err_code;
  err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
  timer_init();

  blend_param_t m_blend_param = {epoch_length_ms, adv_interval_ms, BLEND_MODE_FULL};

  NRF_LOG_DEBUG("===== Blend mode %d started! =====\r\n", m_blend_param.blend_mode);
		
  err_code = nrf_drv_rng_init(NULL);
  APP_ERROR_CHECK(err_code);
		
  
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  board_init();
  thingy_init();
  sensor_init();
  blend_init(m_blend_param, m_blend_handler, m_ble_service_handles);

  middleware_init();

  run_test();
    
  for (;;)
    {
      app_sched_execute();

      if (!NRF_LOG_PROCESS()) // Process logs
        { 
	  power_manage();
        }
    }
}

/* === Section (Function Prototype Implementation) === */

/**@brief Comparing the current neighborhood status with the snapshot.
 *
 * @return Whether the state is the same.
 */
bool compare_snapshots(void) {
  node_t* cur_it = node_lst_head;
  node_t* snap_it = snapshot;
  while(cur_it || snap_it) {
    if (snap_it && snap_it->node_id == localhost->node_id) {
      snap_it = snap_it->next;
      continue;
    }
    if (!cur_it || !snap_it) {
      return false;
    }
    if (!(cur_it->node_id == snap_it->node_id && cur_it->cap_vec == snap_it->cap_vec && cur_it->demand_vec == snap_it->demand_vec)) {
      return false;
    }
    if (cur_it) {
      cur_it = cur_it->next;
    }
    if (snap_it) {
      snap_it = snap_it->next;
    }
  }
  return true;
}

/**@brief Take a snapshot of the current neighborhood.
 *
 * @return Index of localhost.
 */
uint8_t take_snapshot(void) {
  node_t* it = snapshot;
  while(snapshot) {
    it = snapshot;
    snapshot = snapshot->next;
    free(it);
  }
  uint8_t l_id = 0;
  if (node_lst_head) {
    mergesort(&(node_lst_head));
    it = node_lst_head;
    node_t* s_prev;
    while(it) {
      node_t* elmt = (node_t*)malloc(sizeof(node_t));
      elmt->node_id = it->node_id;
      if (elmt->node_id < localhost->node_id) {
	++l_id;
      }
      elmt->cap_vec = it->cap_vec;
      elmt->demand_vec = it->demand_vec;
      elmt->next = NULL;
      if (!snapshot) {
	snapshot = elmt;
      } else {
	s_prev->next = elmt;
      }
      s_prev = elmt;
      it = it->next;
    }
  }
  // Insert localhost
  node_t* copy_loc = (node_t*)malloc(sizeof(node_t));
  copy_loc->node_id = localhost->node_id;
  copy_loc->cap_vec = localhost->cap_vec;
  copy_loc->demand_vec = localhost->demand_vec;
  if (l_id == 0) {
    copy_loc->next = snapshot;
    snapshot = copy_loc;
  } else {
    it = snapshot;
    for (int i = l_id - 1; i > 0; --i) {
      it = it->next;
    }
    copy_loc->next = it->next;
    it->next = copy_loc;
  }
}

void mergesort(node_t** head_ref) {
  node_t* head = *head_ref;
  node_t* lst1;
  node_t* lst2;
  if ((head == NULL) || head->next == NULL) {
    return;
  }
  front_back_split(head, &lst1, &lst2);
  mergesort(&lst1);
  mergesort(&lst2);
  *head_ref = merge_sorted(lst1, lst2);
}

node_t* merge_sorted(node_t* lst1, node_t* lst2) {
  node_t* ret = NULL;
  if (!lst1) {
    return lst2;
  } else if (!lst2) {
    return lst1;
  }

  if (lst1->node_id <= lst2->node_id) {
    ret = lst1;
    ret->next = merge_sorted(lst1->next, lst2);
  } else {
    ret = lst2;
    ret->next = merge_sorted(lst1, lst2->next);
  }

  return ret;
}

void front_back_split(node_t* source, node_t** front_ref, node_t** back_ref) {
  node_t* slow = source;
  node_t* fast = source->next;
  while (fast) {
    fast = fast->next;
    if (fast) {
      slow = slow->next;
      fast = fast->next;
    }
  }
  // Slow stops before the mid.
  *front_ref = source;
  *back_ref = slow->next;
  slow->next = NULL;
}

/* === End of Section (Function Prototype Implementation) === */

/* === Section (Tests) === */

void push(node_t** head_ref, int node_id)
{
    node_t* new_node = (node_t*) malloc(sizeof(node_t));
    new_node->node_id  = node_id;
    new_node->next = (*head_ref);
    (*head_ref) = new_node;
}

void printlist(node_t* node)
{
  NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN " ==== PRINT LIST \r\n");
  while(node != NULL) {
      NRF_LOG_DEBUG("%d ", node->node_id);
      node = node->next;
  }
  NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN "\n ==== PRINT LIST \r\n");
}

void unittest_sorting() {
  push(&node_lst_head, 15);
  push(&node_lst_head, 8);
  push(&node_lst_head, 2);
  push(&node_lst_head, 3);
  push(&node_lst_head, 11);
  mergesort(&(node_lst_head->next));
  printlist(node_lst_head);
  // Expected: 11, 1, 2, 3, 8, 15
}
/* === End of Section (Tests) === */
