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
 * @brief    Thingy application main file.
 *
 * This file contains the source code for the Blend template application on Thingy. 
 */
#define BLEND_THINGY_SDK
#include <float.h>
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
#include "drv_ext_gpio.h"
#include "drv_ext_light.h"
#include "m_batt_meas.h"
#include "m_ble.h"
#include "m_environment.h"
#include "m_motion.h"
#include "m_sound.h"
#include "m_ui.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "pca20020.h"
#include "softdevice_handler.h"
#include "support_func.h"
#include "twi_manager.h"

#define  NRF_LOG_MODULE_NAME "main          "
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ble_gap.h"

#include "command.h"
#include "speaker.h"

//! Value used as error code on stack dump, can be used to identify stack location on stack unwind.
#define DEAD_BEEF 0xDEADBEEF
//! Maximum size of scheduler events.
#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE)
//! Maximum number of events in the scheduler queue.
#define SCHED_QUEUE_SIZE 60

//! BLEnd parameters {Epoch, Adv. interval, mode}.
blend_param_t m_blend_param = { 150, 100, BLEND_MODE_FULL};
#define discover_index 1

#define LED_CONFIG_OFF \
  {             \
    .mode = BLE_UIS_LED_MODE_OFF  \
  }             

#define LED_CONFIG_GREEN			\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 11,				\
	  .g  = 102,				\
	  .b  = 35				\
        }					\
      }						\
  }

#define LED_CONFIG_PURPLE			\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 75,				\
	  .g  = 0,				\
	  .b  = 130				\
        }					\
      }						\
  }

#define LED_CONFIG_RED				\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 177,				\
	  .g  = 0,				\
	  .b  = 0				\
        }					\
      }						\
  }

  /* === Section (LED Configs) === */
#define LED_CONFIG_CHARGING			\
  {						\
    .mode = BLE_UIS_LED_MODE_BREATHE,		\
    .data =					\
    {						\
      .mode_const =				\
      {						\
	.r  = 255,				\
	.g  = 255,				\
	.b  = 255,				\
      }						\
    }						\
  }

//! Discovered device count.
#define m_data_len 1
uint8_t payload[m_data_len] = {0x01};
blend_data_t m_blend_data;

#define FILTER_PREFIX_LEN 3
#define MAC_ADDR_LEN 6
#define CMD_START (FILTER_PREFIX_LEN+MAC_ADDR_LEN)
// 0xFF is the manufacturer data field. 
uint8_t filter_prefix[FILTER_PREFIX_LEN] = {
    0xFF, 0xFE, 0xFF
};
static const ble_uis_led_t m_led_config_on = LED_CONFIG_PURPLE;
static const ble_uis_led_t m_led_config_off = LED_CONFIG_OFF;
// record BLE address of this device
ble_gap_addr_t m_addr;


uint32_t start_tick = 0;
static uint32_t epoch_count = 0;

static m_ble_service_handle_t  m_ble_service_handles[THINGY_SERVICES_MAX];


static const ble_uis_led_t m_led_scan = LED_CONFIG_PURPLE;
static const ble_uis_led_t m_led_adv = LED_CONFIG_GREEN;

uint8_t on_scan_flag  = 0;
uint8_t discovered = 0;
bool discover_mode = true;

/*!< Button related variables */
static bool charging_mode = false;
int btn_hit_cnt = 0;
static const ble_uis_led_t m_led_config_charging = LED_CONFIG_CHARGING;

static const nrf_drv_twi_t m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);
static void button_evt_handler(uint8_t pin_no, uint8_t button_action);




void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
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
static void power_manage(void)
{
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static ret_code_t button_init(void) {
  ret_code_t err_code;

  /* Configure gpiote for the sensors data ready interrupt. */
  if (!nrf_drv_gpiote_is_init()) {
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }
  static const app_button_cfg_t button_cfg = {
    .pin_no         = BUTTON,
    .active_state   = APP_BUTTON_ACTIVE_LOW,
    .pull_cfg       = NRF_GPIO_PIN_PULLUP,
    .button_handler = button_evt_handler
  };  

  err_code = app_button_init(&button_cfg, 1, APP_TIMER_TICKS(50));
  APP_ERROR_CHECK(err_code);

  return app_button_enable();
}

static void thingy_init(void)
{
    uint32_t                 err_code;
    m_ui_init_t              ui_params;
    m_environment_init_t     env_params;
    m_motion_init_t          motion_params;
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

    /* err_code = led_set(&m_led_idle,NULL); */
    /* APP_ERROR_CHECK(err_code); */
    err_code = button_init();
    APP_ERROR_CHECK(err_code);

}

static void board_init(void)
{
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

static void toggle_charging_mode() {
  if (charging_mode) {
    charging_mode = false;
    sd_nvic_SystemReset();
  }
  charging_mode = true;
  blend_sched_stop();
  ret_code_t err_code = led_set(&m_led_config_charging, NULL);
  APP_ERROR_CHECK(err_code);
}

static void button_evt_handler(uint8_t pin_no, uint8_t button_action) {
  uint32_t err_code;
  if (pin_no == BUTTON)
  {
    if (button_action == 0){
      btn_hit_cnt += 1;
      if (btn_hit_cnt == 3){
        NRF_LOG_DEBUG("Pressed 3 times. Switch modes.\n");
        toggle_charging_mode();
        btn_hit_cnt = 0;
      }
    }
  }
}

static void run_test(){
  ret_code_t err_code;
  start_tick = app_timer_cnt_get();
  blend_sched_start();
}

static void set_blend_data()
{
  m_blend_data.data_length = m_data_len;
  m_blend_data.data = payload;
  if (blend_advdata_set(&m_blend_data) != BLEND_NO_ERROR) {
    NRF_LOG_ERROR("Blend data set error");
  }
}

static bool verify_beacon(uint8_t* p_data) {
  for (int i = 0; i < FILTER_PREFIX_LEN; i ++) {
    if (p_data[i] != filter_prefix[i]) {
      return false;
    }
  }
  uint8_t* p_addr = p_data + FILTER_PREFIX_LEN;
  // Verfiy Mac address;
  for (int i = 0; i < MAC_ADDR_LEN; ++i) {
    if (p_addr[i] != m_addr.addr[MAC_ADDR_LEN-1-i]) {
      return false;
    }
  }
  return true;
}

static void process_cmd(uint8_t* p_data) {
  switch(p_data[0]) {
    case CMD_LIGHT: {
      if (p_data[1] == 0) {
        uint32_t err_code = led_set(&m_led_config_on, NULL);
        APP_ERROR_CHECK(err_code);
      } else {
        uint32_t err_code = led_set(&m_led_config_off, NULL);
        APP_ERROR_CHECK(err_code);
      }
      break;
    }
    case CMD_SOUND: {
      if (p_data[1] > SPEAKER_SAMPLE_MAX) {
        NRF_LOG_ERROR(" Speaker service sample soundtrack id invalid\r\n");
      }
      NRF_LOG_INFO("Playing speaker sample soundtrack%d\r\n", p_data[1]);
      play_sample_sound(p_data[1]);
      break;
    }
  }
}

void parse_beacon(uint8_t* p_data, uint8_t dlen) {

  uint16_t index  = 0;
  bool flag = false;
  while (index < dlen) {
    uint8_t field_length = p_data[index];
    if (verify_beacon(p_data+index+1)) {
      // NRF_LOG_HEXDUMP_INFO(p_data + index + 1 + CMD_START, 2);
      process_cmd(p_data + index + 1 + CMD_START);
    }
    index += field_length + 1;
  }
}

static void m_blend_handler(blend_evt_t * p_blend_evt)
{
  if (p_blend_evt->evt_id == BLEND_EVT_UNFILTERED_BEACON) {
    parse_beacon(p_blend_evt->evt_data.data, p_blend_evt->evt_data.data_length);
  }
}

int main(void) {
    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
		timer_init();
	
    // NRF_LOG_DEBUG("===== Blend mode %d started! =====\r\n", m_blend_param.blend_mode);


    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    board_init();
    thingy_init();
	  m_speaker_init();
    blend_init(m_blend_param, m_blend_handler, m_ble_service_handles);
    set_blend_data();
    set_does_report_unfiltered_beacon(true);

    // Get the BLE address of this device
    err_code = sd_ble_gap_addr_get(&m_addr);
    APP_ERROR_CHECK(err_code);

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
