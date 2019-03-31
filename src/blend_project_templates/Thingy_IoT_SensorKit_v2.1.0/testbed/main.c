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
 * @brief    Testbed application main file.
 *
 * Source code of the testbed application based on the BLEnd template. 
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
#include "m_batt_meas.h"
#include "m_ble.h"
#include "m_ui.h"
#include "node.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_rng.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "pca20020.h"
#include "sensor.h"
#include "softdevice_handler.h"
#include "support_func.h"
#include "twi_manager.h"


#define DEAD_BEEF 0xDEADBEEF /*<! //! Value used as error code on stack dump.*/
//! Maximum size of scheduler events.
#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE)
#define SCHED_QUEUE_SIZE 60 /*!< Maximum number of events in the scheduler queue. */
#define DATA_LENGTH 22
#define BATT_READ_INTERVAL_MS 600000 /*!< 10min. */
#define ENERGY_SAVING_SENSING 0 /*!< 1-true, 0-false */
#define BATTERY_PROFILING 0 /*!< 1-true, 0-false */

/* === Section (Sampling schedule parameters) === */
/* Start => Early stop => Gas stop => Next Start */
#define SAMPLE_TIMER_WARMUP_MS 50
#define SAMPLE_INTERVAL_MS 300000
#define SAMPLE_TIMER_SAMPLE_EARLY_STOP_MS 5000
#define SAMPLE_TIMER_SAMPLE_GAS_MS 30000
#define MICROPHONE_TIMER_INTERVAL_MS 20000 // 10s auto-stoped sampling period followed 10s rest
/* === End of Section (Sampling schedule parameters) === */

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
/* === End of Section (LED Configs) === */

APP_TIMER_DEF (sampling_timer);
APP_TIMER_DEF (mic_timer);

/* === Section (BLEnd parameters) === */
const uint16_t lambda_ms = 4000;
const uint16_t epoch_length_ms = 1328;
const uint16_t adv_interval_ms = 127;
/* === End of Section (BLEnd parameters) === */


uint8_t payload[DATA_LENGTH];
static m_ble_service_handle_t  m_ble_service_handles[THINGY_SERVICES_MAX];
static const nrf_drv_twi_t m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);
static const ble_uis_led_t m_led_config_charging = LED_CONFIG_CHARGING;
const ctx_type_def enabled_sensors[5] = {TEMP_CTX, HUMID_CTX, PRESS_CTX, VOC_CTX, NOISE_CTX};

blend_data_t m_blend_data; /*!< Complete user payload. */
uint8_t on_scan_flag  = 0;
bool discover_mode = true;
bool sample_initiated = false;
bool gas_unfinished = false;
uint8_t batt_lvl_read = 0; /*!< Most recent read of battery level. */
bool mic_on = false;
/*!< Button related variables */
static bool charging_mode = false;
int btn_hit_cnt = 0;

/* === Section (Function Prototypes) === */

/*!< Sensing related functions */
uint32_t initiate_sampling(void);
void stop_sampling_update_payload(void);
void app_init(void);
void app_start(void);
void update_payload(void); /*!< Grab the context values and update the adv. content */
static void toggle_charging_mode();

/*!< Callback functions */
static void m_blend_handler(blend_evt_t * p_blend_evt);
static void m_batt_meas_handler(m_batt_meas_event_t const * p_batt_meas_event);
static void button_evt_handler(uint8_t pin_no, uint8_t button_action);
void sampling_timer_handler(void);
void mic_timer_handler(void);

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
    
  app_button_disable();

  err_code = button_init();
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

/**@brief Onboard battery measurement initialization.
*/
void batt_init(){
  uint32_t err_code;
  batt_meas_init_t batt_meas_init = BATT_MEAS_PARAM_CFG;
  batt_meas_init.evt_handler = m_batt_meas_handler;
  err_code = m_batt_meas_init(&m_ble_service_handles[THINGY_SERVICE_BATTERY], &batt_meas_init);
  APP_ERROR_CHECK(err_code);

  err_code = m_batt_meas_enable(BATT_READ_INTERVAL_MS);
  APP_ERROR_CHECK(err_code);
}

/**@brief Sensor initialization.
*/
void sensor_init() {
  humidity_sensor_init(&m_twi_sensors);
  pressure_sensor_init(&m_twi_sensors);
  color_sensor_init(&m_twi_sensors);
  gas_sensor_init(&m_twi_sensors);
  sound_init();
}

static void timer_init(void) {
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/* === End of Section (Board Functions) === */


/* === Section (Function Prototypes Implementation) === */

/**@brief Initiate all sensor sampling.
 */
uint32_t initiate_sampling(void) {
  //NRF_LOG_DEBUG("initiate_sampling: start sampling at %d ms.\r\n", _BLEND_APP_TIMER_MS(app_timer_cnt_get()));
  for (int i = 0; i < sizeof(enabled_sensors)/sizeof(ctx_type_def); ++i) {
    context_start(enabled_sensors[i]);
  }
  sample_initiated = true;
  return 0;
}

/**@brief Retrieve the sensor readings and update the beacon content.
 * @details Stop all sensors at the end.
 */
void stop_sampling_update_payload(void) {
  //NRF_LOG_DEBUG("stop_sampling_update_payload: turn off sampling at %d ms.\r\n", _BLEND_APP_TIMER_MS(app_timer_cnt_get()));
  if (gas_unfinished) {
    context_stop(VOC_CTX);
    gas_unfinished = false;
  } else {
    for (int i = 0; i < sizeof(enabled_sensors)/sizeof(ctx_type_def); ++i) {
      if (enabled_sensors[i] != VOC_CTX && enabled_sensors[i] != NOISE_CTX) {
	      context_stop(enabled_sensors[i]);
      }
    }
    gas_unfinished = true;
  }

  update_payload();
}

/**@brief Initialize the node's sensing equipments, app timer.
*/
void app_init(void) {
  uint32_t err_code = app_timer_create(&sampling_timer, APP_TIMER_MODE_SINGLE_SHOT, sampling_timer_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_create(&mic_timer, APP_TIMER_MODE_REPEATED, mic_timer_handler);
  APP_ERROR_CHECK(err_code);
  
  sample_initiated = false;
}

/**@brief Start timers.
*/
void app_start(void) {
  blend_sched_start();
  uint32_t err_code = app_timer_start(sampling_timer, APP_TIMER_TICKS(SAMPLE_TIMER_WARMUP_MS), NULL);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_start(mic_timer, APP_TIMER_TICKS(MICROPHONE_TIMER_INTERVAL_MS), NULL);
  APP_ERROR_CHECK(err_code);
}

void update_payload() {
  context_all_t* context_all = context_read_all();
  uint8_t payload[CONTEXT_ALL_SIZE + 1];
  context_all_to_bytes((uint8_t*)(payload), context_all);
  payload[sizeof(payload) - 1] = batt_lvl_read;
  free(context_all);
  
  m_blend_data.data_length = sizeof(payload);
  m_blend_data.data = payload;
  if (blend_advdata_set(&m_blend_data) != BLEND_NO_ERROR) {
    NRF_LOG_ERROR("Blend data set error");
  }
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

/**@brief Blend soft interrupt handlers (No use in testbed for now).
*/
static void m_blend_handler(blend_evt_t * p_blend_evt) {
  //  NRF_LOG_DEBUG("m_blend_handler:%d, (%d ms).\r\n",p_blend_evt->evt_id, _BLEND_APP_TIMER_MS(app_timer_cnt_get()));
  switch (p_blend_evt->evt_id) {
  case BLEND_EVT_ADV_REPORT: {
    break;
    }
  case BLEND_EVT_EPOCH_START: {
    break;
  }
  case BLEND_EVT_AFTER_SCAN: {
    break;
  }
  case BLEND_EVT_LAST_FULL_BEACON: {
    break;
  }
  }
}

static void m_batt_meas_handler(m_batt_meas_event_t const * p_batt_meas_event) {
  NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN"Voltage: %d V, Charge: %d %%, Event type: %d \r\n",
              p_batt_meas_event->voltage_mv, p_batt_meas_event->level_percent, p_batt_meas_event->type);
  batt_lvl_read = p_batt_meas_event->level_percent;
  if (p_batt_meas_event != NULL)
  {
    if( p_batt_meas_event->type == M_BATT_MEAS_EVENT_LOW)
    {
      uint32_t err_code;
      err_code = support_func_configure_io_shutdown();
      APP_ERROR_CHECK(err_code);
      // Enable wake on USB detect only.
      nrf_gpio_cfg_sense_input(USB_DETECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
      NRF_LOG_WARNING("Battery voltage low, shutting down Thingy. Connect USB to charge \r\n");
      NRF_LOG_FINAL_FLUSH();
      // Go to system-off mode (This function will not return; wakeup will cause a reset).
      err_code = sd_power_system_off();
      APP_ERROR_CHECK(err_code);
    }
  }
}

/**@brief App timer callback handler for controlling the start and end of sensor sampling.
 * @details Implicit input: sample_initiated(flag)
 */
void sampling_timer_handler(void) {
  ret_code_t ret = app_timer_stop(sampling_timer);
  APP_ERROR_CHECK(ret);
  uint64_t next_timer_ms = 999;
  if (sample_initiated) {
    stop_sampling_update_payload();
    if (gas_unfinished) {
      next_timer_ms = SAMPLE_TIMER_SAMPLE_GAS_MS - SAMPLE_TIMER_SAMPLE_EARLY_STOP_MS;
      //NRF_LOG_DEBUG("sampling_timer_handler: Turning off all sensors except gas and mic at %d ms.\r\n", _BLEND_APP_TIMER_MS(app_timer_cnt_get()));
    } else {
      sample_initiated = false;
      next_timer_ms = SAMPLE_INTERVAL_MS - SAMPLE_TIMER_SAMPLE_GAS_MS;
      //NRF_LOG_DEBUG("sampling_timer_handler: Turning off gas sensor at %d ms.\r\n", _BLEND_APP_TIMER_MS(app_timer_cnt_get()));
    }
  } else {
    initiate_sampling();
    sample_initiated = true;
    next_timer_ms = SAMPLE_TIMER_SAMPLE_EARLY_STOP_MS;
    //NRF_LOG_DEBUG("sampling_timer_handler: Start all sampling at %d ms.\r\n", _BLEND_APP_TIMER_MS(app_timer_cnt_get()));
  }
  ret = app_timer_start(sampling_timer, APP_TIMER_TICKS(next_timer_ms), NULL);
  APP_ERROR_CHECK(ret);
}

void mic_timer_handler(void) {
  // Update the payload
  update_payload();
  // Restart mic. sampling
  context_start(NOISE_CTX);
  //NRF_LOG_DEBUG("sampling_timer_handler: Turning on mic at %d ms.\r\n", _BLEND_APP_TIMER_MS(app_timer_cnt_get()));
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
/* === Section (Function Prototypes Implementation) === */


int main(void) {
  uint32_t err_code;
  err_code = NRF_LOG_INIT(NULL);

  //NRF_LOG_DEBUG("size context_all_t struct with pragma: %d", sizeof(context_all_t));
  APP_ERROR_CHECK(err_code);
  timer_init();

  blend_param_t m_blend_param = {epoch_length_ms, adv_interval_ms, BLEND_MODE_FULL};

  NRF_LOG_DEBUG("===== Blend mode %d started! =====\r\n", m_blend_param.blend_mode);
		
  err_code = nrf_drv_rng_init(NULL);
  APP_ERROR_CHECK(err_code);

  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  board_init();
  thingy_init();
  sensor_init();
  blend_init(m_blend_param, m_blend_handler, m_ble_service_handles);
  batt_init();
  app_init();

  app_start();
    
  for (;;) {
    app_sched_execute();
    if (!NRF_LOG_PROCESS()) {
      power_manage();
    }
  }
}

/* === Section (Function Prototype Implementation) === */

/* === End of Section (Function Prototype Implementation) === */

/* === Section (Tests) === */

/* === End of Section (Tests) === */
