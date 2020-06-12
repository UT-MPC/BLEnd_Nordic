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

//! Value used as error code on stack dump, can be used to identify stack location on stack unwind.
#define DEAD_BEEF 0xDEADBEEF
//! Maximum size of scheduler events.
#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE)
//! Maximum number of events in the scheduler queue.
#define SCHED_QUEUE_SIZE 60

//! BLEnd parameters {Epoch, Adv. interval, mode}.
blend_param_t m_blend_param = { 2000, 77, BLEND_MODE_FULL};
#define APP_DEVICE_NUM 0x01
#define MAX_DEVICE 10
#define m_data_len 7
#define discover_index 1

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

//! Discovered device count.
uint8_t payload[m_data_len] = {APP_DEVICE_NUM,
			       0x00, 0x00,
			       0x00, 0x00,
			       0x00, 0x00};
blend_data_t m_blend_data;
				
uint32_t start_tick = 0;
static uint32_t epoch_count = 0;
// uint8_t found_device[MAX_DEVICE];

static m_ble_service_handle_t  m_ble_service_handles[THINGY_SERVICES_MAX];

uint8_t found_device[MAX_DEVICE] = {0,0,0};

static const ble_uis_led_t m_led_scan = LED_CONFIG_PURPLE;
static const ble_uis_led_t m_led_adv = LED_CONFIG_GREEN;

uint8_t on_scan_flag  = 0;
uint8_t discovered = 0;
bool discover_mode = true;

static const nrf_drv_twi_t m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);

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

/**@brief Function for putting Thingy into sleep mode.
 *
 * @note This function will not return.
 */
/* static void sleep_mode_enter(void) */
/* { */
/*     uint32_t err_code; */

/*     NRF_LOG_INFO("Entering sleep mode \r\n"); */
/*     err_code = m_motion_sleep_prepare(true); */
/*     APP_ERROR_CHECK(err_code); */

/*     err_code = support_func_configure_io_shutdown(); */
/*     APP_ERROR_CHECK(err_code); */
    
/*     // Enable wake on button press. */
/*     nrf_gpio_cfg_sense_input(BUTTON, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW); */
/*     // Enable wake on low power accelerometer. */
/*     nrf_gpio_cfg_sense_input(LIS_INT1, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH); */
   
/*     NRF_LOG_FLUSH(); */
/*     nrf_delay_ms(5); */
/*     // Go to system-off (sd_power_system_off() will not return; wakeup will cause a reset). When debugging, this function may return and code execution will continue. */
/*     err_code = sd_power_system_off(); */
/*     NRF_LOG_WARNING("sd_power_system_off() returned. -Probably due to debugger being used. Instructions will still run. \r\n"); */
/*     NRF_LOG_FLUSH(); */
    
/*     #ifdef DEBUG */
/*         if(!support_func_sys_halt_debug_enabled()) */
/*         { */
/*             APP_ERROR_CHECK(err_code); // If not in debug mode, return the error and the system will reboot. */
/*         } */
/*         else */
/*         { */
/*             NRF_LOG_WARNING("Exec stopped, busy wait \r\n"); */
/*             NRF_LOG_FLUSH(); */
            
/*             while(true) // Only reachable when entering emulated system off. */
/*             { */
/*                 // Infinte loop to ensure that code stops in debug mode. */
/*             } */
/*         } */
/*     #else */
/*         APP_ERROR_CHECK(err_code); */
/*     #endif */
/* } */


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


/**@brief Battery module data handler.
 */
/* static void m_batt_meas_handler(m_batt_meas_event_t const * p_batt_meas_event) */
/* { */
/* 	NRF_LOG_INFO("MichHello:Voltage: %d V, Charge: %d %%, Event type: %d \r\n", */
/*                 p_batt_meas_event->voltage_mv, p_batt_meas_event->level_percent, p_batt_meas_event->type); */
   
/*     if (p_batt_meas_event != NULL) */
/*     { */
/*         if( p_batt_meas_event->type == M_BATT_MEAS_EVENT_LOW) */
/*         { */
/*             uint32_t err_code; */

/*             err_code = support_func_configure_io_shutdown(); */
/*             APP_ERROR_CHECK(err_code); */
            
/*             // Enable wake on USB detect only. */
/*             nrf_gpio_cfg_sense_input(USB_DETECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH); */

/*             NRF_LOG_WARNING("Battery voltage low, shutting down Thingy. Connect USB to charge \r\n"); */
/*             NRF_LOG_FINAL_FLUSH(); */
/*             // Go to system-off mode (This function will not return; wakeup will cause a reset). */
/*             err_code = sd_power_system_off(); */

/*             #ifdef DEBUG */
/*                 if(!support_func_sys_halt_debug_enabled()) */
/*                 { */
/*                     APP_ERROR_CHECK(err_code); // If not in debug mode, return the error and the system will reboot. */
/*                 } */
/*                 else */
/*                 { */
/*                     NRF_LOG_WARNING("Exec stopped, busy wait \r\n"); */
/*                     NRF_LOG_FLUSH(); */
/*                     while(true) // Only reachable when entering emulated system off. */
/*                     { */
/*                         // Infinte loop to ensure that code stops in debug mode. */
/*                     } */
/*                 } */
/*             #else */
/*                 APP_ERROR_CHECK(err_code); */
/*             #endif */
/*         } */
/*     } */
/* } */

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

/* static void restart_timer_handler(void* p_context){ */
/*   for (int i = 0; i < MAX_DEVICE; i ++) */
/*     { */
/*       if (i == (APP_DEVICE_NUM - 1)){ */
/* 	continue; */
/*       } */
/*       if (found_device[i] == 0){ */
/* 	NRF_LOG_INFO(NRF_LOG_COLOR_CODE_RED"===Fail to find device %d ===\r\n", i+1); */
/*       } */
/*     } */
/*   nrf_delay_ms(100); */
/*   sd_nvic_SystemReset(); */
/* } */

static void run_test(){
  ret_code_t err_code;
  memset(&found_device, 0, sizeof(found_device));
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

static void m_blend_handler(blend_evt_t * p_blend_evt)
{
  if (p_blend_evt->evt_id == BLEND_EVT_ADV_REPORT) {
      uint8_t * p_data = p_blend_evt->evt_data.data;
      uint8_t dlen = p_blend_evt->evt_data.data_length;
      int nbgr_id = p_data[0];
      uint32_t now_time = app_timer_cnt_get();
      uint8_t dis_flag = 0;
      if (dis_flag == 0) {
	found_device[nbgr_id - 1] = 1;
	uint32_t time = app_timer_cnt_diff_compute(now_time, start_tick);
	time = APP_TIMER_MS(time) & 0xffff;
	NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN"===Found device %d At time: %d===\r\n", nbgr_id, time);
      }
      return;
    }
  if (p_blend_evt->evt_id == BLEND_EVT_EPOCH_START) {
      epoch_count += 1;
      ret_code_t err_code = led_set(&m_led_scan,NULL);
      NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN"Epoch %d started.\r\n", epoch_count);
      APP_ERROR_CHECK(err_code);
    }	
  if (p_blend_evt->evt_id == BLEND_EVT_AFTER_SCAN) {
      ret_code_t err_code = led_set(&m_led_adv,NULL);
      NRF_LOG_DEBUG("Scan stopped.\r\n", epoch_count);
      APP_ERROR_CHECK(err_code);
    }

  if (p_blend_evt->evt_id == BLEND_EVT_UNFILTERED_BEACON) {
    NRF_LOG_DEBUG(NRF_LOG_COLOR_CODE_GREEN"Get unfiltered beacon.\r\n");
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
	
    blend_init(m_blend_param, m_blend_handler, m_ble_service_handles);
    set_blend_data();
    
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
