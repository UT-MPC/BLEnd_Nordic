#include "sensor.h"
#include <string.h>
#include <stdlib.h>

#include "app_timer.h"
#include "app_util_platform.h"
#include "drv_color.h"
#include "drv_gas_sensor.h"
#include "drv_humidity.h"
#include "drv_pressure.h"
#include "fstorage.h"
#include "m_ui.h"
#include "macros_common.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "pca20020.h"
#include "blend.h"




static void temperature_conv_data(float in_temp, temperature_t * p_out)
{
  float f_decimal;

  p_out->integer = (int8_t)in_temp;
  f_decimal = in_temp - p_out->integer;
  p_out->decimal = (uint8_t)(f_decimal * 100.0f);
  NRF_LOG_DEBUG("temperature_conv_data: Temperature: ,%d.%d,C\r\n", p_out->integer, p_out->decimal);
}

/**@brief Function for converting the humidity sample.
 */
static void humidity_conv_data(uint8_t humid, humidity_t * p_out_humid)
{
  p_out_humid->humid = (uint8_t)humid;
  NRF_LOG_DEBUG("humidity_conv_data: Relative Humidty: ,%d,%%\r\n", humid);
}


/**@brief Function for converting the pressure sample.
 */
static void pressure_conv_data(float in_press, pressure_t * p_out_press)
{
  float f_decimal;

  p_out_press->integer = (int32_t)in_press;
  f_decimal = in_press - p_out_press->integer;
  p_out_press->decimal = (uint8_t)(f_decimal * 100.0f);
  NRF_LOG_DEBUG("pressure_conv_data: Pressure/Altitude: %d.%d Pa/m\r\n", p_out_press->integer, p_out_press->decimal);
}

void temp_get(){
	float temperature = drv_humidity_temp_get();
  uint16_t humidity = drv_humidity_get();
  temperature_t new_temp;
  humidity_t new_humid;
  temperature_conv_data(temperature,&new_temp);
  humidity_conv_data(humidity, &new_humid);
}
void drv_humidity_evt_handler(drv_humidity_evt_t event)
{
  uint32_t err_code;
  if (event == DRV_HUMIDITY_EVT_DATA)
  {
    float temperature = drv_humidity_temp_get();
    uint16_t humidity = drv_humidity_get();
    temperature_t new_temp;
    humidity_t new_humid;
    temperature_conv_data(temperature,&new_temp);
    humidity_conv_data(humidity, &new_humid);
  }
  else
  {
    APP_ERROR_CHECK_BOOL(false);
  }
}
static void drv_pressure_evt_handler(drv_pressure_evt_t const * p_event)
{
  switch (p_event->type)
  {
    case DRV_PRESSURE_EVT_DATA:
    {
      if (p_event->mode == DRV_PRESSURE_MODE_BAROMETER)
      {
        pressure_t pressure;
        pressure_conv_data(drv_pressure_get(),&pressure);
      }
    }
    break;

    case DRV_PRESSURE_EVT_ERROR:
      APP_ERROR_CHECK_BOOL(false);
      break;

    default:
      break;
  }
}
uint32_t humidity_sensor_init(nrf_drv_twi_t const * p_twi_instance)
{
  ret_code_t  err_code = NRF_SUCCESS;
  
  static const nrf_drv_twi_config_t twi_config =
  {
    .scl                = TWI_SCL,
    .sda                = TWI_SDA,
    .frequency          = NRF_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
  };		
//	nrf_drv_twi_t const * p_twi_instance = m_twi_sensors;  ///< The instance of TWI master to be used for transactions.
  drv_humidity_init_t    init_params =
  {
    .twi_addr            = HTS221_ADDR,
    .pin_int             = HTS_INT,
    .p_twi_instance      = p_twi_instance,
    .p_twi_cfg           = &twi_config,
    .evt_handler         = drv_humidity_evt_handler
  };
  err_code = drv_humidity_init(&init_params);
  return err_code;
}
uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
  drv_pressure_init_t init_params;

  static const nrf_drv_twi_config_t twi_config =
  {
    .scl                = TWI_SCL,
    .sda                = TWI_SDA,
    .frequency          = NRF_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
  };

  init_params.twi_addr                = LPS22HB_ADDR;
  init_params.pin_int                 = LPS_INT;
  init_params.p_twi_instance          = p_twi_instance;
  init_params.p_twi_cfg               = &twi_config;
  init_params.evt_handler             = drv_pressure_evt_handler;
  init_params.mode                    = DRV_PRESSURE_MODE_BAROMETER;

  return drv_pressure_init(&init_params);
}

void m_humidity_sample(){
  ret_code_t err_code;
  err_code = drv_humidity_enable();
  err_code = drv_humidity_sample();
  APP_ERROR_CHECK(err_code);
}
void m_pressure_sample()
{
  uint32_t err_code;

  err_code = drv_pressure_enable();
  APP_ERROR_CHECK(err_code);

  err_code = drv_pressure_sample();
  APP_ERROR_CHECK(err_code);
}
uint32_t m_humidity_disable(){
  return drv_humidity_disable();
}

uint32_t m_pressure_disable(){
  return drv_pressure_disable();
}