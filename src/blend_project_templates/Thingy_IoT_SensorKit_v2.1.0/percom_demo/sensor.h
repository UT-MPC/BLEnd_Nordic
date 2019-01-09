#ifndef _UTMPCSENSOR_H
#define _UTMPCSENSOR_H

#include "drv_humidity.h"
#include "drv_pressure.h"
#include "drv_gas_sensor.h"
#include "drv_motion.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nordic_common.h"
#include "blend.h"

typedef struct
{
  int8_t  integer;
  uint8_t decimal;
  uint32_t timestamp;
} temperature_t;

typedef struct{
  uint8_t humid;
  uint32_t timestamp;
}humidity_t;
typedef struct
{
  int32_t  integer;
  uint8_t  decimal;
  uint32_t timestamp;
} pressure_t;

typedef struct
{
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t clear;
  uint32_t timestamp;
} color_t;

typedef struct
{
    uint8_t  led_red;
    uint8_t  led_green;
    uint8_t  led_blue;
} color_config_t;

typedef void (*sensor_read_func_t)(void **);
typedef void (*sensor_sample_func_t)();
typedef void (*sensor2str_func_t)(void*, char*);

// Initialize humidity sensor
uint32_t humidity_sensor_init(nrf_drv_twi_t const* p_twi_instance);
uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance);
uint32_t color_sensor_init(const nrf_drv_twi_t * p_twi_instance);


void m_humidity_sample();
void m_pressure_sample();
void m_color_sample();


void m_temperature_read(void**);
void m_humidity_read(void**);
void m_pressure_read(void** );
void m_color_read(void** );

void m_temperature2str(void* , char*);
void m_humidity2str(void* , char*);
void m_pressure2str(void* , char*);
void m_color2str(void* , char*);

uint32_t m_humidity_disable();
uint32_t m_pressure_disable();
uint32_t m_color_disable();

extern temperature_t _temp_cache;
extern humidity_t _humid_cache;
extern pressure_t _pressure_cache;
extern color_t _color_cache;

#define COLOR_CONFIG_DEFAULT {        \
  .led_red             = 103,         \
  .led_green           = 78,          \
  .led_blue            = 29           \
}
#endif    // _UTMPCBLEND_H
