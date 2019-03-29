#ifndef _UTMPCSENSOR_H
#define _UTMPCSENSOR_H

#include "app_error.h"
#include "app_timer.h"
#include "blend.h"
#include "drv_gas_sensor.h"
#include "drv_humidity.h"
#include "drv_motion.h"
#include "drv_pressure.h"
#include "nordic_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

typedef struct {
  int8_t  integer;
  uint8_t decimal;
  uint32_t timestamp_ms;
} temperature_t;

typedef struct {
  uint8_t humid;
  uint32_t timestamp_ms;
}humidity_t;

typedef struct {
  int32_t  integer;
  uint8_t  decimal;
  uint32_t timestamp_ms;
} pressure_t;

typedef struct {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t clear;
  uint32_t timestamp_ms;
} color_t;

typedef struct {
  uint16_t ec02_ppm; ///< The equivalent CO2 (eCO2) value in parts per million (ppm).
  uint16_t tvoc_ppb; ///< The Total Volatile Organic Compound (TVOC) value in parts per billion (ppb).
  uint32_t timestamp_ms;
} gas_t;

typedef struct {
  uint8_t  led_red;
  uint8_t  led_green;
  uint8_t  led_blue;
} color_config_t;

typedef enum {
  GAS_STATE_IDLE,
  GAS_STATE_WARMUP,
  GAS_STATE_ACTIVE
} gas_state_t;

typedef struct {
  //float sound_level;
  int16_t avg_peak;
  int16_t max_peak;
  int16_t count_over_thres_per_frame;
  int16_t avg_all;
  int16_t avg_over_thres; 
  uint32_t timestamp_ms;
} sound_t;

typedef struct {
  //float acc_noise_level;
  //int acc_num_samples;
  int acc_num_frames;
  int acc_peak;
  int running_peak;
  int acc_count_over_thres;
  int num_frames_over_thres;
  float acc_avg_all;
  float acc_avg_over_thres;
} acc_sound_t;

typedef struct
{
  uint16_t mode_250ms;
  uint16_t mode_1s;
  uint16_t mode_10s;
  uint16_t mode_60s;
} m_gas_baseline_t;


typedef void (*sensor_read_func_t)(void **);
typedef void (*sensor_sample_func_t)();
typedef uint32_t (*sensor_disable_func_t) ();
typedef void (*sensor2str_func_t)(void*, char*);

uint32_t humidity_sensor_init(nrf_drv_twi_t const* p_twi_instance);
uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance);
uint32_t color_sensor_init(const nrf_drv_twi_t * p_twi_instance);
uint32_t gas_sensor_init(const nrf_drv_twi_t * p_twi_instance);
uint32_t sound_init();

void m_humidity_sample();
void m_pressure_sample();
void m_color_sample();
void m_gas_sample();
void m_sound_sample();

void m_temperature_read(void**);
void m_humidity_read(void**);
void m_pressure_read(void** );
void m_color_read(void** );
void m_gas_read(void** );
void m_sound_read(void** );

void m_temperature2str(void* , char*);
void m_humidity2str(void* , char*);
void m_pressure2str(void* , char*);
void m_color2str(void* , char*);
void m_gas2str(void* , char*);
void m_sound2str(void*, char*);

uint32_t m_humidity_disable();
uint32_t m_pressure_disable();
uint32_t m_color_disable();
uint32_t m_gas_disable();
uint32_t m_sound_disable();


extern temperature_t _temp_cache;
extern humidity_t _humid_cache;
extern pressure_t _pressure_cache;
extern color_t _color_cache;
extern gas_t _gas_cache;
extern sound_t _sound_cache;

#define COLOR_CONFIG_DEFAULT {        \
  .led_red             = 103,         \
  .led_green           = 78,          \
  .led_blue            = 29           \
}
#endif    // _UTMPCBLEND_H

#define GAS_BASELINE_DEFAULT {      \
  .mode_250ms = 0,                    \
  .mode_1s    = 0,                    \
  .mode_10s   = 0,                    \
  .mode_60s   = 0,                    \
}
