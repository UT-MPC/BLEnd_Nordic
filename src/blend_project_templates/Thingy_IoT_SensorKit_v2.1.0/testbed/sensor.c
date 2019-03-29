#include "sensor.h"
#include <string.h>
#include <stdlib.h>

#include "app_timer.h"
#include "app_util_platform.h"
#include "drv_color.h"
#include "drv_gas_sensor.h"
#include "drv_humidity.h"
#include "drv_mic.h"
#include "drv_pressure.h"
#include "fstorage.h"
#include "m_ui.h"
#include "macros_common.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "pca20020.h"
#define	APP_TIMER_MS(TICKS) TICKS*(1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1))/(uint64_t) APP_TIMER_CLOCK_FREQ

#define SOUND_LEVEL_NUM_FRAMES 33 //33*16ms

static gas_state_t m_gas_state = GAS_STATE_IDLE;
static m_gas_baseline_t     * m_p_baseline;     ///< Baseline pointer.
static const m_gas_baseline_t m_default_baseline = GAS_BASELINE_DEFAULT; ///< Default baseline.

drv_gas_sensor_mode_t m_gas_mode = DRV_GAS_SENSOR_MODE_1S;

temperature_t _temp_cache;
humidity_t _humid_cache;
pressure_t _pressure_cache;
color_t _color_cache;
gas_t _gas_cache;
sound_t _sound_cache;

acc_sound_t _acc_sound = {0, 0, 0, 0, 0, 0.0f, 0.0f};

static color_config_t m_color_config = COLOR_CONFIG_DEFAULT;

void m_temperature2str(void* temp_p, char* str) {
  temperature_t temp_in = *((temperature_t*)temp_p);
  sprintf(str, "Temperature: %d.%d\'C", temp_in.integer, temp_in.decimal);
}

void m_humidity2str(void* humid_p, char* str) {
  humidity_t humid_in = *((humidity_t*)humid_p);
  sprintf(str, "Relative Humidity: %d%%", humid_in.humid);
}

void m_pressure2str(void* pressure_p, char* str) {
  pressure_t pressure_in = *((pressure_t*)pressure_p);
  sprintf(str, "Pressure: %d.%d hPa\r\n", pressure_in.integer, pressure_in.decimal);
}

void m_color2str(void* color_p, char* str) {
  color_t color_in = *((color_t*)color_p);
  sprintf(str, "Color: (%d, %d, %d, %d) \r\n", color_in.red, color_in.green, color_in.blue, color_in.clear);
}

void m_gas2str(void* gas_p, char* str) {
  gas_t gas_in = *((gas_t*)gas_p);
  sprintf(str, "Total VOC: (CO2:%d, TVOC:%d) \r\n", gas_in.ec02_ppm, gas_in.tvoc_ppb);
}

void m_sound2str(void* sound_p, char* str) {
  sound_t sound_in = *((sound_t*)sound_p);
  //sprintf(str, "Average noise level: " NRF_LOG_FLOAT_MARKER " \r\n", NRF_LOG_FLOAT(sound_in.sound_level));
  sprintf(str, "Sound info \n: ------ %d \n ------ %d \n ------ %d \n ------ %d \n ------ %d \n",
    sound_in.avg_peak, sound_in.max_peak, sound_in.count_over_thres_per_frame, sound_in.avg_all, sound_in.avg_over_thres);
}

/**@brief Function for converting the temperature sample.
 */
static void temperature_conv_data(float in_temp, temperature_t * p_out_temp) {
  float f_decimal;

  p_out_temp->integer = (int8_t)in_temp;
  f_decimal = in_temp - p_out_temp->integer;
  p_out_temp->decimal = (uint8_t)(f_decimal * 100.0f);
  p_out_temp->timestamp_ms = APP_TIMER_MS(app_timer_cnt_get());
  NRF_LOG_DEBUG("temperature_conv_data: Temperature: ,%d.%d,C\r\n", p_out_temp->integer, p_out_temp->decimal);
}

/**@brief Function for converting the humidity sample.
 */
static void humidity_conv_data(uint8_t humid, humidity_t * p_out_humid) {
  p_out_humid->humid = (uint8_t)humid;
  p_out_humid->timestamp_ms = APP_TIMER_MS(app_timer_cnt_get());
  NRF_LOG_DEBUG("humidity_conv_data: Relative Humidity: ,%d,%%\r\n", humid);
}


/**@brief Function for converting the pressure sample.
 */
static void pressure_conv_data(float in_press, pressure_t * p_out_press) {
  float f_decimal;

  p_out_press->integer = (int16_t)in_press;
  f_decimal = in_press - p_out_press->integer;
  p_out_press->decimal = (uint8_t)(f_decimal * 100.0f);
  p_out_press->timestamp_ms = APP_TIMER_MS(app_timer_cnt_get());
  NRF_LOG_DEBUG("pressure_conv_data: Pressure: %d.%d hPa\r\n", p_out_press->integer, p_out_press->decimal);
}

void m_temperature_read(void** data_ptr) {
  *data_ptr = &_temp_cache;
  return;
}
void m_humidity_read(void** data_ptr) {
  *data_ptr = &_humid_cache;
  return;
}

void m_pressure_read(void** data_ptr) {
  *data_ptr = &_pressure_cache;
  return;
}

void m_color_read(void** data_ptr) {
  *data_ptr = &_color_cache;
  return;
}

void m_gas_read(void** data_ptr) {
  *data_ptr = &_gas_cache;
  return;
}

void m_sound_read(void** data_ptr) {
  *data_ptr = &_sound_cache;
}

void drv_humidity_evt_handler(drv_humidity_evt_t event) {
  uint32_t err_code;
  if (event == DRV_HUMIDITY_EVT_DATA)
  {
    
    float temperature = drv_humidity_temp_get();
    uint16_t humidity = drv_humidity_get();
    // if (m_calib_gas_sensor == true)
    // {
    //     err_code = calibrate_gas_sensor(humidity, temperature);
    //     APP_ERROR_CHECK(err_code);
    //     m_calib_gas_sensor = false;
    // }
    temperature_t new_temp;
    humidity_t new_humid;
    temperature_conv_data(temperature,&_temp_cache);
    humidity_conv_data(humidity, &_humid_cache);
  }
  else
  {
    APP_ERROR_CHECK_BOOL(false);
  }
}

static void drv_pressure_evt_handler(drv_pressure_evt_t const * p_event) {
  switch (p_event->type)
  {
    case DRV_PRESSURE_EVT_DATA:
    {
      if (p_event->mode == DRV_PRESSURE_MODE_BAROMETER)
      {
        pressure_conv_data(drv_pressure_get(),&_pressure_cache);
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

/**@brief Color sensor data handler.
 */
static void drv_color_data_handler(drv_color_data_t const * p_data) {
  (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

  if (p_data != NULL)
  {
    _color_cache.red   = p_data->red;
    _color_cache.green = p_data->green;
    _color_cache.blue  = p_data->blue;
    _color_cache.clear = p_data->clear;
    _color_cache.timestamp_ms = APP_TIMER_MS(app_timer_cnt_get());
  }
}

/**@brief Gas sensor data handler.
 */
void drv_gas_evt_handler(drv_gas_sensor_data_t const * p_data)
{
  if (p_data != NULL)
  {
    _gas_cache.ec02_ppm = p_data->ec02_ppm;
    _gas_cache.tvoc_ppb = p_data->tvoc_ppb;
    _gas_cache.timestamp_ms = APP_TIMER_MS(app_timer_cnt_get());
  }
}

/**@brief Accumulate the average noise level of each frame (256 samples) and average the result from 33 frames.
 */
uint32_t drv_mic_data_handler(m_audio_frame_t * p_frame)
{
  
  //float noise_level = ((float *)p_frame->data)[0];
  int peak_in_frame = *(int *)(p_frame->data);
  int count_over_thres = *(int *)(p_frame->data + 4);
  float avg_all_samples = *(float *)(p_frame->data + 8);
  float avg_over_thres_samples = *(float *)(p_frame->data + 12);
  //int num_samples = *(int *)(p_frame->data + 20);

  // NRF_LOG_DEBUG("drv_mic_data_handler peak_in_frame = %d \r\n: ", peak_in_frame);
  // NRF_LOG_DEBUG("drv_mic_data_handler count_over_thres = %d \r\n: ", count_over_thres);
  // NRF_LOG_DEBUG("drv_mic_data_handler avg_all_samples = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(avg_all_samples));
  // NRF_LOG_DEBUG("drv_mic_data_handler avg_over_thres_samples = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(avg_over_thres_samples));
  // NRF_LOG_DEBUG("drv_mic_data_handler num_samples = %d \r\n: ", num_samples);

  //_acc_sound.acc_noise_level += noise_level;
  //_acc_sound.acc_num_samples += p_frame->data_size;
  _acc_sound.acc_peak += peak_in_frame;
  if (peak_in_frame > _acc_sound.running_peak) {
    _acc_sound.running_peak = peak_in_frame; 
  }
  _acc_sound.acc_count_over_thres += count_over_thres; // could be 0
  if (count_over_thres > 0) {
    _acc_sound.num_frames_over_thres++;
  }
  _acc_sound.acc_avg_all += avg_all_samples;
  _acc_sound.acc_avg_over_thres += avg_over_thres_samples; // could be 0

  if (++(_acc_sound.acc_num_frames) >= SOUND_LEVEL_NUM_FRAMES) {
    //float avg_noise_level = _acc_sound.acc_noise_level / (float)_acc_sound.acc_num_frames;
    //_sound_cache.sound_level = avg_noise_level;

    _sound_cache.avg_peak = (int16_t) ((float) _acc_sound.acc_peak) / ((float) _acc_sound.acc_num_frames);
    _sound_cache.max_peak = (int16_t) _acc_sound.running_peak;
    _sound_cache.count_over_thres_per_frame = (int16_t) ((float) _acc_sound.acc_count_over_thres) / ((float) _acc_sound.acc_num_frames);
    _sound_cache.avg_all =  (int16_t) ((float) _acc_sound.acc_avg_all) / ((float) _acc_sound.acc_num_frames);
    _sound_cache.avg_over_thres = (int16_t) ((float) _acc_sound.acc_avg_over_thres) / (_acc_sound.num_frames_over_thres);
    _sound_cache.timestamp_ms = APP_TIMER_MS(app_timer_cnt_get());

    NRF_LOG_DEBUG("drv_mic_data_handler count_over_thres = %d \r\n: ", count_over_thres);
    NRF_LOG_DEBUG("drv_mic_data_handler avg_peak = %d \r\n: ", _sound_cache.avg_peak);
    NRF_LOG_DEBUG("drv_mic_data_handler max_peak = %d \r\n: ", _sound_cache.max_peak);
    NRF_LOG_DEBUG("drv_mic_data_handler count_over_thres_per_frame = %d \r\n: ", _sound_cache.count_over_thres_per_frame);
    NRF_LOG_DEBUG("drv_mic_data_handler avg_all = %d \r\n: ", _sound_cache.avg_all);
    NRF_LOG_DEBUG("drv_mic_data_handler avg_over_thres = %d \r\n: ", _sound_cache.avg_over_thres);
    NRF_LOG_DEBUG("drv_mic_data_handler avg_peak = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(_sound_cache.avg_peak));

    NRF_LOG_DEBUG("drv_mic_data_handler (%d ms): number of frames  = %d \r\n: ", _sound_cache.timestamp_ms, _acc_sound.acc_num_frames);
    //_acc_sound.acc_noise_level = 0;
    //_acc_sound.acc_num_samples = 0;
    _acc_sound.acc_num_frames = 0;
    _acc_sound.acc_peak = 0;
    _acc_sound.running_peak = 0;
    _acc_sound.acc_count_over_thres = 0;
    _acc_sound.num_frames_over_thres = 0;
    _acc_sound.acc_avg_all = 0;
    _acc_sound.acc_avg_over_thres = 0;

  }
  return NRF_SUCCESS;
}

uint32_t gas_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t       err_code;
    drv_gas_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = CCS811_ADDR;
    init_params.data_handler   = drv_gas_evt_handler;

    err_code = drv_gas_sensor_init(&init_params);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

uint32_t humidity_sensor_init(nrf_drv_twi_t const * p_twi_instance) {
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

uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance) {
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

uint32_t color_sensor_init(const nrf_drv_twi_t * p_twi_instance) {
    uint32_t err_code;
    drv_color_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = BH1745_ADDR;
    init_params.data_handler   = drv_color_data_handler;

    err_code = drv_color_init(&init_params);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

uint32_t sound_init() {
  return drv_mic_init(drv_mic_data_handler);
}

void m_color_sample(void) {
  ret_code_t err_code;
  drv_ext_light_rgb_intensity_t color;

  color.r = m_color_config.led_red;
  color.g = m_color_config.led_green;
  color.b = m_color_config.led_blue;

  (void)drv_ext_light_rgb_intensity_set(DRV_EXT_RGB_LED_SENSE, &color);

  err_code = drv_color_start();
  APP_ERROR_CHECK(err_code);

  err_code = drv_color_sample();
  APP_ERROR_CHECK(err_code);

}

void m_humidity_sample() {
  ret_code_t err_code;
  err_code = drv_humidity_enable();
  APP_ERROR_CHECK(err_code);

  err_code = drv_humidity_sample();
  APP_ERROR_CHECK(err_code);
}

void m_pressure_sample() {
  uint32_t err_code;

  err_code = drv_pressure_enable();
  APP_ERROR_CHECK(err_code);

  err_code = drv_pressure_sample();
  APP_ERROR_CHECK(err_code);
}

void m_gas_sample(){
  uint32_t err_code;
  err_code = drv_gas_sensor_start(m_gas_mode);
  RETURN_IF_ERROR(err_code);

  m_gas_state = GAS_STATE_WARMUP;

  // return app_timer_start(gas_calib_timer_id,
  //                         APP_TIMER_TICKS(M_GAS_BASELINE_WRITE_MS),
  //                         NULL);
}

void m_sound_sample() {
  uint32_t err_code;
  err_code = drv_mic_start();
  APP_ERROR_CHECK(err_code);
}

uint32_t m_sound_disable() {
  uint32_t err_code;
  err_code = drv_mic_stop();
  APP_ERROR_CHECK(err_code);
  return NRF_SUCCESS;
}

uint32_t m_humidity_disable() {
  return drv_humidity_disable();
}

uint32_t m_pressure_disable() {
  return drv_pressure_disable();
}

uint32_t m_color_disable() {
  uint32_t err_code;

  (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

  err_code = drv_color_stop();
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

uint32_t m_gas_disable(){
  uint32_t err_code;
  uint16_t baseline;

  if (m_gas_state == GAS_STATE_ACTIVE)
  {
      // err_code = drv_gas_sensor_baseline_get(&baseline);
      // RETURN_IF_ERROR(err_code);

      // err_code = gas_store_baseline_flash(baseline);
      // RETURN_IF_ERROR(err_code);
  }

    m_gas_state = GAS_STATE_IDLE;

    return drv_gas_sensor_stop();
}
