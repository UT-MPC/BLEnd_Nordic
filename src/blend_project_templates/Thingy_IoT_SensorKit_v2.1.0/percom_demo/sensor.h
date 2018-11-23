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
} temperature_t;

typedef struct{
  uint8_t humid;
  
}humidity_t;
typedef struct
{
  int32_t  integer;
  uint8_t  decimal;
} pressure_t;




// Initialize humidity sensor
uint32_t humidity_sensor_init(nrf_drv_twi_t const* p_twi_instance);
uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance);

void m_humidity_sample();
void m_pressure_sample();

uint32_t m_humidity_disable();
uint32_t m_pressure_disable();
