#include "context.h"

#include "app_timer.h"
#include "sensor.h"
#include <string.h>
#include <stdlib.h>
#include "nrf_log.h"
#include "pca20020.h"

void temp2ctx(void* temp_in, context_t* context_out);
void humid2ctx(void* temp_in, context_t* context_out);
void pressure2ctx(void* temp_in, context_t* context_out);

char* ctx_name[]={"Temperature", "Humidity", "Pressure"};
sensor2ctx_func_t sensor2ctx_func[]={temp2ctx, humid2ctx, pressure2ctx};
sensor_read_func_t sensor_read_func[]={m_temperature_read, m_humidity_read, m_pressure_read};

void temp2ctx(void* temp_in, context_t* context_out){
  temperature_t temp = *((temperature_t*)temp_in);
  context_out->timestamp_ms = temp.timestamp;
  context_out->value1 = (temp.integer<<8) + (temp.decimal & 0xff);
  // NRF_LOG_DEBUG("!!!! %08X!!!!\r\n", context_out->value1);
  // NRF_LOG_DEBUG("!!!! %02X!!!!\r\n", temp.integer);
  // NRF_LOG_DEBUG("!!!! %02X!!!!\r\n", temp.decimal);
}

void humid2ctx(void* humid_in, context_t* context_out){

}

void pressure2ctx(void* pressure_in, context_t* context_out){

}

context_t context_read(uint8_t ctx_type){
  void ** cur_sensor_ptr = malloc(sizeof(void*));
  sensor_read_func[ctx_type](cur_sensor_ptr);
  context_t new_context={-1,ctx_type,0,0,0};
  // temperature_t* temp = (temperature_t*)(*cur_sensor_ptr);

  sensor2ctx_func[ctx_type](*cur_sensor_ptr, &new_context);
  return new_context;
}

 
