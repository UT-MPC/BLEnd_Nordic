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
void color2ctx(void* temp_in, context_t* context_out);

void ctx2temp(context_t* , void** );
void ctx2humid(context_t* , void** );
void ctx2pressure(context_t* , void** );
void ctx2color(context_t* , void** );

char* ctx_name[]={"Temperature", "Humidity", "Pressure", "Color"};
sensor2ctx_func_t sensor2ctx_func[]={temp2ctx, humid2ctx, pressure2ctx, color2ctx};
ctx2sensor_func_t ctx2sensor_func[]={ctx2temp, ctx2humid, ctx2pressure, ctx2color};
sensor_sample_func_t sensor_sample_func[]={m_humidity_sample, m_humidity_sample, m_pressure_sample, m_color_sample};
sensor_read_func_t sensor_read_func[]={m_temperature_read, m_humidity_read, m_pressure_read, m_color_read};
sensor2str_func_t sensor2str_func[]={m_temperature2str, m_humidity2str, m_pressure2str, m_color2str};

void color2ctx(void* color_in, context_t* context_out){
  color_t color = *((color_t*)color_in);
  context_out->timestamp_ms = color.timestamp;
  context_out->value1 = (color.red << 16) + color.green;          // red | green
  context_out->value2 = (color.blue << 16) + color.clear;          // blue | clear
  return;
}
void ctx2color(context_t* context_in, void** sensor_p){
  color_t* color;
  color = malloc(sizeof(color_t));
  color->red = context_in->value1 >> 16;
  color->green = context_in->value1 & 0xffff;
  color->blue = context_in->value2 >> 16;
  color->clear = context_in->value2 & 0xffff;
  *sensor_p = color;
  return;
}

void temp2ctx(void* temp_in, context_t* context_out){
  temperature_t temp = *((temperature_t*)temp_in);
  context_out->timestamp_ms = temp.timestamp;
  context_out->value1 = (temp.integer<<8) + (temp.decimal & 0xff);
  context_out->value2 = 0;
}
void ctx2temp(context_t* context_in, void** sensor_p){
  temperature_t* temp;
  temp = malloc(sizeof(temperature_t));
  temp->timestamp = context_in->timestamp_ms;
  temp->decimal = context_in->value1 & 0xff;
  temp->integer = (context_in->value1 >>8) & 0xff;
  *sensor_p = temp;
  return;
}

void humid2ctx(void* humid_in, context_t* context_out){
  humidity_t humid = *((humidity_t*)humid_in);
  context_out->timestamp_ms = humid.timestamp;
  context_out->value1 = humid.humid & 0xff;
  context_out->value2 = 0;
  return;
}
void ctx2humid(context_t* context_in, void** sensor_p){
  humidity_t* humid;
  humid = malloc(sizeof(humidity_t));
  humid->timestamp = context_in->timestamp_ms;
  humid->humid = context_in->value1 & 0xff;
  *sensor_p = humid;
  return;
}

void pressure2ctx(void* pressure_in, context_t* context_out){
  pressure_t pressure = *((pressure_t*) pressure_in);
  context_out->timestamp_ms = pressure.timestamp;
  context_out->value1 = pressure.decimal & 0xff;
  context_out->value1 += (pressure.integer & 0xffffff) << 8;
  return;
}
void ctx2pressure(context_t* context_in, void** sensor_p){
  pressure_t* pressure;
  pressure = malloc(sizeof(pressure_t));
  pressure->timestamp = context_in->timestamp_ms;
  pressure->decimal = context_in->value1 & 0xff;
  pressure->integer = context_in->value1 >> 8;
  *sensor_p = pressure;
  return;
}

context_t context_read(uint8_t ctx_type){
  void ** cur_sensor_ptr = malloc(sizeof(void*));
  sensor_read_func[ctx_type](cur_sensor_ptr);
  context_t new_context={-1,ctx_type,0,0,0};
  sensor2ctx_func[ctx_type](*cur_sensor_ptr, &new_context);
  free(cur_sensor_ptr);
  return new_context;
}

void context_sample(uint8_t ctx_type){
  sensor_sample_func[ctx_type]();
  return;
}

void context2str(context_t context_in, char* str_out){
  uint8_t ctype = context_in.ctx_type;
  void ** cur_sensor_ptr = malloc(sizeof(void*));
  ctx2sensor_func[ctype](&context_in, cur_sensor_ptr);
  sensor2str_func[ctype](*cur_sensor_ptr, str_out);
  free(*cur_sensor_ptr);
  free(cur_sensor_ptr);
}
 