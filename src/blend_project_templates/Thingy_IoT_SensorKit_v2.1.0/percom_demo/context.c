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
void gas2ctx(void* gas_in, context_t* context_out);
void sound2ctx(void* sound_in, context_t* context_out);

void ctx2temp(context_t* , void** );
void ctx2humid(context_t* , void** );
void ctx2pressure(context_t* , void** );
void ctx2color(context_t* , void** );
void ctx2gas(context_t* , void** );
void ctx2sound(context_t*, void** );

#define max_ctx 5
char* ctx_name[]={"Temperature", "Humidity", "Pressure", "Color", "VOC", "SoundLevel"};
sensor2ctx_func_t sensor2ctx_func[]={temp2ctx, humid2ctx, pressure2ctx, color2ctx, gas2ctx, sound2ctx};
ctx2sensor_func_t ctx2sensor_func[]={ctx2temp, ctx2humid, ctx2pressure, ctx2color, ctx2gas, ctx2sound};
sensor_sample_func_t sensor_start_func[]={m_humidity_sample, m_humidity_sample, m_pressure_sample, m_color_sample, m_gas_sample, m_sound_sample};
sensor_read_func_t sensor_read_func[]={m_temperature_read, m_humidity_read, m_pressure_read, m_color_read, m_gas_read, m_sound_read};
sensor_disable_func_t sensor_stop_func[]={m_humidity_disable, m_humidity_disable, m_pressure_disable, m_color_disable, m_gas_disable, m_sound_disable};
sensor2str_func_t sensor2str_func[]={m_temperature2str, m_humidity2str, m_pressure2str, m_color2str, m_gas2str, m_sound2str};

void gas2ctx(void* gas_in, context_t* context_out) {
  gas_t gas = *((gas_t*)gas_in);
  context_out->timestamp_ms = gas.timestamp_ms;
  context_out->value1 = gas.ec02_ppm;
  context_out->value2 = gas.tvoc_ppb;
}

void ctx2gas(context_t* context_in, void** sensor_p) {
  gas_t* gas_p;
  gas_p = malloc(sizeof(gas_t));
  gas_p->ec02_ppm = context_in->value1 & 0xffff;
  gas_p->tvoc_ppb = context_in->value2 & 0xffff;
  gas_p->timestamp_ms = context_in->timestamp_ms;
  *sensor_p = gas_p;
}

void sound2ctx(void* sound_in, context_t* context_out) {
  sound_t sound = *((sound_t*)sound_in);
  memcpy(&context_out->value1, &sound.sound_level, sizeof(uint32_t));
  context_out->timestamp_ms = sound.timestamp_ms;
}

void ctx2sound(context_t* context_in, void** sensor_p) {
  sound_t* sound_p;
  sound_p = malloc(sizeof(sound_t));
  memcpy(&sound_p->sound_level, &context_in->value1, sizeof(uint32_t));
  //  sound_p->sound_level = (float)context_in->value1;
  sound_p->timestamp_ms = context_in->timestamp_ms;
  *sensor_p = sound_p;
}

void color2ctx(void* color_in, context_t* context_out){
  color_t color = *((color_t*)color_in);
  context_out->timestamp_ms = color.timestamp_ms;
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
  context_out->timestamp_ms = temp.timestamp_ms;
  context_out->value1 = (temp.integer<<8) + (temp.decimal & 0xff);
  context_out->value2 = 0;
}

void ctx2temp(context_t* context_in, void** sensor_p){
  temperature_t* temp;
  temp = malloc(sizeof(temperature_t));
  temp->timestamp_ms = context_in->timestamp_ms;
  temp->decimal = context_in->value1 & 0xff;
  temp->integer = (context_in->value1 >>8) & 0xff;
  *sensor_p = temp;
  return;
}

void humid2ctx(void* humid_in, context_t* context_out){
  humidity_t humid = *((humidity_t*)humid_in);
  context_out->timestamp_ms = humid.timestamp_ms;
  context_out->value1 = humid.humid & 0xff;
  context_out->value2 = 0;
  return;
}

void ctx2humid(context_t* context_in, void** sensor_p){
  humidity_t* humid;
  humid = malloc(sizeof(humidity_t));
  humid->timestamp_ms = context_in->timestamp_ms;
  humid->humid = context_in->value1 & 0xff;
  *sensor_p = humid;
  return;
}

void pressure2ctx(void* pressure_in, context_t* context_out){
  pressure_t pressure = *((pressure_t*) pressure_in);
  context_out->timestamp_ms = pressure.timestamp_ms;
  context_out->value1 = pressure.decimal & 0xff;
  context_out->value1 += (pressure.integer & 0xffffff) << 8;
  return;
}

void ctx2pressure(context_t* context_in, void** sensor_p){
  pressure_t* pressure;
  pressure = malloc(sizeof(pressure_t));
  pressure->timestamp_ms = context_in->timestamp_ms;
  pressure->decimal = context_in->value1 & 0xff;
  pressure->integer = context_in->value1 >> 8;
  *sensor_p = pressure;
  return;
}

context_t context_read(uint8_t ctx_type){
  if (ctx_type > max_ctx){
    NRF_LOG_ERROR("Context %d type exceeds", ctx_type);
    return;
  }
  void ** cur_sensor_ptr = malloc(sizeof(void*));
  sensor_read_func[ctx_type](cur_sensor_ptr);
  context_t new_context={-1,ctx_type,0,0,0};
  sensor2ctx_func[ctx_type](*cur_sensor_ptr, &new_context);
  free(cur_sensor_ptr);
  return new_context;
}

void context_start(uint8_t ctx_type){
  if (ctx_type > max_ctx){
    NRF_LOG_ERROR("Context %d type exceeds", ctx_type);
    return;
  }
  sensor_start_func[ctx_type]();
  return;
}

void context_pause(uint8_t ctx_type){
  if (ctx_type > max_ctx){
    NRF_LOG_ERROR("Context %d type exceeds", ctx_type);
    return;
  }
  if ((ctx_type == LOCATION_CTX) || (ctx_type == VOC_CTX)){
    return;
  }
  context_stop(ctx_type);
}

uint32_t context_stop(uint8_t ctx_type){
  if (ctx_type > max_ctx){
    NRF_LOG_ERROR("Context %d type exceeds", ctx_type);
    return;
  }
  return sensor_stop_func[ctx_type]();
}


void context2str(context_t context_in, char* str_out){
  uint8_t ctype = context_in.ctx_type;
  if (ctype > max_ctx){
    NRF_LOG_ERROR("Context %d type exceeds", ctype);
    return;
  }
  void ** cur_sensor_ptr = malloc(sizeof(void*));
  ctx2sensor_func[ctype](&context_in, cur_sensor_ptr);
  sensor2str_func[ctype](*cur_sensor_ptr, str_out);
  free(*cur_sensor_ptr);
  free(cur_sensor_ptr);
}
 
