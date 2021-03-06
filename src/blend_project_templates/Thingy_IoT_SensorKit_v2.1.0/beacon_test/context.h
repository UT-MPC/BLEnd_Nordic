#ifndef _UTMPCCONTEXT_H
#define _UTMPCCONTEXT_H

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "app_timer.h"
#include "pca20020.h"
#include "sensor.h"

#define NUM_CONTEXT_TYPES 16
//! Senisng task id = context type id + TASK_OFFET (Zero for idle)
#define TASK_OFFSET 1
#define TASK_IDLE 0

#define VALID_DURATION_DEFAULT_S 30
#define VALID_DURATION_TEMP_S VALID_DURATION_DEFAULT_S
#define VALID_DURATION_HUMID_S VALID_DURATION_DEFAULT_S
#define VALID_DURATION_PRESS_S VALID_DURATION_DEFAULT_S
#define VALID_DURATION_COLOR_S 10
#define VALID_DURATION_VOC_S VALID_DURATION_DEFAULT_S
#define VALID_DURATION_NOISE_S VALID_DURATION_DEFAULT_S
#define VALID_DURATION_LOCATION_S 5

#define SetBit(A,k) (A |= (1 << (k%NUM_CONTEXT_TYPES)))
#define ClearBit(A,k) (A &= ~(1 << (k%NUM_CONTEXT_TYPES)))
#define TestBit(A,k) (A & (1 << (k%NUM_CONTEXT_TYPES)))
// Extend to larger range using an array: (A[(k/NUM_CONTEXT_TYPES)] |= (1 << (k%NUM_CONTEXT_TYPES)))

#define CONTEXT_ALL_SIZE 20

// Constants for noise measurement
// #define MICROPHONE_ADPCM_RAW_THRESHOLD 20

typedef enum
{
    TEMP_CTX,
    HUMID_CTX,
    PRESS_CTX,
    COLOR_CTX,
    VOC_CTX,
    NOISE_CTX,
    LOCATION_CTX,
} ctx_type_def;

/*! \brief Structure for context values.
 */
typedef struct {
  uint8_t source_id; /*!< Identifer of the source node. */
  uint8_t ctx_type; /*!< Type of the context value contained. */
  uint32_t value1; /*!< 32-bit context value. */
  uint32_t value2; /*!< (optional)32-bit context value. */
  uint16_t value3;
  uint64_t timestamp_ms; /*!< Received time in microseconds. */
} context_t;

#pragma pack(push, 1)
typedef struct {

  //sound_t sound;
  int16_t avg_peak;
  int16_t max_peak;
  int16_t count_over_thres_per_frame;
  int16_t avg_all;
  int16_t avg_over_thres; 

  //temperature_t temperature;
  int8_t  temp_integer;
  uint8_t temp_decimal;

  //humidity_t humidity;
  uint8_t humid;

  //pressure_t pressure;
  uint8_t  pressure_decimal;
  int16_t  pressure_integer;

  //gas_t gas;
  uint16_t ec02_ppm; ///< The equivalent CO2 (eCO2) value in parts per million (ppm).
  uint16_t tvoc_ppb; ///< The Total Volatile Organic Compound (TVOC) value in parts per billion (ppb).

} context_all_t;
#pragma pack(pop)
/*! \brief Structure of the exchange packets.
 */
typedef struct {
  uint8_t node_id; /*!< Identifer of the source node. */
  uint16_t cap_vec; /*!< Bit vector for sensing capabilities. */
  uint16_t demand_vec; /*!< Bit vector for sensing capabilities. */
  bool is_ctx_valid; /*!< Whether the context is valid. */
  context_t context;  /*!< Context values in the packet*/
  uint64_t timestamp_ms; /*!< Received time in microseconds. */
} decoded_packet_t;

typedef void (*sensor2ctx_func_t) (void* , context_t*);
typedef void (*ctx2sensor_func_t) (context_t* , void**);

void context_start(uint8_t ctx_tye);
context_t context_read(uint8_t ctx_tye);
void context_all_to_bytes(uint8_t* bytes_temp, context_all_t* context_all);
context_all_t* context_read_all();
uint32_t context_stop(uint8_t ctx_type);
void context_pause(uint8_t ctx_tye);

/*
  @brief Convert a context to string. The char* passed in must 
  have enough space for the string
*/
void context2str(context_t context_in, char* str_out);

extern char* ctx_name[];
extern const uint64_t context_valid_duraion_s[];


#endif
