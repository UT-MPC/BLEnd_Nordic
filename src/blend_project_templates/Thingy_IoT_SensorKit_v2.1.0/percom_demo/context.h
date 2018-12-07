#ifndef _UTMPCCONTEXT_H
#define _UTMPCCONTEXT_H
#include "sensor.h"
#include "app_timer.h"
#include "sensor.h"
#include <string.h>
#include "pca20020.h"
#include <stdlib.h>
#define NUM_CONTEXT_TYPES 16

/*! \brief Structure for context values.
 */
typedef struct {
  uint8_t source_id; /*!< Identifer of the source node. */
  uint8_t ctx_type; /*!< Type of the context value contained. */   
  uint32_t value1; /*!< 32-bit context value. */
  uint32_t value2; /*!< (optional)32-bit context value. */
  uint32_t timestamp_ms; /*!< Received time in microseconds. */
} context_t;

/*! \brief Structure of the exchange packets.
 */
typedef struct {
  uint8_t node_id; /*!< Identifer of the source node. */
  uint16_t cap_vec; /*!< Bit vector for sensing capabilities. */
  uint16_t demand_vec; /*!< Bit vector for sensing capabilities. */
  context_t context;  /*!< Context values in the packet*/
  uint32_t timestamp_ms; /*!< Received time in microseconds. */
} decoded_packet_t;

typedef void (*sensor2ctx_func_t) (void* , context_t*);

// void temp2ctx(void* temp_in, context_t* context_out);
// void humid2ctx(void* temp_in, context_t* context_out);
// void pressure2ctx(void* temp_in, context_t* context_out);

// void sensor2ctx(void* sensor_in, context_t* context_out);
// void ctx2sensor(context_t* context_in, void* sensor_out);

void context_sample(uint8_t ctx_tye);
context_t context_read(uint8_t ctx_tye);

extern char* ctx_name[];
// sensor2ctx_func_t sensor2ctx_func[];


#endif