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
typedef void (*ctx2sensor_func_t) (context_t* , void**);

void context_sample(uint8_t ctx_tye);
context_t context_read(uint8_t ctx_tye);

/*
  @brief Convert a context to string. The char* passed in must 
  have enough space for the string
*/
void context2str(context_t context_in, char* str_out);
extern char* ctx_name[];


#endif