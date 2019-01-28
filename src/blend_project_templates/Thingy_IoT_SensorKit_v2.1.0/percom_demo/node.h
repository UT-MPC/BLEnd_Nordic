#ifndef _UTMPC_NODE_H
#define _UTMPC_NODE_H

#include <stdint.h>

/*! \brief Structure for a list of neighbor nodes.
 *
 */
typedef struct node {
  uint8_t node_id; /*!< Id of the discovered node. */
  uint16_t cap_vec; /*!< Bit vector for sensing capabilities. */
  uint16_t demand_vec; /*!< Bit vector for context demand. */
  uint32_t last_sync_ms; /*!< Time of last sync in microseconds */
  struct node *next;
} node_t;

#endif // _UTMPC_NODE_H
