#ifndef _UTMPC_MATCHING_H
#define _UTMPC_MATCHING_H

#include <stdint.h>

#include "node.h"

/**@brief Entry of the selection program.
 * @param[in] snapshot    Ptr to the most recent snapshot.
 *
 * @return New assignment for the sensing task (context type).
 */
uint8_t get_new_assignment(node_t const *snapshot);

#endif // _UTMPC_MATCHING_H
