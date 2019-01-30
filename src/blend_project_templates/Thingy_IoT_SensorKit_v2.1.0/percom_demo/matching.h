#ifndef _UTMPC_MATCHING_H
#define _UTMPC_MATCHING_H

#include <stdint.h>
#include <stdio.h>

#include "node.h"

/**@brief Entry of the selection program.
 * @param[in] snapshot    Const ptr. to the most recent snapshot.
 * @param[in] local_idx   Index of localhost.
 *
 * @return New assignment for the sensing task (context type).
 */
uint8_t get_new_assignment(node_t const *snapshot, uint8_t local_idx);

/**@brief Convert the snapshot into a matrix.
 *
 * @details Matrix dimension (m = n_ngbr, n = n_types)
 * @param[in] snapshot    Const ptr. to the most recent snapshot.
 * @param[in] n_types    Number of context types.
 * @param[in] n_ngbr    Size of the neighborhood.
 *
 * @return Matrix representation of the snapshot.
 */
int* convert_to_matrix(node_t const *snapshot, size_t n_ngbr, size_t n_types);

#endif // _UTMPC_MATCHING_H
