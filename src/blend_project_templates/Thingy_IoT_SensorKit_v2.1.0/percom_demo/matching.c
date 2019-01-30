#include "matching.h"

#include <stdint.h>

#include "context.h"
#include "hungarian.h"
#include "nrf_log.h"

#define MAXUTIL 999

/**@brief Entry of the selection program.
 * @param[in] snapshot    Const ptr. to the most recent snapshot.
 * @param[in] local_idx   Index of localhost.
 *
 * @return New assignment for the sensing task (context type).
 */
uint8_t get_new_assignment(node_t const *snapshot, uint8_t local_idx) {
  if (snapshot) {
    size_t n_ngbr = 0;
    const node_t* it = snapshot;
    while(it) {
      ++n_ngbr;
      it = it->next;
    }
    
    hungarian_t prob;
    int* matrix = convert_to_matrix(snapshot, n_ngbr, NUM_CONTEXT_TYPES);
    hungarian_init(&prob, (int*)matrix, n_ngbr, NUM_CONTEXT_TYPES, HUNGARIAN_MIN);
    hungarian_solve(&prob);
    uint8_t task = prob.a[local_idx];
    // debug
    /* NRF_LOG_DEBUG("Matching details(device seq=%d, n_ngbr=%d, task=%d): \n", local_idx, n_ngbr, prob.a[0]); */
    /* it = snapshot; */
    /* int i = 0; */
    /* while(it) { */
    /*   NRF_LOG_DEBUG("| Device seq: %d, id: %d, cap: %d, task: %d \n", i, it->node_id, it->cap_vec, prob.a[i]); */
    /*   ++i; */
    /*   it = it->next; */
    /* } */
    // end of debug
    hungarian_fini(&prob);
    free(matrix);
    
    return task + TASK_OFFSET;
  }
  return TASK_IDLE;
}

/**@brief Convert the snapshot into a matrix.
 *
 * @details Matrix dimension (m = n_ngbr, n = n_types)
 * @param[in] snapshot    Const ptr. to the most recent snapshot.
 * @param[in] n_types    Number of context types.
 * @param[in] n_ngbr    Size of the neighborhood.
 *
 * @return Matrix representation of the snapshot.
 */
int* convert_to_matrix(node_t const *snapshot, size_t n_ngbr, size_t n_types) {
  int* ret = malloc(sizeof(int)*n_types*n_ngbr);
  const node_t* it = snapshot;
  for (int i = 0; i < n_ngbr; ++i) {
    for (int j = 0; j < n_types; ++j) {
      if (TestBit(it->cap_vec, j)) {
	ret[i*n_types + j] = 1; // OPTIONAL(liuchg): Relate to context demand model.
      } else {
	ret[i*n_types + j] = MAXUTIL;
      }
    }
    it = it->next;
  }
  return ret;
}
