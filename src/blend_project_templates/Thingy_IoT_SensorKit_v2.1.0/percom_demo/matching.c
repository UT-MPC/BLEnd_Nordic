#include "matching.h"

#include <stdint.h>

/**@brief Entry of the selection program.
 * @param[in] snapshot    Ptr to the most recent snapshot.
 *
 * @return New assignment for the sensing task (context type).
 */
uint8_t get_new_assignment(node_t const *snapshot) {
  if (snapshot) {
    return 0;
  }
  return 1;
}
