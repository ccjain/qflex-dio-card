#ifndef APP_RELAY_H
#define APP_RELAY_H

#include <stdbool.h>
#include <stdint.h>

void relay_init(void);
bool relay_get(uint8_t index);          /* index 0..APP_RELAY_COUNT-1 */
void relay_set(uint8_t index, bool on); /* updates in-memory state    */
void relay_apply(void);                 /* writes state to GPIOs      */

#endif
