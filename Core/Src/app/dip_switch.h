#ifndef APP_DIP_SWITCH_H
#define APP_DIP_SWITCH_H

#include <stdint.h>

/* Pure-logic helper: take 4 pin-level booleans (closed=true, open=false)
 * and return the encoded slave ID 0..15. Exposed for unit testing. */
uint8_t dip_decode(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);

/* Initialise the 4 DIP pins as input pull-up and return the configured
 * slave ID. Call exactly once at boot. */
uint8_t dip_switch_read(void);

#endif
