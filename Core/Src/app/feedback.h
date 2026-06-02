#ifndef APP_FEEDBACK_H
#define APP_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

/* Pure-logic debounce step: takes the previous shift-register, the new
 * raw sample (0/1), and returns the new shift-register (low N bits). */
uint8_t feedback_debounce_shift(uint8_t history, uint8_t raw_bit);

/* Returns true once `history` is all-1s (debounced HIGH), false once
 * all-0s (debounced LOW), otherwise returns `previous`. */
bool feedback_debounce_value(uint8_t history, bool previous);

void feedback_init(void);
void feedback_scan(void);          /* call every APP_FEEDBACK_SCAN_PERIOD_MS */
bool feedback_get(uint8_t index);  /* debounced logical state, 1 = contactor closed */

#endif
