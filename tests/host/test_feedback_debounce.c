#include "../../Core/Src/app/feedback.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    /* shift_left semantics: new bit shifted into LSB, mask to N bits (3). */
    uint8_t h = 0;
    h = feedback_debounce_shift(h, 1); assert(h == 0x01);
    h = feedback_debounce_shift(h, 1); assert(h == 0x03);
    h = feedback_debounce_shift(h, 1); assert(h == 0x07);   /* 3 in a row */

    /* debounce_value: needs N matching bits to switch. */
    bool prev = false;
    assert(feedback_debounce_value(0x07, prev) == true);
    assert(feedback_debounce_value(0x06, prev) == false);   /* not all 1s, stays prev */
    assert(feedback_debounce_value(0x00, true)  == false);  /* all 0s -> switch off */
    assert(feedback_debounce_value(0x05, true)  == true);   /* mixed -> stay prev */

    printf("feedback_debounce: OK\n");
    return 0;
}
