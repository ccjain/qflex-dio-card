#include "../../Core/Src/app/dip_switch.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    assert(dip_decode(0,0,0,0) == 0);   /* all open  */
    assert(dip_decode(1,0,0,0) == 1);   /* LSB only  */
    assert(dip_decode(0,1,0,0) == 2);
    assert(dip_decode(1,1,0,0) == 3);
    assert(dip_decode(0,0,1,0) == 4);
    assert(dip_decode(1,1,1,1) == 15);  /* all closed */
    /* Sanity: only the low bit matters per arg */
    assert(dip_decode(2,0,0,0) == 0);
    assert(dip_decode(3,0,0,0) == 1);
    printf("dip_decode: OK\n");
    return 0;
}
