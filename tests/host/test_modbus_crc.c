#include "../../Core/Src/app/modbus_rtu.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Known-good Modbus CRC16 vectors (poly 0xA001, seed 0xFFFF).
 * Verified by hand and by Python during plan execution.
 * On the wire Modbus serialises LSB-first, so the function's host-uint16
 * result 0xE181 becomes wire bytes 0x81 0xE1. */
int main(void) {
    {
        uint8_t  v[] = {0x01, 0x02};
        uint16_t c = mb_rtu_crc16(v, sizeof(v));
        assert(c == 0xE181);
    }
    {
        uint8_t  v[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
        uint16_t c = mb_rtu_crc16(v, sizeof(v));
        assert(c == 0xCDC5);
    }
    {
        uint16_t c = mb_rtu_crc16(NULL, 0);
        assert(c == 0xFFFF);
    }
    printf("modbus_crc16: OK\n");
    return 0;
}
