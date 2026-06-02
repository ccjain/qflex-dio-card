#ifndef APP_MODBUS_RTU_H
#define APP_MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MB_OK                        = 0x00,
    MB_EXC_ILLEGAL_FUNCTION      = 0x01,
    MB_EXC_ILLEGAL_DATA_ADDRESS  = 0x02,
    MB_EXC_ILLEGAL_DATA_VALUE    = 0x03,
} mb_error_t;

typedef struct {
    mb_error_t (*read_coils)(uint16_t addr, uint16_t qty, uint8_t *out_bits);
    mb_error_t (*write_single_coil)(uint16_t addr, bool value);
    mb_error_t (*write_multiple_coils)(uint16_t addr, uint16_t qty, const uint8_t *in_bits);
    mb_error_t (*read_discrete_inputs)(uint16_t addr, uint16_t qty, uint8_t *out_bits);
} mb_handlers_t;

/* CRC16-Modbus: poly 0xA001 (reflected), seed 0xFFFF. */
uint16_t mb_rtu_crc16(const uint8_t *data, size_t len);

/* Validate, dispatch, build response.
 *   req / req_len   - raw bytes from the UART idle-line buffer
 *   slave_id        - configured slave ID (1..15; 0 means broadcast on the wire)
 *   h               - application FC handlers
 *   resp / resp_len - output buffer + size in/out
 *
 * Returns true if *resp_len bytes were written and the caller MUST transmit.
 * Returns false on silent drop: wrong slave-ID, bad CRC, frame too short, or
 * broadcast (apply but don't reply). */
bool mb_rtu_process(const uint8_t *req, size_t req_len,
                    uint8_t slave_id,
                    const mb_handlers_t *h,
                    uint8_t *resp, size_t *resp_len);

#endif
