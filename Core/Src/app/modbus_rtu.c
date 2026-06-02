/* Pure Modbus RTU protocol layer: CRC16, frame validation, FC dispatch.
 * No hardware dependencies -- testable on the host. */
#include "modbus_rtu.h"
#include <string.h>

#define FC_READ_COILS              0x01
#define FC_READ_DISCRETE_INPUTS    0x02
#define FC_WRITE_SINGLE_COIL       0x05
#define FC_WRITE_MULTIPLE_COILS    0x0F

#define MIN_FRAME_LEN              4   /* slave + fc + payload>=0 + crc(2) */

uint16_t mb_rtu_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static size_t build_exception(uint8_t *resp, uint8_t slave_id, uint8_t fc, mb_error_t exc)
{
    resp[0] = slave_id;
    resp[1] = (uint8_t)(fc | 0x80u);
    resp[2] = (uint8_t)exc;
    uint16_t crc = mb_rtu_crc16(resp, 3);
    resp[3] = (uint8_t)(crc & 0xFFu);
    resp[4] = (uint8_t)(crc >> 8);
    return 5;
}

static size_t finalize(uint8_t *resp, size_t body_len)
{
    uint16_t crc = mb_rtu_crc16(resp, body_len);
    resp[body_len + 0] = (uint8_t)(crc & 0xFFu);
    resp[body_len + 1] = (uint8_t)(crc >> 8);
    return body_len + 2u;
}

bool mb_rtu_process(const uint8_t *req, size_t req_len,
                    uint8_t slave_id,
                    const mb_handlers_t *h,
                    uint8_t *resp, size_t *resp_len)
{
    if (req_len < MIN_FRAME_LEN) return false;

    /* CRC check: the trailing 2 bytes are CRC-low / CRC-high. */
    uint16_t calc_crc  = mb_rtu_crc16(req, req_len - 2);
    uint16_t frame_crc = (uint16_t)req[req_len - 2]
                       | ((uint16_t)req[req_len - 1] << 8);
    if (calc_crc != frame_crc) return false;

    uint8_t addr = req[0];
    uint8_t fc   = req[1];
    bool    is_broadcast = (addr == 0u);

    if (!is_broadcast && addr != slave_id) return false;

    mb_error_t exc = MB_OK;
    size_t     body_len = 0u;

    switch (fc) {
        case FC_READ_COILS:
        case FC_READ_DISCRETE_INPUTS: {
            if (is_broadcast) return false;   /* reads are not broadcast */
            if (req_len < 8u) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            uint16_t a = ((uint16_t)req[2] << 8) | req[3];
            uint16_t q = ((uint16_t)req[4] << 8) | req[5];
            if (q == 0u || q > 2000u) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }

            uint8_t bytes = (uint8_t)((q + 7u) / 8u);
            resp[0] = slave_id;
            resp[1] = fc;
            resp[2] = bytes;
            memset(&resp[3], 0, bytes);
            exc = (fc == FC_READ_COILS)
                ? h->read_coils(a, q, &resp[3])
                : h->read_discrete_inputs(a, q, &resp[3]);
            if (exc == MB_OK) body_len = 3u + bytes;
            break;
        }
        case FC_WRITE_SINGLE_COIL: {
            if (req_len < 8u) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            uint16_t a = ((uint16_t)req[2] << 8) | req[3];
            uint16_t v = ((uint16_t)req[4] << 8) | req[5];
            if (v != 0x0000u && v != 0xFF00u) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            exc = h->write_single_coil(a, v == 0xFF00u);
            if (exc == MB_OK) {
                memcpy(resp, req, 6);       /* echo addr/fc/coil/value */
                body_len = 6u;
            }
            break;
        }
        case FC_WRITE_MULTIPLE_COILS: {
            if (req_len < 9u) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            uint16_t a  = ((uint16_t)req[2] << 8) | req[3];
            uint16_t q  = ((uint16_t)req[4] << 8) | req[5];
            uint8_t  bc = req[6];
            if (q == 0u || q > 1968u
                || bc != (uint8_t)((q + 7u) / 8u)
                || req_len < (9u + bc)) {
                exc = MB_EXC_ILLEGAL_DATA_VALUE; break;
            }
            exc = h->write_multiple_coils(a, q, &req[7]);
            if (exc == MB_OK) {
                memcpy(resp, req, 6);       /* echo addr/fc/start/qty */
                body_len = 6u;
            }
            break;
        }
        default:
            exc = MB_EXC_ILLEGAL_FUNCTION;
            break;
    }

    if (is_broadcast) return false;          /* apply but never reply */
    if (exc != MB_OK) {
        *resp_len = build_exception(resp, slave_id, fc, exc);
        return true;
    }
    *resp_len = finalize(resp, body_len);
    return true;
}
