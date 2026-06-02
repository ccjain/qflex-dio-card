#include "../../Core/Src/app/modbus_rtu.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Tiny in-memory model. Mirrors APP_RELAY_COUNT=7 / APP_FEEDBACK_COUNT=12. */
static uint8_t fake_coils[8];
static uint8_t fake_inputs[12];

static mb_error_t h_read_coils(uint16_t addr, uint16_t qty, uint8_t *out) {
    if ((uint32_t)addr + qty > 7) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    memset(out, 0, (qty + 7) / 8);
    for (uint16_t i = 0; i < qty; ++i) {
        if (fake_coils[addr + i]) out[i / 8] |= (uint8_t)(1u << (i % 8));
    }
    return MB_OK;
}
static mb_error_t h_write_single_coil(uint16_t addr, bool v) {
    if (addr >= 7) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    fake_coils[addr] = v ? 1 : 0;
    return MB_OK;
}
static mb_error_t h_write_multiple_coils(uint16_t addr, uint16_t qty, const uint8_t *in) {
    if ((uint32_t)addr + qty > 7) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    for (uint16_t i = 0; i < qty; ++i) {
        fake_coils[addr + i] = (in[i / 8] >> (i % 8)) & 1u;
    }
    return MB_OK;
}
static mb_error_t h_read_di(uint16_t addr, uint16_t qty, uint8_t *out) {
    if ((uint32_t)addr + qty > 12) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    memset(out, 0, (qty + 7) / 8);
    for (uint16_t i = 0; i < qty; ++i) {
        if (fake_inputs[addr + i]) out[i / 8] |= (uint8_t)(1u << (i % 8));
    }
    return MB_OK;
}

static const mb_handlers_t H = {
    .read_coils           = h_read_coils,
    .write_single_coil    = h_write_single_coil,
    .write_multiple_coils = h_write_multiple_coils,
    .read_discrete_inputs = h_read_di,
};

/* Append CRC (LSB-first) to a frame body. */
static size_t build(uint8_t *buf, const uint8_t *body, size_t body_len) {
    memcpy(buf, body, body_len);
    uint16_t crc = mb_rtu_crc16(buf, body_len);
    buf[body_len + 0] = (uint8_t)(crc & 0xFF);
    buf[body_len + 1] = (uint8_t)(crc >> 8);
    return body_len + 2;
}

int main(void) {
    uint8_t req[64], resp[64];
    size_t  req_len, resp_len;

    /* 1. Read 7 coils, all zero on boot. */
    memset(fake_coils, 0, sizeof(fake_coils));
    req_len = build(req, (uint8_t[]){0x01,0x01,0x00,0x00,0x00,0x07}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp_len == 6);
    assert(resp[0]==0x01 && resp[1]==0x01 && resp[2]==0x01 && resp[3]==0x00);

    /* 2. Write single coil 0 ON. */
    req_len = build(req, (uint8_t[]){0x01,0x05,0x00,0x00,0xFF,0x00}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp_len == 8);
    assert(fake_coils[0] == 1);

    /* 3. Read it back. */
    req_len = build(req, (uint8_t[]){0x01,0x01,0x00,0x00,0x00,0x01}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[3] == 0x01);

    /* 4. Bad write-coil value -> exception 03. */
    req_len = build(req, (uint8_t[]){0x01,0x05,0x00,0x00,0x12,0x34}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[1] == 0x85);
    assert(resp[2] == MB_EXC_ILLEGAL_DATA_VALUE);

    /* 5. Read out of range -> exception 02. */
    req_len = build(req, (uint8_t[]){0x01,0x01,0x00,0x00,0x00,0x08}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[1] == 0x81);
    assert(resp[2] == MB_EXC_ILLEGAL_DATA_ADDRESS);

    /* 6. Read all 12 DIs with FB5 set. */
    memset(fake_inputs, 0, sizeof(fake_inputs));
    fake_inputs[4] = 1;
    req_len = build(req, (uint8_t[]){0x01,0x02,0x00,0x00,0x00,0x0C}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[2] == 0x02);
    assert(resp[3] == 0x10);
    assert(resp[4] == 0x00);

    /* 7. Write 7 coils with pattern 0x55. */
    req_len = build(req, (uint8_t[]){0x01,0x0F,0x00,0x00,0x00,0x07,0x01,0x55}, 8);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp_len == 8);
    assert(fake_coils[0]==1 && fake_coils[1]==0 && fake_coils[2]==1
        && fake_coils[3]==0 && fake_coils[4]==1 && fake_coils[5]==0 && fake_coils[6]==1);

    /* 8. Unsupported FC -> exception 01. */
    req_len = build(req, (uint8_t[]){0x01,0x03,0x00,0x00,0x00,0x01}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[1] == 0x83);
    assert(resp[2] == MB_EXC_ILLEGAL_FUNCTION);

    /* 9. Wrong slave-ID -> silent drop. */
    req_len = build(req, (uint8_t[]){0x07,0x01,0x00,0x00,0x00,0x01}, 6);
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));

    /* 10. Bad CRC -> silent drop. */
    req_len = build(req, (uint8_t[]){0x01,0x01,0x00,0x00,0x00,0x01}, 6);
    req[req_len - 1] ^= 0xFF;
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));

    /* 11. Broadcast write -> applied, no reply. */
    memset(fake_coils, 0, sizeof(fake_coils));
    req_len = build(req, (uint8_t[]){0x00,0x05,0x00,0x02,0xFF,0x00}, 6);
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(fake_coils[2] == 1);

    /* 12. Broadcast read -> silent drop, no apply. */
    req_len = build(req, (uint8_t[]){0x00,0x01,0x00,0x00,0x00,0x01}, 6);
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));

    /* 13. Frame too short -> silent drop. */
    resp_len = sizeof(resp);
    assert(!mb_rtu_process((uint8_t[]){0x01,0x01}, 2, 1, &H, resp, &resp_len));

    printf("modbus_rtu: 13 cases OK\n");
    return 0;
}
