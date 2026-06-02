/* Application glue: implements the mb_handlers_t contracts using the
 * relay and feedback modules, and pumps RTU frames between the UART
 * transport and the protocol layer. */
#include "modbus_app.h"
#include "main.h"
#include "mb_uart.h"
#include "modbus_rtu.h"
#include "relay.h"
#include "feedback.h"
#include <string.h>

static uint8_t s_slave_id;
static uint8_t s_scratch[APP_MODBUS_FRAME_BUF];
static uint8_t s_resp[APP_MODBUS_FRAME_BUF];

static mb_error_t app_read_coils(uint16_t addr, uint16_t qty, uint8_t *out)
{
    if ((uint32_t)addr + qty > APP_RELAY_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    memset(out, 0, (qty + 7u) / 8u);
    for (uint16_t i = 0; i < qty; ++i) {
        if (relay_get((uint8_t)(addr + i))) {
            out[i / 8u] |= (uint8_t)(1u << (i % 8u));
        }
    }
    return MB_OK;
}

static mb_error_t app_write_single_coil(uint16_t addr, bool value)
{
    if (addr >= APP_RELAY_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    relay_set((uint8_t)addr, value);
    return MB_OK;
}

static mb_error_t app_write_multiple_coils(uint16_t addr, uint16_t qty, const uint8_t *in)
{
    if ((uint32_t)addr + qty > APP_RELAY_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    for (uint16_t i = 0; i < qty; ++i) {
        bool v = ((in[i / 8u] >> (i % 8u)) & 1u) != 0u;
        relay_set((uint8_t)(addr + i), v);
    }
    return MB_OK;
}

static mb_error_t app_read_discrete_inputs(uint16_t addr, uint16_t qty, uint8_t *out)
{
    if ((uint32_t)addr + qty > APP_FEEDBACK_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    memset(out, 0, (qty + 7u) / 8u);
    for (uint16_t i = 0; i < qty; ++i) {
        if (feedback_get((uint8_t)(addr + i))) {
            out[i / 8u] |= (uint8_t)(1u << (i % 8u));
        }
    }
    return MB_OK;
}

static const mb_handlers_t s_handlers = {
    .read_coils           = app_read_coils,
    .write_single_coil    = app_write_single_coil,
    .write_multiple_coils = app_write_multiple_coils,
    .read_discrete_inputs = app_read_discrete_inputs,
};

void modbus_app_init(uint8_t slave_id) { s_slave_id = slave_id; }

void modbus_app_poll(void)
{
    size_t rx_len;
    if (!mb_uart_rx_ready(&rx_len)) return;

    size_t n = (rx_len < sizeof(s_scratch)) ? rx_len : sizeof(s_scratch);
    memcpy(s_scratch, mb_uart_rx_buffer(), n);
    mb_uart_rx_release();

    size_t resp_len = sizeof(s_resp);
    if (mb_rtu_process(s_scratch, n, s_slave_id, &s_handlers, s_resp, &resp_len)) {
        mb_uart_send(s_resp, resp_len);
    }
    /* else: silent drop (broadcast / bad CRC / wrong slave-id / too short). */
}
