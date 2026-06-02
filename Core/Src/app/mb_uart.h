#ifndef APP_MB_UART_H
#define APP_MB_UART_H

#include <stddef.h>
#include <stdint.h>

void mb_uart_init(void);

/* Blocking-style helper used in early bring-up only. The frame-buffer
 * RX API + interrupt-driven TX (mb_uart_send, etc.) is added in later tasks. */
void mb_uart_tx_blocking(const uint8_t *data, size_t len);

#endif
