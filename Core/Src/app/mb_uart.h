#ifndef APP_MB_UART_H
#define APP_MB_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

void mb_uart_init(void);

/* Half-duplex transmit. Raises DE, polls TXE for each byte, waits for TC,
 * lowers DE. Modbus is half-duplex so blocking is fine: the application
 * never has more than one response in flight. */
void mb_uart_send(const uint8_t *data, size_t len);

/* Frame reception (USART3 RXNE + IDLE interrupt-driven; software latched).
 *   - mb_uart_rx_ready(&len): true iff a complete frame is in the buffer
 *   - mb_uart_rx_buffer():    pointer to the latched frame
 *   - mb_uart_rx_release():   clear the latch, rearm reception. */
bool           mb_uart_rx_ready(size_t *len);
const uint8_t *mb_uart_rx_buffer(void);
void           mb_uart_rx_release(void);

/* USART3/4 IRQ entry: called from stm32c0xx_it.c. */
void mb_uart_irq_handler(void);

#endif
