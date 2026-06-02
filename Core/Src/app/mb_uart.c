/* USART3 transport for Modbus RTU on the DIO card.
 *
 * Implementation notes:
 *   - Init is done by direct register writes. HAL_UART_Init on STM32C092
 *     (CubeC0 v1.4.0) does not bring USART3 fully online; bare-register
 *     init reproducing the same configuration works correctly. We use HAL
 *     for everything else (GPIO, RCC, NVIC, SysTick).
 *   - RX is interrupt-driven. RXNE fires per byte (stashed into the frame
 *     buffer); USART IDLE flag fires after 1 character time of bus silence
 *     and latches the frame for the application to consume. At 9600 8N1
 *     one char = 1.146 ms; Modbus masters always leave at least T3.5 = 4 ms
 *     between frames, so IDLE-based framing is more than adequate.
 *   - TX is polled (TXE per byte, then TC before dropping DE). Modbus is
 *     half-duplex, so no concurrent transmit pressure.
 */
#include "mb_uart.h"
#include "main.h"
#include "pin_map.h"

#define MB_USART3_BRR  ((uint32_t)(48000000UL / APP_MODBUS_BAUD))

static uint8_t           s_rx_buf[APP_MODBUS_FRAME_BUF];
static volatile uint16_t s_rx_idx;
static volatile uint16_t s_rx_frame_len;
static volatile bool     s_rx_ready;

static void set_alt_func(GPIO_TypeDef *port, uint32_t pin_pos, uint8_t af)
{
    uint32_t idx = pin_pos >> 3;        /* AFR[0] for pins 0-7, AFR[1] for 8-15 */
    uint32_t shift = (pin_pos & 7u) * 4u;
    uint32_t v = port->AFR[idx];
    v &= ~(0xFu << shift);
    v |=  ((uint32_t)af << shift);
    port->AFR[idx] = v;
}

void mb_uart_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    PIN_UART_CLK_EN();

    /* DE/RE GPIO. */
    GPIO_InitTypeDef g = {0};
    g.Pin   = PIN_RS485_DE_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_RS485_DE_PORT, &g);
    RS485_RX_MODE();

    /* PB10 (TX) and PB11 (RX) -> alternate-function mode 4 (USART3). */
    GPIOB->MODER   = (GPIOB->MODER   & ~((3u << (10*2)) | (3u << (11*2))))
                   | ((2u << (10*2)) | (2u << (11*2)));
    GPIOB->OSPEEDR |= (3u << (10*2)) | (3u << (11*2));
    GPIOB->PUPDR  &= ~((3u << (10*2)) | (3u << (11*2)));
    set_alt_func(GPIOB, 10, 4);
    set_alt_func(GPIOB, 11, 4);

    /* USART3 -> 9600 8N1, TE/RE/UE, RXNE + IDLE interrupts. */
    USART3->CR1 = 0;
    USART3->BRR = MB_USART3_BRR;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE
                | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE;

    /* NVIC for USART3 (shared IRQ with USART4 on STM32C092). */
    s_rx_idx       = 0;
    s_rx_frame_len = 0;
    s_rx_ready     = false;
    HAL_NVIC_SetPriority(USART3_4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);
}

void mb_uart_irq_handler(void)
{
    uint32_t isr = USART3->ISR;

    /* Byte received. */
    if (isr & USART_ISR_RXNE_RXFNE) {
        uint8_t b = (uint8_t)USART3->RDR;        /* reading RDR clears RXNE */
        if (!s_rx_ready && s_rx_idx < sizeof(s_rx_buf)) {
            s_rx_buf[s_rx_idx++] = b;
        }
    }

    /* Bus idle for >=1 char -> end of frame. */
    if (isr & USART_ISR_IDLE) {
        USART3->ICR = USART_ICR_IDLECF;
        if (!s_rx_ready && s_rx_idx > 0) {
            s_rx_frame_len = s_rx_idx;
            s_rx_ready     = true;
        }
    }

    /* Drop any overrun silently (we never want a stuck flag). */
    if (isr & USART_ISR_ORE) {
        USART3->ICR = USART_ICR_ORECF;
    }
}

bool mb_uart_rx_ready(size_t *len)
{
    if (!s_rx_ready) return false;
    if (len) *len = s_rx_frame_len;
    return true;
}

const uint8_t *mb_uart_rx_buffer(void) { return s_rx_buf; }

void mb_uart_rx_release(void)
{
    __disable_irq();
    s_rx_idx       = 0;
    s_rx_frame_len = 0;
    s_rx_ready     = false;
    __enable_irq();
}

void mb_uart_send(const uint8_t *data, size_t len)
{
    RS485_TX_MODE();
    for (size_t i = 0; i < len; ++i) {
        while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0u) { /* wait TXE */ }
        USART3->TDR = data[i];
    }
    while ((USART3->ISR & USART_ISR_TC) == 0u) { /* wait TC */ }
    RS485_RX_MODE();
}
