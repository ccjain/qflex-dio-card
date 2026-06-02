/* USART3 transport for Modbus RTU.
 *
 * Note: We initialise USART3 via direct register writes rather than
 * HAL_UART_Init. The HAL UART driver on STM32C092 (CubeC0 v1.4.0) does
 * not bring the peripheral fully online on this silicon; the same
 * configuration applied to the registers directly works correctly.
 * See git history Phase D for the bring-up trail.
 */
#include "mb_uart.h"
#include "main.h"
#include "pin_map.h"

/* PCLK = 48 MHz, baud = 9600, BRR = 48e6 / 9600 = 5000. */
#define MB_USART3_BRR  ((uint32_t)(48000000UL / APP_MODBUS_BAUD))

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
    set_alt_func(GPIOB, 10, 4);     /* USART3_TX */
    set_alt_func(GPIOB, 11, 4);     /* USART3_RX */

    /* USART3 -> 9600 8N1, TE/RE/UE. */
    USART3->CR1 = 0;
    USART3->BRR = MB_USART3_BRR;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void mb_uart_tx_blocking(const uint8_t *data, size_t len)
{
    RS485_TX_MODE();
    for (size_t i = 0; i < len; ++i) {
        while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0u) { /* wait TXE */ }
        USART3->TDR = data[i];
    }
    /* Wait for the last byte to clear the shift register before dropping DE. */
    while ((USART3->ISR & USART_ISR_TC) == 0u) { /* wait TC */ }
    RS485_RX_MODE();
}
