#include "mb_uart.h"
#include "main.h"
#include "pin_map.h"

static UART_HandleTypeDef s_uart;

void mb_uart_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    PIN_UART_CLK_EN();
    PIN_RS485_DE_CLK_EN();

    /* DE/RE pin: output, default RX mode. */
    GPIO_InitTypeDef g = {0};
    g.Pin   = PIN_RS485_DE_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_RS485_DE_PORT, &g);
    RS485_RX_MODE();

    /* TX/RX AF pins. */
    g.Pin       = PIN_UART_TX_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = PIN_UART_AF;
    HAL_GPIO_Init(PIN_UART_TX_PORT, &g);

    g.Pin       = PIN_UART_RX_PIN;
    HAL_GPIO_Init(PIN_UART_RX_PORT, &g);

    /* USART3 9600 8N1. */
    s_uart.Instance        = USART3;
    s_uart.Init.BaudRate   = APP_MODBUS_BAUD;
    s_uart.Init.WordLength = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits   = UART_STOPBITS_1;
    s_uart.Init.Parity     = UART_PARITY_NONE;
    s_uart.Init.Mode       = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling   = UART_OVERSAMPLING_16;
    s_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    s_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&s_uart) != HAL_OK) { Error_Handler(); }
}

void mb_uart_tx_blocking(const uint8_t *data, size_t len)
{
    RS485_TX_MODE();
    (void)HAL_UART_Transmit(&s_uart, (uint8_t *)data, (uint16_t)len, 1000);
    RS485_RX_MODE();
}
