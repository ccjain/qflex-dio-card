#include "main.h"
#include "heartbeat.h"
#include "mb_uart.h"
#include <string.h>

static void SystemClock_Config(void);

void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    heartbeat_init(HEARTBEAT_NORMAL);
    mb_uart_init();

    while (1) {
        heartbeat_tick();

        size_t rx_len;
        if (mb_uart_rx_ready(&rx_len)) {
            /* Snapshot then release so the next request can arrive
             * while we transmit. */
            uint8_t scratch[APP_MODBUS_FRAME_BUF];
            size_t n = (rx_len < sizeof(scratch)) ? rx_len : sizeof(scratch);
            memcpy(scratch, mb_uart_rx_buffer(), n);
            mb_uart_rx_release();
            mb_uart_send(scratch, n);
        }
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSIDiv         = RCC_HSI_DIV1;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                       | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
}
