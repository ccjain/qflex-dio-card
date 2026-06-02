#include "main.h"
#include "heartbeat.h"
#include "mb_uart.h"
#include "relay.h"
#include "feedback.h"
#include "dip_switch.h"
#include "modbus_app.h"

static void SystemClock_Config(void);

void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Outputs first so a glitch during init cannot energise a relay. */
    relay_init();
    relay_apply();

    feedback_init();
    heartbeat_init(HEARTBEAT_NORMAL);
    mb_uart_init();

    uint8_t slave_id = dip_switch_read();
    if (slave_id == 0u) {
        /* Config error: ignore Modbus, fast-blink heartbeat forever. */
        heartbeat_set_mode(HEARTBEAT_FAULT);
        while (1) { heartbeat_tick(); }
    }

    modbus_app_init(slave_id);

    while (1) {
        feedback_scan();
        relay_apply();
        modbus_app_poll();
        heartbeat_tick();
    }
}

/* HSI 48 MHz internal RC -> SYSCLK = HCLK = PCLK = 48 MHz.
 * STM32C0 has no PLL, so HSI direct (HSIDiv = 1) is the simplest 48 MHz path. */
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
