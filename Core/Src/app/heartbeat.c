#include "heartbeat.h"
#include "main.h"

#define HB_PORT  GPIOF
#define HB_PIN   GPIO_PIN_3

static heartbeat_mode_t s_mode;
static uint32_t s_last_ms;

static uint32_t period_ms(void) {
    return (s_mode == HEARTBEAT_FAULT) ? APP_HEARTBEAT_FAULT_MS
                                       : APP_HEARTBEAT_NORMAL_MS;
}

void heartbeat_init(heartbeat_mode_t mode) {
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = HB_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HB_PORT, &g);
    HAL_GPIO_WritePin(HB_PORT, HB_PIN, GPIO_PIN_RESET);

    s_mode    = mode;
    s_last_ms = HAL_GetTick();
}

void heartbeat_set_mode(heartbeat_mode_t mode) { s_mode = mode; }

void heartbeat_tick(void) {
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ms) >= period_ms()) {
        HAL_GPIO_TogglePin(HB_PORT, HB_PIN);
        s_last_ms = now;
    }
}
