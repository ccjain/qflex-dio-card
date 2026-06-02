#include "feedback.h"

#ifndef HOST_UNIT_TEST
#include "main.h"
#include "pin_map.h"
#endif

#include "app_config.h"

#define DEBOUNCE_N    APP_FEEDBACK_DEBOUNCE_N
#define DEBOUNCE_MASK ((1u << DEBOUNCE_N) - 1u)

uint8_t feedback_debounce_shift(uint8_t history, uint8_t raw_bit)
{
    history = (uint8_t)((history << 1) | (raw_bit & 1u));
    return (uint8_t)(history & DEBOUNCE_MASK);
}

bool feedback_debounce_value(uint8_t history, bool previous)
{
    if ((history & DEBOUNCE_MASK) == DEBOUNCE_MASK) return true;
    if ((history & DEBOUNCE_MASK) == 0u)            return false;
    return previous;
}

#ifndef HOST_UNIT_TEST

static const pin_t s_fb[APP_FEEDBACK_COUNT] = FEEDBACK_PINS_INIT;
static uint8_t     s_hist[APP_FEEDBACK_COUNT];
static bool        s_value[APP_FEEDBACK_COUNT];
static uint32_t    s_last_ms;

void feedback_init(void)
{
    FEEDBACK_CLK_EN();
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint8_t i = 0; i < APP_FEEDBACK_COUNT; ++i) {
        g.Pin = s_fb[i].pin;
        HAL_GPIO_Init(s_fb[i].port, &g);
        s_hist[i]  = 0;
        s_value[i] = false;
    }
    s_last_ms = HAL_GetTick();
}

void feedback_scan(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ms) < APP_FEEDBACK_SCAN_PERIOD_MS) return;
    s_last_ms = now;

    for (uint8_t i = 0; i < APP_FEEDBACK_COUNT; ++i) {
        /* Pin reads 0 when contactor is closed -> logical 1. Invert. */
        uint8_t raw = (HAL_GPIO_ReadPin(s_fb[i].port, s_fb[i].pin) == GPIO_PIN_RESET) ? 1u : 0u;
        s_hist[i]  = feedback_debounce_shift(s_hist[i], raw);
        s_value[i] = feedback_debounce_value(s_hist[i], s_value[i]);
    }
}

bool feedback_get(uint8_t index)
{
    return (index < APP_FEEDBACK_COUNT) ? s_value[index] : false;
}

#endif /* HOST_UNIT_TEST */
