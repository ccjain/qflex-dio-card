#include "relay.h"
#include "main.h"
#include "pin_map.h"

static const pin_t s_relay[APP_RELAY_COUNT] = RELAY_PINS_INIT;
static bool        s_state[APP_RELAY_COUNT];

void relay_init(void)
{
    RELAY_CLK_EN();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < APP_RELAY_COUNT; ++i) {
        /* Drive OFF before enabling output to avoid glitch. */
        HAL_GPIO_WritePin(s_relay[i].port, s_relay[i].pin, RELAY_OFF_STATE);
        g.Pin = s_relay[i].pin;
        HAL_GPIO_Init(s_relay[i].port, &g);
        s_state[i] = false;
    }
}

bool relay_get(uint8_t index)
{
    return (index < APP_RELAY_COUNT) ? s_state[index] : false;
}

void relay_set(uint8_t index, bool on)
{
    if (index < APP_RELAY_COUNT) {
        s_state[index] = on;
    }
}

void relay_apply(void)
{
    for (uint8_t i = 0; i < APP_RELAY_COUNT; ++i) {
        HAL_GPIO_WritePin(s_relay[i].port, s_relay[i].pin,
                          s_state[i] ? RELAY_ON_STATE : RELAY_OFF_STATE);
    }
}
