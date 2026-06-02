#include "dip_switch.h"

#ifndef HOST_UNIT_TEST
#include "main.h"
#include "pin_map.h"
#endif

uint8_t dip_decode(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    return (uint8_t)((b0 & 1u)
                   | ((b1 & 1u) << 1)
                   | ((b2 & 1u) << 2)
                   | ((b3 & 1u) << 3));
}

#ifndef HOST_UNIT_TEST
uint8_t dip_switch_read(void)
{
    static const pin_t dip[APP_DIP_BIT_COUNT] = DIP_PINS_INIT;
    DIP_CLK_EN();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (int i = 0; i < APP_DIP_BIT_COUNT; ++i) {
        g.Pin = dip[i].pin;
        HAL_GPIO_Init(dip[i].port, &g);
    }

    /* Settling time for pull-ups against external wiring/RC. */
    HAL_Delay(2);

    /* Switch closed -> pin reads 0 -> bit value 1.  Invert. */
    uint8_t b[APP_DIP_BIT_COUNT];
    for (int i = 0; i < APP_DIP_BIT_COUNT; ++i) {
        b[i] = (HAL_GPIO_ReadPin(dip[i].port, dip[i].pin) == GPIO_PIN_RESET) ? 1u : 0u;
    }
    return dip_decode(b[0], b[1], b[2], b[3]);
}
#endif
