#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "stm32c0xx_hal.h"

/* ------------------------------------------------------------------
 * Heartbeat LED
 * ----------------------------------------------------------------*/
#define PIN_HEARTBEAT_PORT     GPIOF
#define PIN_HEARTBEAT_PIN      GPIO_PIN_3
#define PIN_HEARTBEAT_CLK_EN() __HAL_RCC_GPIOF_CLK_ENABLE()

/* ------------------------------------------------------------------
 * Relays (active-low, 7 channels). Indexed 0..6 to match coil addresses.
 * ----------------------------------------------------------------*/
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} pin_t;

#define RELAY_PINS_INIT { \
    { GPIOA, GPIO_PIN_15 },  /* R1 */ \
    { GPIOA, GPIO_PIN_12 },  /* R2 */ \
    { GPIOA, GPIO_PIN_11 },  /* R3 */ \
    { GPIOA, GPIO_PIN_10 },  /* R4 */ \
    { GPIOC, GPIO_PIN_7  },  /* R5 */ \
    { GPIOC, GPIO_PIN_6  },  /* R6 */ \
    { GPIOA, GPIO_PIN_9  },  /* R7 */ \
}
#define RELAY_CLK_EN() do { \
    __HAL_RCC_GPIOA_CLK_ENABLE(); \
    __HAL_RCC_GPIOC_CLK_ENABLE(); \
} while (0)

/* Active-low: LOW energises the coil. */
#define RELAY_ON_STATE   GPIO_PIN_RESET
#define RELAY_OFF_STATE  GPIO_PIN_SET

/* ------------------------------------------------------------------
 * Feedback inputs (active-low w/ pull-up, 12 channels)
 * ----------------------------------------------------------------*/
#define FEEDBACK_PINS_INIT { \
    { GPIOD, GPIO_PIN_0  },  /* FB1  */ \
    { GPIOD, GPIO_PIN_1  },  /* FB2  */ \
    { GPIOD, GPIO_PIN_2  },  /* FB3  */ \
    { GPIOD, GPIO_PIN_3  },  /* FB4  */ \
    { GPIOB, GPIO_PIN_3  },  /* FB5  */ \
    { GPIOB, GPIO_PIN_4  },  /* FB6  */ \
    { GPIOB, GPIO_PIN_5  },  /* FB7  */ \
    { GPIOB, GPIO_PIN_6  },  /* FB8  */ \
    { GPIOB, GPIO_PIN_7  },  /* FB9  */ \
    { GPIOB, GPIO_PIN_8  },  /* FB10 */ \
    { GPIOB, GPIO_PIN_9  },  /* FB11 */ \
    { GPIOC, GPIO_PIN_13 },  /* FB12 */ \
}
#define FEEDBACK_CLK_EN() do { \
    __HAL_RCC_GPIOB_CLK_ENABLE(); \
    __HAL_RCC_GPIOC_CLK_ENABLE(); \
    __HAL_RCC_GPIOD_CLK_ENABLE(); \
} while (0)

/* ------------------------------------------------------------------
 * DIP switch (active-low w/ pull-up, 4 bits: PA8 = LSB, PB13 = MSB)
 * ----------------------------------------------------------------*/
#define DIP_PINS_INIT { \
    { GPIOA, GPIO_PIN_8  },  /* DIP1 (LSB, weight 1) */ \
    { GPIOB, GPIO_PIN_15 },  /* DIP2 (weight 2)      */ \
    { GPIOB, GPIO_PIN_14 },  /* DIP3 (weight 4)      */ \
    { GPIOB, GPIO_PIN_13 },  /* DIP4 (MSB, weight 8) */ \
}
#define DIP_CLK_EN() do { \
    __HAL_RCC_GPIOA_CLK_ENABLE(); \
    __HAL_RCC_GPIOB_CLK_ENABLE(); \
} while (0)

/* ------------------------------------------------------------------
 * USART3 + RS485 DE/RE.  AF4 is the USART3 mapping on PB10/PB11 for STM32C0.
 * ----------------------------------------------------------------*/
#define PIN_UART_TX_PORT       GPIOB
#define PIN_UART_TX_PIN        GPIO_PIN_10
#define PIN_UART_RX_PORT       GPIOB
#define PIN_UART_RX_PIN        GPIO_PIN_11
#define PIN_UART_AF            GPIO_AF4_USART3
#define PIN_UART_CLK_EN()      __HAL_RCC_USART3_CLK_ENABLE()

#define PIN_RS485_DE_PORT      GPIOB
#define PIN_RS485_DE_PIN       GPIO_PIN_2
#define PIN_RS485_DE_CLK_EN()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define RS485_TX_MODE() HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_RX_MODE() HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_RESET)

#endif /* PIN_MAP_H */
