#include "main.h"

void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    /* Clock + peripherals come in later tasks. */
    while (1) {
        /* nothing yet */
    }
}
