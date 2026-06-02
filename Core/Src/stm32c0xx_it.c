#include "main.h"
#include "stm32c0xx_it.h"
#include "mb_uart.h"

void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void SVC_Handler(void)        {}
void PendSV_Handler(void)     {}
void SysTick_Handler(void)    { HAL_IncTick(); }

/* STM32C092 routes USART3 and USART4 to the same NVIC line. */
void USART3_4_IRQHandler(void) { mb_uart_irq_handler(); }
