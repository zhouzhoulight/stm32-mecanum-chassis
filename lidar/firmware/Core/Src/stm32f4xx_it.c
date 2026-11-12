/* Interrupt entries for the standalone bench board. ISRs only sample time,
 * indexes and error counters; all parsing/clustering happens in the main loop. */
#include "board_config.h"
#include "usart1.h"
#include "usart2.h"
#include "vehicle_uart.h"
void USART3_IRQHandler(void){vehicle_uart_irq();}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART1_IRQHandler(void)
{
    usart1_uart_irq();
}

void USART2_IRQHandler(void)
{
    usart2_uart_irq();
}

void DMA1_Stream5_IRQHandler(void)
{
    usart2_dma_irq();
}

void DMA2_Stream7_IRQHandler(void)
{
    usart1_dma_tx_irq();
}

void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
