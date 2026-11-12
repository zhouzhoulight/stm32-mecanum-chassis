#ifndef USART2_H
#define USART2_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t rx_bytes;
    uint32_t dma_ht_events;
    uint32_t dma_tc_events;
    uint32_t idle_events;
    uint32_t uart_errors;
    uint32_t overrun_bytes;
    uint32_t last_event_us;
} Usart2Stats;

bool usart2_init(void);
/* Drain bytes from the circular DMA ring. rx_end_us is the read-time estimate
 * for the last byte in this block; *discontinuous is set when the ring wrapped
 * past the reader (data was lost), *uart_error when UART error flags occurred. */
uint32_t usart2_read(uint8_t *dst, uint32_t max, uint32_t *rx_end_us,
                     bool *discontinuous, bool *uart_error);
const Usart2Stats *usart2_stats(void);
void usart2_uart_irq(void);            /* USART2_IRQHandler body (IDLE/errors) */
void usart2_dma_irq(void);             /* DMA1_Stream5_IRQHandler body */
#endif
