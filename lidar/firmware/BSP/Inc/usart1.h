#ifndef USART1_H
#define USART1_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t tx_frames;
    uint32_t tx_bytes;
    uint32_t tx_dropped_bytes;
    uint32_t rx_bytes;
    uint32_t rx_overruns;
    uint32_t uart_errors;
} Usart1Stats;

bool usart1_init(void);
/* Queue a complete frame (binary only; printf is never used on this port). */
bool usart1_send(const uint8_t *data, uint32_t len);
void usart1_service(void);              /* called from the main loop */
uint32_t usart1_read(uint8_t *dst, uint32_t max);
const Usart1Stats *usart1_stats(void);
void usart1_uart_irq(void);             /* USART1_IRQHandler body */
void usart1_dma_tx_irq(void);           /* DMA2_Stream7_IRQHandler body */
#endif
