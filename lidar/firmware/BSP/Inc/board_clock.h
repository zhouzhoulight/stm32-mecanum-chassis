#ifndef BOARD_CLOCK_H
#define BOARD_CLOCK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t hse_hz;
    uint32_t sysclk_hz;
    uint32_t hclk_hz;
    uint32_t apb1_hz;
    uint32_t apb2_hz;
    uint32_t pllm, plln, pllp, pllq;
    bool hse_ready;
    bool verified;
} BoardClockInfo;

/* Configure HSE25/PLL(M25,N336,P2,Q7) -> 168 MHz, APB1 42 / APB2 84 MHz and
 * verify the result by reading back registers (not just printing constants). */
bool board_clock_init(BoardClockInfo *out);
const BoardClockInfo *board_clock_info(void);
#endif
