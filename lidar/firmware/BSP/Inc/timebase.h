#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

/* TIM2 free-running 32-bit microsecond counter (PSC 83 at 84 MHz APB1 timer).
 * Differences MUST use unsigned arithmetic (wrap ~71.6 min). */
bool timebase_init(void);
uint32_t timebase_us(void);
static inline uint32_t timebase_diff_us(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b);
}
/* DWT cycle counter for operation timing (168 cycles/us nominal). */
void timebase_dwt_init(void);
uint32_t timebase_cycles(void);
#endif
