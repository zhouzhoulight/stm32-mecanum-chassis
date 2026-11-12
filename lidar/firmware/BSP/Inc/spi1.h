#ifndef SPI1_H
#define SPI1_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool init_ok;
    bool cs_high_at_boot;
    uint32_t jedec_reads;
    uint32_t errors;
} Spi1Stats;

/* PC13 must be driven high BEFORE SPI1 is enabled (W25Q CS, never an LED). */
bool spi1_init(void);
bool w25q_read_jedec(uint8_t id[3]);
const Spi1Stats *spi1_stats(void);
#endif
