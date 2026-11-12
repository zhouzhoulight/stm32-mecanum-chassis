#ifndef I2C1_H
#define I2C1_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool init_ok;
    uint32_t reads;
    uint32_t errors;
} I2c1Stats;

/* PB6 SCL / PB7 SDA only; 24C02 at 7-bit 0x50. Read-only diagnostics. */
bool i2c1_init(void);
bool eeprom_read_byte(uint8_t mem_addr, uint8_t *value);
const I2c1Stats *i2c1_stats(void);
#endif
