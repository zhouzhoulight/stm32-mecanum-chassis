#ifndef STORAGE_DIAG_H
#define STORAGE_DIAG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool w25q_ok;
    uint8_t w25q_jedec[3];
    bool eeprom_ok;
    uint8_t eeprom_byte0;
    uint32_t read_us;
    bool fresh;
} StorageDiag;

/* One read-only probe at boot: W25Q JEDEC + 24C02 byte 0. Never erases or
 * writes either device. */
void storage_diag_init(void);
const StorageDiag *storage_diag_get(void);
#endif
