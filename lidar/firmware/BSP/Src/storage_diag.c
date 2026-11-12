/* Read-only storage diagnostics: W25Q128 JEDEC + 24C02 first byte. */
#include "storage_diag.h"

#include <string.h>

#include "i2c1.h"
#include "spi1.h"
#include "timebase.h"

static StorageDiag diag;

void storage_diag_init(void)
{
    memset(&diag, 0, sizeof diag);
    uint32_t t0 = timebase_cycles();

    diag.w25q_ok = spi1_init() && w25q_read_jedec(diag.w25q_jedec);
    diag.eeprom_ok = i2c1_init() && eeprom_read_byte(0x00U, &diag.eeprom_byte0);

    diag.read_us = (timebase_cycles() - t0) / 168U; /* nominal 168 cycles/us */
    diag.fresh = true;
}

const StorageDiag *storage_diag_get(void)
{
    return &diag;
}
