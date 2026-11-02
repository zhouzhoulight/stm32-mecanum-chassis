#ifndef NAV_PLATFORM_H
#define NAV_PLATFORM_H
#include "nav_types.h"
bool nav_hw_tx(const uint8_t *p,size_t n);
bool nav_hw_tx_busy(void);
uint32_t nav_hw_tx_end(void);
void nav_log(const char *s);
void nav_logf(const char *fmt,...);
uint32_t nav_hw_cycles(void);
#endif
