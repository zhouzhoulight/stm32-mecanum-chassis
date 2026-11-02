#ifndef NAV_TIME_H
#define NAV_TIME_H
#include <stdint.h>
/* ISR timestamp may lead a main-loop snapshot by one tick. Preserve wrap
 * arithmetic; tolerate at most two milliseconds of that causal read race. */
static inline uint32_t nav_age(uint32_t now,uint32_t at)
{
 int32_t elapsed=(int32_t)(now-at);
 return elapsed>=0?(uint32_t)elapsed:elapsed>=-2?0:UINT32_MAX;
}
#endif
