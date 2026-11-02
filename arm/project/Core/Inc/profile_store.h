#ifndef ARM_PROFILE_STORE_H
#define ARM_PROFILE_STORE_H
#include <stdint.h>
/* Last 128KB sector is reserved for idle-only append records; no motion here. */
int Board_CalRead(void *payload,uint32_t bytes);
int Board_CalWrite(const void *payload,uint32_t bytes);
#endif
