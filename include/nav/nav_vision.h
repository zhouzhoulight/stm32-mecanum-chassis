#ifndef NAV_VISION_H
#define NAV_VISION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void nav_vision_feed(uint8_t,uint32_t);
void nav_vision_tick(uint32_t);
void nav_vision_error(void);
bool nav_vision_command(const char *,uint32_t);
bool nav_vision_hw_send(const uint8_t *,size_t);
#endif
