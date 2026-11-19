#ifndef NAV_VISION_H
#define NAV_VISION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct {
 uint8_t cmd, status, length;
 uint32_t received_at;
 uint8_t data[87]; /* largest supported result: six raw candidates */
} NavVisionResult;
bool nav_vision_request(unsigned cmd,unsigned profile,const uint8_t *args,unsigned count,uint32_t timeout,uint32_t now);
bool nav_vision_pending(void);
const NavVisionResult *nav_vision_result(void);
void nav_vision_cancel(uint32_t now);
void nav_vision_feed(uint8_t,uint32_t);
void nav_vision_tick(uint32_t);
void nav_vision_error(void);
bool nav_vision_command(const char *,uint32_t);
bool nav_vision_hw_send(const uint8_t *,size_t);
#endif
