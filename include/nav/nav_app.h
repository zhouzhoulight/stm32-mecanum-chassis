#ifndef NAV_APP_H
#define NAV_APP_H
#include "nav_types.h"
void nav_app_init(uint32_t now);
void nav_app_command(const char *line,uint32_t now);
void nav_app_tick(uint32_t now);
void nav_app_emergency(uint32_t now,const char *reason);
bool nav_app_active(void);
unsigned nav_app_state(void);
#endif
