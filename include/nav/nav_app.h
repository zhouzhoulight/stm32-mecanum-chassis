#ifndef NAV_APP_H
#define NAV_APP_H
#include "nav_types.h"
void nav_app_init(uint32_t now);
void nav_app_command(const char *line,uint32_t now);
void nav_app_tick(uint32_t now);
void nav_app_emergency(uint32_t now,const char *reason);
bool nav_app_active(void);
bool nav_app_replanning(void);
unsigned nav_app_state(void);
bool nav_app_indexing(void);
unsigned nav_app_work_station(void);
bool nav_app_station_target(float pose[3]);
void nav_app_pose(float pose[3]);
bool nav_app_segmented(void);
bool nav_app_upcoming(float pose[3],unsigned *station,float *remaining);
bool nav_app_arm_approach_clear(float radius,float ox,float oy);
/* v2.7.14 distributed wheel-consistency guard. Public so the native regression
 * can exercise the exact decision logic with synthetic masks. Returns true when
 * the chassis must stop; *why receives the emergency reason. */
bool nav_app_wheel_guard(uint32_t now,uint8_t rejected,uint8_t ambiguous,unsigned pairs,float max_ratio,const char **why);
#endif
