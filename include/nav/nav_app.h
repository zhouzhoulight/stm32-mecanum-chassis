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
bool nav_app_official_plan_ready(void);
bool nav_app_work_clear(float x,float y);
bool nav_app_upcoming(float pose[3],unsigned *station,float *remaining);
bool nav_app_arm_approach_clear(float radius,float ox,float oy);
/* Default statistical rejection -> slower reference, continue the task.
 * Strict diagnostic mode alone returns true for a consistency emergency.
 * Hardware stall, stale feedback and transport guards remain independent. */
bool nav_app_wheel_guard(uint32_t now,uint8_t rejected,uint8_t ambiguous,unsigned pairs,float max_ratio,const char **why);
float nav_app_traction_scale(void);
#endif
