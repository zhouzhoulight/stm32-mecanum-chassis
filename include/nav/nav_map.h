#ifndef NAV_MAP_H
#define NAV_MAP_H
#include "nav_types.h"
#define NAV_MAP_NODES 256
typedef struct {float x,y,h;unsigned stop;} NavMapNode;
typedef struct {unsigned kind,id;} NavObstacle;
bool nav_map_set(unsigned round,const NavObstacle *,unsigned count);
bool nav_map_set_positions(unsigned round,const uint16_t *x,const uint16_t *y,unsigned count);
bool nav_map_valid(void);
void nav_map_invalidate(void);
unsigned nav_map_generation(void);
bool nav_map_has_obstacles(void);
void nav_map_plan_profile(unsigned percent,unsigned forward_ppm);
bool nav_map_qr_enabled(void);
void nav_map_plan_start(float x,float y,float yaw);
bool nav_map_detour(NavMapNode *,unsigned *count,const NavMapNode *start,const NavMapNode *goals,unsigned goal_count,const char **why);
bool nav_map_sensed_path(NavMapNode *,unsigned *,const char **why);
bool nav_map_path(NavMapNode *,unsigned *n,const char **why);
bool nav_map_fixture_core(float local_x,float local_y,float inset);
bool nav_map_footprint(float x,float y,float h,float margin);
bool nav_map_command(const char *);
void nav_map_report(void);
bool nav_map_arm_safe(float x,float y,float h,float radius,float ox,float oy);
#endif
