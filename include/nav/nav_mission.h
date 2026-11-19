#ifndef NAV_MISSION_H
#define NAV_MISSION_H
#include "nav_types.h"
void nav_mission_init(void);
bool nav_mission_command(const char *line,uint32_t now);
void nav_mission_tick(uint32_t now);
unsigned nav_coordination_features(void);
unsigned nav_mission_indexing_mode(void);
unsigned nav_mission_index_accel(void);
unsigned nav_mission_index_jerk(void);
bool nav_mission_work_pose(void);
unsigned nav_mission_home(bool *park_only);
void nav_mission_select_park(void);
bool nav_mission_owns_vision(void);
#endif
