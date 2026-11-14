#pragma once
#include "nav_types.h"
#define NAV_LIDAR_MAP_MAX 48U
typedef struct {float x,y,r;uint32_t seen;unsigned hits;} NavLidarMemory;
void nav_lidar_map_clear(void);
/* Trusted geometry injection used by deterministic tests. Sensor observations
 * must use track(), which confirms distinct scans before permanent occupancy. */
void nav_lidar_map_observe(float x,float y,float r,uint32_t now);
void nav_lidar_map_track(uint16_t id,float x,float y,float r,uint32_t scan,uint32_t now);
void nav_lidar_map_freeze(bool frozen);
bool nav_lidar_map_frozen(void);
unsigned nav_lidar_map_candidates(void);
unsigned nav_lidar_map_outliers(void);
bool nav_lidar_map_safe(float x,float y,float yaw,float margin);
bool nav_lidar_map_view_safe(const NavLidarMemory *,unsigned count,bool full,float x,float y,float yaw,float margin);
unsigned nav_lidar_map_count(void);
bool nav_lidar_map_full(void);
const NavLidarMemory *nav_lidar_map_items(void);
