#pragma once
#include "nav_types.h"
#define NAV_LIDAR_MAP_MAX 48U
typedef struct {float x,y,r;uint32_t seen;unsigned hits;} NavLidarMemory;
void nav_lidar_map_clear(void);
void nav_lidar_map_observe(float x,float y,float r,uint32_t now);
bool nav_lidar_map_safe(float x,float y,float yaw,float margin);
bool nav_lidar_map_view_safe(const NavLidarMemory *,unsigned count,bool full,float x,float y,float yaw,float margin);
unsigned nav_lidar_map_count(void);
bool nav_lidar_map_full(void);
const NavLidarMemory *nav_lidar_map_items(void);
