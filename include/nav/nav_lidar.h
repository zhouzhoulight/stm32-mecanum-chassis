#pragma once
#include "nav_types.h"
typedef bool (*NavLidarSend)(const uint8_t *,unsigned);
void nav_lidar_init(NavLidarSend send);
void nav_lidar_reset(void);
void nav_lidar_feed(uint8_t byte,uint32_t at);
void nav_lidar_error(void);
void nav_lidar_tick(uint32_t now,bool fused,float gyro);
void nav_lidar_departure(bool active);
bool nav_lidar_command(const char *line,bool idle,uint32_t now);
bool nav_lidar_clear(float x,float y,float yaw,float margin);
/* Planner-only immutable view; live braking never reads this snapshot. */
bool nav_lidar_plan_begin(void);
bool nav_lidar_plan_end(void);
/* Nonzero only while the immutable planner input/session is still valid. */
uint32_t nav_lidar_plan_token(void);
bool nav_lidar_plan_clear(float x,float y,float yaw,float margin);
void nav_lidar_plan_blockers(float x,float y,float yaw,float margin,bool live);
void nav_lidar_capture_hazard(uint32_t now,const NavPose *pose);
bool nav_lidar_fresh(uint32_t now);
bool nav_lidar_start_ready(uint32_t now,uint32_t since);
bool nav_lidar_avoid_ready(uint32_t now);
bool nav_lidar_hazard(uint32_t now,const NavPose *pose);
const char *nav_lidar_hazard_cause(void);
