#pragma once
#include <stdbool.h>
#include <stdint.h>
bool nav_lidar_front150(float ex,float ey,float *x,float *y,float *dx,float *dy,bool segment);
void nav_lidar_echo_reset(void);
bool nav_lidar_echo_accept(uint16_t id,float bx,float by,uint32_t stamp,bool departure,bool known_obstacle);
