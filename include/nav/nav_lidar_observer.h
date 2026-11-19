#pragma once
#include <stdbool.h>
#include <stdint.h>
#define NAV_LIDAR_SEGMENT 0x0100U
typedef struct {float x,y,r;uint16_t id,flags;uint8_t kind;float dx,dy;} NavLidarFeature;
typedef struct {unsigned matches,points,lines;float dx,dy,rms,rank;} NavLidarFit;
/* Translation only: IMU heading and current scan-time projection are inputs,
 * not additional independent lidar observations. All units m in N frame. */
float nav_lidar_feature_cost(const NavLidarFeature *a,const NavLidarFeature *b);
bool nav_lidar_observer_fit(const NavLidarFeature *anchors,unsigned na,
                           const NavLidarFeature *observed,unsigned no,NavLidarFit *fit);
