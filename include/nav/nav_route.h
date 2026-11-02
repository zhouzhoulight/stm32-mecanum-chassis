#ifndef NAV_ROUTE_H
#define NAV_ROUTE_H
#include "nav_types.h"
#define NAV_ROUTE_MAX 1536U
typedef struct {
 float x,y,h,s,v,t,cap;
 unsigned stop;
} NavRoutePoint;
typedef struct {
 NavRoutePoint p[NAV_ROUTE_MAX];
 unsigned n,index,station,stable,percent;
 float time,elapsed,hold,integral[3],wheel[4],error_m,yaw_error;
 float duration,length,lever,rpm_mps,scale_y,clock_rate,response_s;
 bool ready,finished;
 const char *fault;
} NavRoute;
bool nav_route_plan(NavRoute *,unsigned percent,unsigned ppm_x,unsigned ppm_y,float lever);
/* Local frame is fixed at zone1 center WEST; no origin/reset at waypoints. */
bool nav_route_footprint(float local_x,float local_y,float yaw,float margin);
int nav_route_step(NavRoute *,const NavPose *,float gyro,float dt,int wheel[4]);
const char *nav_route_station(unsigned station);
#endif
