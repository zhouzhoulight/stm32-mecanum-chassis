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
 unsigned n,index,station,stable,percent,map_generation;
 unsigned max_rpm;float accel_limit,yaw_limit;
 float time,elapsed,hold,integral[3],wheel[4],error_m,yaw_error;
 float duration,length,lever,rpm_mps,scale_y,clock_rate,response_s,traction_scale;
 float wheel_accel[4],yaw_correction;
 unsigned smooth_profile;bool lowyaw;
 bool ready,finished,manual,waiting;
 NavControl tracker;
 /* v2.7.12 predicted-pose footprint guard diagnostics:
  * footprint_bad = consecutive control ticks whose PREDICTED pose left the
  * 0 mm-margin footprint (3 => sustained fault); predict_age_ms is set by
  * nav_app to the measured-odometry age used for that extrapolation;
  * guard_log_s throttles [ROUTE_GUARD] lines (route elapsed seconds). */
 uint8_t footprint_bad;
 uint32_t predict_age_ms;
 float guard_log_s;
 const char *fault;
} NavRoute;
bool nav_route_dynamics(unsigned rpm,unsigned accel_mmps2,unsigned yaw_mradps);
bool nav_route_plan_sensed(NavRoute *,unsigned percent,unsigned px,unsigned py,float lever);
bool nav_route_plan_qr(NavRoute *,unsigned percent,unsigned px,unsigned py,float lever);
bool nav_route_replan(NavRoute *,const NavPose *,unsigned px,unsigned py,float lever);
bool nav_route_plan(NavRoute *,unsigned percent,unsigned ppm_x,unsigned ppm_y,float lever);
/* Local frame is fixed at zone1 center WEST; no origin/reset at waypoints. */
bool nav_route_footprint(float local_x,float local_y,float yaw,float margin);
int nav_route_step(NavRoute *,const NavPose *,float gyro,float dt,int wheel[4]);
bool nav_route_controller(NavRoute *,unsigned algorithm,float b0_ratio);
const char *nav_route_station(unsigned station);
bool nav_route_yaw_gain(unsigned gain);
bool nav_route_smooth(unsigned profile);
void nav_route_lowyaw(bool enabled);
#endif
