#ifndef NAV_CONTROL_H
#define NAV_CONTROL_H
#include "nav_types.h"
void nav_control_begin(NavControl *,const NavPose *,float x,float y,unsigned rpm,bool xy);
void nav_control_config(NavControl *s,unsigned algorithm,unsigned ppm_x,unsigned ppm_y,float lever,float b0_ratio);
int nav_control_step(NavControl *,const NavPose *,float yaw_error,float gyro,float dt,int rpm[4]);
#endif
