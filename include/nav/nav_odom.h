#ifndef NAV_ODOM_H
#define NAV_ODOM_H
#include "nav_types.h"
/* Yaw-rate plausibility gate: trust fresh IMU samples for short high-rate
 * transients, reject only when the rate stays implausible >300 ms. */
bool nav_odom_gyro_valid(float gyro,uint32_t now);
void nav_odom_init(void);
bool nav_odom_lidar_shift(float dx,float dy);
void nav_odom_yaw(int32_t mdeg,uint32_t at);
void nav_odom_wheel(uint8_t id,int32_t count,uint32_t at);
void nav_odom_tick(uint32_t now);
void nav_odom_rebind(bool origin);
bool nav_odom_restore_pose(float x,float y,float yaw);
bool nav_odom_config(unsigned ppm_x,unsigned ppm_y,unsigned imu_ms,unsigned encoder_ms,float lever);
const NavOdom *nav_odom_data(void);
float nav_odom_heading_now(void);
/* Bounded control-only extrapolation; never written back into odometry. */
bool nav_odom_predict(bool fused,uint32_t now,float gyro,NavPose *out);
/* Matched F6 ACK only, not evidence of actual wheel speed/arrival. */
void nav_odom_command(uint8_t id,int rpm,uint32_t at);
#endif
