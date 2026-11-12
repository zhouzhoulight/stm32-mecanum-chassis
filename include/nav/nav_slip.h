#ifndef NAV_SLIP_H
#define NAV_SLIP_H
#include <stdbool.h>
/* Experimental, CURRENT IMU + drive encoders only. Not enabled in navigation.
 * Caller must supply common-time, gravity-removed acceleration in BODY axes,
 * SI units, verified mounting/scale, and an independently qualified still flag.
 * No function writes motors, resets map origin or claims absolute XY accuracy. */
typedef struct {
 float position[2],velocity[2],bias[2],previous_acc[2],age_s,match_s;
 unsigned matches,events,expired;
 bool bridging,reliable;
} NavSlip;
void nav_slip_init(NavSlip *,const float bias[2]);
bool nav_slip_step(NavSlip *,float dt,float yaw,const float acceleration[2],
                   const float wheel_velocity[2],bool stationary,bool verified);
#endif
