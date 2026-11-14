#ifndef NAV_MOTION_H
#define NAV_MOTION_H
/* RPM is the position coordinate of this scalar profile. Its velocity is
 * wheel acceleration (RPM/s); its acceleration is jerk (RPM/s^2). */
typedef struct {float rpm,accel;} NavMotion;
void nav_motion_step(NavMotion *,float target,float accel_limit,float jerk_limit,float dt);
#endif
