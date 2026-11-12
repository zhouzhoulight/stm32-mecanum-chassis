#ifndef NAV_INCREMENT_H
#define NAV_INCREMENT_H
#include <stdbool.h>
bool nav_increment_angular_model(const float ds[4],float dtheta,float dt,float lever);
void nav_increment(const float ds[4],float dtheta,float dt,float ys,float lever,
                   float previous_vx,float previous_vy,float out[2],float diag[4]);
#endif
