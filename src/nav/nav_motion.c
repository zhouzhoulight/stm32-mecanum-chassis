#include "nav/nav_motion.h"
#include <math.h>
/* Analytic acceleration-limited scalar position profile, applied one order
 * higher to wheel RPM. Replan from the CURRENT RPM/acceleration each tick,
 * never restart at zero at the dock cap boundary or a guide node.
 * Same construction as the standard triangular/trapezoidal profile; no
 * iteration, allocation or double-precision math. Emergency stop bypasses it. */
void nav_motion_step(NavMotion *s,float target,float limit,float jerk,float dt)
{
 float error=target-s->rpm;
 /* An online target can enter the current braking distance. Brake first,
  * allowing the unavoidable overshoot; never snap velocity/acceleration to
  * the new target. A plain rest-to-rest formula is infeasible in this case. */
 if(s->accel!=0&&error*s->accel>=0&&fabsf(error)<s->accel*s->accel/(2*jerk)){
  float direction=s->accel>0?1.f:-1.f;
  float stop=fabsf(s->accel)/jerk,t=fminf(dt,stop);
  float next=s->accel-direction*jerk*t;
  s->rpm+=(s->accel+next)*.5f*t;s->accel=next;
  if(dt>stop)nav_motion_step(s,target,limit,jerk,dt-stop);
  return;
 }
 float direction=s->rpm>target?-1.f:1.f;
 float x=direction*s->rpm,v=direction*s->accel,goal=direction*target;
 v=fmaxf(-limit,fminf(limit,v));
 float cut=v/jerk;
 float distance=fmaxf(0,goal-x+.5f*v*v/jerk);
 float ramp=limit/jerk,cruise=distance-ramp*ramp*jerk;
 if(cruise<0){ramp=sqrtf(distance/jerk);cruise=0;}
 float t1=ramp-cut,t2=t1+cruise/limit,t3=t2+ramp;
 if(dt<t1){x+=(v+.5f*jerk*dt)*dt;v+=jerk*dt;}
 else if(dt<t2){x+=(v+.5f*jerk*t1)*t1+limit*(dt-t1);v=limit;}
 else if(dt<t3){float left=t3-dt;x=goal-.5f*jerk*left*left;v=jerk*left;}
 else{x=goal;v=0;}
 s->rpm=direction*x;s->accel=direction*v;
}
