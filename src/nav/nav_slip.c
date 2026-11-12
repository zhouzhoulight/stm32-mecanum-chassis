#include "nav/nav_slip.h"
#include <math.h>
#include <string.h>
void nav_slip_init(NavSlip*s,const float b[2])
{memset(s,0,sizeof*s);s->bias[0]=b[0];s->bias[1]=b[1];s->reliable=isfinite(b[0])&&isfinite(b[1]);}
bool nav_slip_step(NavSlip*s,float dt,float yaw,const float acc[2],const float wheel[2],bool still,bool verified)
{
 if(!verified||!isfinite(dt)||dt<.005f||dt>.1f||!isfinite(yaw)||
    !isfinite(acc[0])||!isfinite(acc[1])||!isfinite(wheel[0])||!isfinite(wheel[1])||
    hypotf(acc[0],acc[1])>6||hypotf(wheel[0],wheel[1])>2){s->reliable=false;return false;}
 float c=cosf(yaw),q=sinf(yaw),a[2]={c*(acc[0]-s->bias[0])-q*(acc[1]-s->bias[1]),q*(acc[0]-s->bias[0])+c*(acc[1]-s->bias[1])};
 float measured[2]={c*wheel[0]-q*wheel[1],q*wheel[0]+c*wheel[1]},predicted[2];
 for(unsigned k=0;k<2;k++)predicted[k]=s->velocity[k]+.5f*(s->previous_acc[k]+a[k])*dt;
 float innovation=hypotf(measured[0]-predicted[0],measured[1]-predicted[1]);
 if(!s->bridging&&innovation>.035f&&hypotf(measured[0],measured[1])>.04f){s->bridging=true;s->age_s=0;s->matches=0;s->match_s=0;s->events++;}
 if(s->bridging){
  s->age_s+=dt;s->matches=innovation<.025f?s->matches+1:0;
  s->match_s=innovation<.025f?s->match_s+dt:0;
  if(s->match_s>=.09999f)s->bridging=false;
  if(s->age_s>2){s->expired++;s->bridging=false;s->reliable=false;}
 }
 for(unsigned k=0;k<2;k++)s->velocity[k]=s->bridging?predicted[k]:measured[k];
 if(still&&hypotf(measured[0],measured[1])<.002f){
  float beta=expf(-dt/4);for(unsigned k=0;k<2;k++){s->velocity[k]=0;s->bias[k]=beta*s->bias[k]+(1-beta)*acc[k];}
 }
 for(unsigned k=0;k<2;k++){s->position[k]+=s->velocity[k]*dt;s->previous_acc[k]=a[k];}
 return s->reliable;
}
