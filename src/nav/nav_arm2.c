#include "nav/nav_arm2.h"
#include <math.h>
#include <stddef.h>

static float clamp(float x,float lo,float hi){return fmaxf(lo,fminf(hi,x));}
bool nav_arm2_valid(const Arm2Geometry *g)
{
 if(!g||!isfinite(g->pivot_x)||!isfinite(g->pivot_y)||!isfinite(g->tool_side)||
    !isfinite(g->yaw_min)||!isfinite(g->yaw_max)||g->yaw_max<=g->yaw_min||
    g->yaw_min<-6.2832f||g->yaw_max>6.2832f||g->yaw_max-g->yaw_min>6.2832f||
    !isfinite(g->reach_min)||!isfinite(g->reach_max)||g->reach_min<=0||g->reach_max<=g->reach_min||
    g->reach_max>1000||!isfinite(g->z_min)||!isfinite(g->z_max)||g->z_min>=g->z_max||
    !isfinite(g->safe_z)||g->safe_z<g->z_min||g->safe_z>g->z_max||fabsf(g->tool_side)>1000||
    !isfinite(g->sweep_halfwidth)||g->sweep_halfwidth<0||!isfinite(g->tool_below_tcp)||g->tool_below_tcp<0||
    !isfinite(g->settle_s)||g->settle_s<0)return false;
 for(unsigned i=0;i<3;i++)if(!isfinite(g->velocity[i])||g->velocity[i]<=0||
                             !isfinite(g->accel[i])||g->accel[i]<=0)return false;
 return true;
}
static bool in_range(const Arm2Geometry *g,Arm2Pose p)
{return p.yaw>=g->yaw_min&&p.yaw<=g->yaw_max&&p.reach>=g->reach_min&&p.reach<=g->reach_max&&p.z>=g->z_min&&p.z<=g->z_max;}
Arm2Point nav_arm2_tcp(const Arm2Geometry *g,Arm2Pose p)
{
 float c=cosf(p.yaw),s=sinf(p.yaw);
 return (Arm2Point){g->pivot_x+c*p.reach-s*g->tool_side,g->pivot_y+s*p.reach+c*g->tool_side,p.z};
}
bool nav_arm2_solve(const Arm2Geometry *g,Arm2Point goal,float near_yaw,bool locked,float tolerance,Arm2Solution *out)
{
 if(!out||!nav_arm2_valid(g)||!isfinite(near_yaw)||!isfinite(tolerance)||tolerance<0||
    !isfinite(goal.x)||!isfinite(goal.y)||!(goal.z>=g->z_min&&goal.z<=g->z_max))return false;
 if(locked&&!(near_yaw>=g->yaw_min&&near_yaw<=g->yaw_max))return false;
 float x=goal.x-g->pivot_x,y=goal.y-g->pivot_y,rad=hypotf(x,y);
 if(!isfinite(rad)||rad>10000)return false;
 float heading=atan2f(y,x);
 float ray=clamp(sqrtf(fmaxf(0,rad*rad-g->tool_side*g->tool_side)),g->reach_min,g->reach_max);
 float unconstrained=heading-atan2f(g->tool_side,ray);
 float best=INFINITY,best_turn=INFINITY;
 for(int k=-2;k<=2;k++){
  float yaw=locked?near_yaw:clamp(unconstrained+k*6.28318531f,g->yaw_min,g->yaw_max);
  float c=cosf(yaw),s=sinf(yaw),reach=clamp(c*x+s*y,g->reach_min,g->reach_max);
  float miss=hypotf(c*x+s*y-reach,-s*x+c*y-g->tool_side);
  float turn=fabsf(yaw-near_yaw);
  if(miss<best-.0001f||(fabsf(miss-best)<=.0001f&&turn<best_turn)){
   best=miss;best_turn=turn;*out=(Arm2Solution){{yaw,reach,goal.z},miss};
  }
  if(locked)break;
 }
 return best<=tolerance;
}
float nav_arm2_axis_time(float distance,float vmax,float accel)
{
 distance=fabsf(distance);
 if(distance*vmax==0)return 0;
 float ramp=vmax*vmax/accel;
 return distance<ramp?2*sqrtf(distance/accel):2*vmax/accel+(distance-ramp)/vmax;
}
static bool segment_box(Arm2Point a,Arm2Point b,const Arm2Box *box,float margin)
{
 const float lo[2]={box->xmin-margin,box->ymin-margin},hi[2]={box->xmax+margin,box->ymax+margin};
 const float p[2]={a.x,a.y},v[2]={b.x-a.x,b.y-a.y};
 float enter=0,leave=1;
 for(unsigned k=0;k<2;k++){
  if(fabsf(v[k])<1e-6f){if(p[k]<lo[k]||p[k]>hi[k])return false;continue;}
  float u=(lo[k]-p[k])/v[k],w=(hi[k]-p[k])/v[k];
  enter=fmaxf(enter,fminf(u,w));leave=fminf(leave,fmaxf(u,w));
 }
 return enter<=leave;
}
static bool sweep_clear(const Arm2Geometry *g,Arm2Pose a,Arm2Pose b,const Arm2Box *boxes,unsigned n)
{
 /* Check the whole range, including an independently timed rack. The radial
  * centreline covers all shorter extensions. Tool offset and half-width
  * dilate it, and the angular margin covers half the gap between samples. */
 float reach=fmaxf(a.reach,b.reach),z=fminf(a.z,b.z),angle=b.yaw-a.yaw;
 unsigned steps=(unsigned)ceilf(fabsf(angle)/.035f)+1;
 float margin=g->sweep_halfwidth+fabsf(g->tool_side)+reach*fabsf(angle)/(2*steps);
 Arm2Point pivot={g->pivot_x,g->pivot_y,z};
 for(unsigned k=0;k<n;k++){
  if(z-g->tool_below_tcp>boxes[k].height)continue;
  for(unsigned j=0;j<=steps;j++){
   float yaw=a.yaw+angle*j/steps;
   Arm2Point end={pivot.x+reach*cosf(yaw),pivot.y+reach*sinf(yaw),z};
   if(segment_box(pivot,end,boxes+k,margin))return false;
  }
 }
 return true;
}
static bool stage(const Arm2Geometry *g,Arm2Plan *p,Arm2Pose *at,Arm2Pose to,const Arm2Box *boxes,unsigned count)
{
 float distances[]={to.yaw-at->yaw,to.reach-at->reach,to.z-at->z},duration=0;
 uint8_t mask=0;
 for(unsigned i=0;i<3;i++)if(fabsf(distances[i])>.00001f){
  mask|=1U<<i;duration=fmaxf(duration,nav_arm2_axis_time(distances[i],g->velocity[i],g->accel[i]));
 }
 if(!mask)return true;
 if(p->count>=5||!sweep_clear(g,*at,to,boxes,count))return false;
 duration+=g->settle_s;p->stage[p->count++]=(Arm2Stage){to,duration,mask};p->seconds+=duration;
 *at=to;return true;
}
bool nav_arm2_transfer(const Arm2Geometry *g,Arm2Pose a,Arm2Pose b,const Arm2Box *boxes,unsigned n,bool parallel,Arm2Plan *out)
{
 if(!out)return false;
 *out=(Arm2Plan){0};
 if(!nav_arm2_valid(g)||!in_range(g,a)||!in_range(g,b)||n>16||(n&&!boxes))return false;
 for(unsigned i=0;i<n;i++)if(!isfinite(boxes[i].xmin)||!isfinite(boxes[i].xmax)||
    !isfinite(boxes[i].ymin)||!isfinite(boxes[i].ymax)||!isfinite(boxes[i].height)||
    boxes[i].xmin>=boxes[i].xmax||boxes[i].ymin>=boxes[i].ymax)return false;
 if(fabsf(a.yaw-b.yaw)<=.00001f&&fabsf(a.reach-b.reach)<=.00001f){
  Arm2Pose at=a;
  return stage(g,out,&at,b,boxes,n);
 }
 float heights[18];unsigned hc=0;
 heights[hc++]=g->safe_z;heights[hc++]=g->z_max;
 for(unsigned i=0;i<n;i++){
  float h=boxes[i].height+g->tool_below_tcp+1;
  if(h>g->safe_z&&h<g->z_max)heights[hc++]=h;
 }
 float fastest=INFINITY;
 for(unsigned h=0;h<hc;h++)for(unsigned order=0;order<3;order++){
  if(order==2&&!parallel)continue;
  float z=heights[h];
  float za=fmaxf(z,a.z),zb=fmaxf(z,b.z);
  if(order<2)za=zb=fmaxf(za,zb);
  Arm2Plan candidate={0};Arm2Pose p=a;
  if(!stage(g,&candidate,&p,(Arm2Pose){a.yaw,a.reach,za},boxes,n))continue;
  if(order<2){
   Arm2Pose mid=order==0?(Arm2Pose){b.yaw,a.reach,za}:(Arm2Pose){a.yaw,b.reach,za};
   if(!stage(g,&candidate,&p,mid,boxes,n))continue;
  }
  /* Height changes during horizontal motion are legal only when the entire
   * independently timed sweep also clears the scene at the lower height. */
  if(!stage(g,&candidate,&p,(Arm2Pose){b.yaw,b.reach,zb},boxes,n)||
     !stage(g,&candidate,&p,b,boxes,n))continue;
  if(candidate.seconds<fastest){fastest=candidate.seconds;*out=candidate;}
 }
 return isfinite(fastest);
}
