#include "nav/studio_profile.h"
/* Firmware-native continuous SE(2) path, sampled speed-envelope and tracker.
 * This is NOT TOPPRA or a contact model. Only explicit task stops have v=0.
 * Static 300mm square vehicle / four 450mm obstacles; no random-obstacle sensor.
 */
#include "nav/nav_route.h"
#include "nav/nav_map.h"
#include "nav/nav_lidar.h"
#include "nav/nav_platform.h"
#include "nav/nav_control.h"
#include "nav/nav_motion.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846f
typedef NavMapNode Node;
static Node nodes[NAV_MAP_NODES];
static unsigned node_count;
static uint8_t smoothing_level[NAV_MAP_NODES];
static unsigned config_rpm=STUDIO_ROUTE_RPM;static float config_accel=STUDIO_ROUTE_ACCEL*.001f,config_yaw=STUDIO_ROUTE_YAW*.001f;
static float yaw_gain=STUDIO_ROUTE_YAW_GAIN*.001f;
static unsigned smooth_profile=1;static bool lowyaw_enabled=true;
#ifdef NAV_MOTION_SIL
static float motion_debug[5];
void nav_motion_debug(float *out){memcpy(out,motion_debug,sizeof motion_debug);}
#endif
bool nav_route_smooth(unsigned p){if(p>2)return false;smooth_profile=p;return true;}
void nav_route_lowyaw(bool enabled){lowyaw_enabled=enabled;}
bool nav_route_yaw_gain(unsigned gain){if(gain<2000||gain>8000)return false;yaw_gain=gain*.001f;return true;}
bool nav_route_dynamics(unsigned rpm,unsigned accel,unsigned yaw)
{
 if(rpm<50||rpm>400||accel<200||accel>1600||yaw<250||yaw>1800)return false;
 config_rpm=rpm;config_accel=accel*.001f;config_yaw=yaw*.001f;return true;
}
const char *nav_route_station(unsigned n)
{
 static const char *names[]={"guide","raw1","rough1","left1","raw2","rough2","left2","zone2","zone1","qr"};
 return n<=9?names[n]:"invalid";
}
static float wrap(float a){return atan2f(sinf(a),cosf(a));}
static float clip(float a,float b){return fmaxf(-b,fminf(b,a));}
static float phase_accel(const NavRoute *r,float path){
 if(!r->smooth_profile)return r->accel_limit;
 float u=fmaxf(0,fminf(1,(path-.20f)/.30f));
 /* Half acceleration leaving the dock, continuously recover the configured
  * acceleration by500mm. Planning, feedforward and allocator share this. */
 return r->accel_limit*(.5f+.5f*u*u*(3-2*u));
}
/* OBB / AABB separating axes. Expanded chassis includes explicit margin.
 * The field outline uses physical size: blue docks have zero outline margin. */
bool nav_route_footprint(float x,float y,float h,float margin)
{return nav_map_footprint(x,y,h,margin);}
static void tangent(unsigned k,float t[2])
{
 unsigned n=node_count;t[0]=t[1]=0;
 if(!k||k==n-1||nodes[k].stop)return;
 float a[2]={nodes[k].x-nodes[k-1].x,nodes[k].y-nodes[k-1].y};
 float b[2]={nodes[k+1].x-nodes[k].x,nodes[k+1].y-nodes[k].y};
 float la=hypotf(a[0],a[1]),lb=hypotf(b[0],b[1]);
 if(la<.001f||lb<.001f)return;
 float x=a[0]/la+b[0]/lb,y=a[1]/la+b[1]/lb,l=hypotf(x,y);
 if(l>.01f){float size=fminf(.04f,fminf(la,lb))*.8f;t[0]=x/l*size;t[1]=y/l*size;}
}
static void geometry(unsigned edge,float u,float out[3])
{
 const Node *a=&nodes[edge],*b=a+1;float ta[2],tb[2];tangent(edge,ta);tangent(edge+1,tb);
 float gain=smoothing_level[edge]==0?1.f:smoothing_level[edge]==1?.5f:0;
 for(unsigned k=0;k<2;k++){ta[k]*=gain;tb[k]*=gain;}
 float u2=u*u,u3=u2*u;
 float h00=2*u3-3*u2+1,h10=u3-2*u2+u,h01=-2*u3+3*u2,h11=u3-u2;
 out[0]=h00*a->x+h10*ta[0]+h01*b->x+h11*tb[0];
 out[1]=h00*a->y+h10*ta[1]+h01*b->y+h11*tb[1];
 out[2]=a->h+wrap(b->h-a->h)*(u3*(10+u*(-15+6*u)));
}
static void derivative(const NavRoute *r,unsigned k,float d[3])
{
 unsigned a=k?k-1:k,b=k+1<r->n?k+1:k;
 if(r->p[k].stop)b=k; /* do not average across a stopped reversal */
 if(k&&r->p[k-1].stop)a=k;
 float ds=r->p[b].s-r->p[a].s;
 d[0]=(r->p[b].x-r->p[a].x)/ds;d[1]=(r->p[b].y-r->p[a].y)/ds;d[2]=wrap(r->p[b].h-r->p[a].h)/ds;
}
static void wheel_coeff(const NavRoute *r,unsigned k,float w[4])
{
 float d[3];derivative(r,k,d);float c=cosf(r->p[k].h),s=sinf(r->p[k].h);
 float x=c*d[0]+s*d[1],y=(-s*d[0]+c*d[1])*r->scale_y,z=r->lever*d[2];
 w[0]=(x-y-z)/r->rpm_mps;w[1]=(x+y+z)/r->rpm_mps;
 w[2]=(x+y-z)/r->rpm_mps;w[3]=(x-y+z)/r->rpm_mps;
}
static bool build_route(NavRoute *r,unsigned percent,unsigned px,unsigned py,float lever)
{
 memset(r,0,sizeof *r);r->fault="invalid plan arguments";
 if(percent<25||percent>100||px<5000||px>30000||py<5000||py>30000||!isfinite(lever)||lever<.1f||lever>.5f||node_count<2)return false;
 r->map_generation=nav_map_generation();r->smooth_profile=smooth_profile;r->lowyaw=lowyaw_enabled;
 r->response_s=.16f;r->clock_rate=r->traction_scale=1;r->percent=percent;r->lever=lever;r->rpm_mps=3200.0f/px/60;r->scale_y=(float)py/px;r->tracker.b0_ratio=1;r->max_rpm=config_rpm*percent/100;r->accel_limit=config_accel;r->yaw_limit=config_yaw;
 /*100RPM grade can double its old0.45rad/s reference. The200RPM grade
  * retains0.9rad/s:1.2rad/s failed one held development case. Geometry and
  * station tolerances remain unchanged; no silent guard relaxation. */
 if(r->smooth_profile)r->yaw_limit=fminf(r->yaw_limit,.9f*100/percent);
 r->p[0]=(NavRoutePoint){.x=nodes[0].x,.y=nodes[0].y,.h=nodes[0].h};r->n=1;
 memset(smoothing_level,0,sizeof smoothing_level);
 unsigned nc=node_count;
 for(unsigned e=0;e<nc-1;e++){
  if(!nav_hw_plan_service()){r->fault="planning interrupted";return false;}
  unsigned destination=e+1;while(destination<nc&&!nodes[destination].stop)destination++;
  float l=hypotf(nodes[e+1].x-nodes[e].x,nodes[e+1].y-nodes[e].y);
  unsigned count=(unsigned)ceilf(fmaxf(l,.25f*fabsf(wrap(nodes[e+1].h-nodes[e].h)))/.004f);
  if(!count){r->fault="duplicate route node";return false;} /* geometry checked at <=~6mm, even skipped storage points */
  /* Graph edges/fillets may be safe while the extra Hermite tangent bulges
   * into a neighbouring occupied ROI. Select shape BEFORE emitting samples;
   * never relax the vehicle, obstacle or 15mm clearance checks. */
  for(unsigned level=0;level<3;level++){
   smoothing_level[e]=(uint8_t)level;bool clear=true;
   for(unsigned j=1;j<=count;j++){
    if(!(j&15U)&&!nav_hw_plan_service()){r->fault="planning interrupted";return false;}
    float q[3];geometry(e,(float)j/count,q);
    if(!nav_route_footprint(q[0],q[1],q[2],.015f)){clear=false;break;}
   }
   if(clear){if(level)nav_logf("[PLAN_SMOOTHING] edge=%u tangent_permille=%u; same footprint checks, no margin relaxation\r\n",e,level==1?500:0);break;}
  }
  for(unsigned j=1;j<=count;j++){
   if(!(j&15U)&&!nav_hw_plan_service()){r->fault="planning interrupted";return false;}
   float q[3];geometry(e,(float)j/count,q);
   if(!nav_route_footprint(q[0],q[1],q[2],.015f)){
    nav_logf("[PLAN_COLLISION] edge=%u sample=%u/%u local_mm=%d,%d map_mm=%d,%d yaw_mdeg=%d; from=%d,%d to=%d,%d\r\n",e,j,count,(int)roundf(q[0]*1000),(int)roundf(q[1]*1000),(int)roundf((2.25f-q[0])*1000),(int)roundf((2.25f-q[1])*1000),(int)roundf(q[2]*57295.7795f),(int)roundf(nodes[e].x*1000),(int)roundf(nodes[e].y*1000),(int)roundf(nodes[e+1].x*1000),(int)roundf(nodes[e+1].y*1000));
    r->fault="sampled footprint intersects obstacle/outline";return false;
   }
   NavRoutePoint *a=&r->p[r->n-1];float ds=hypotf(q[0]-a->x,q[1]-a->y)+.25f*fabsf(wrap(q[2]-a->h));
   if(ds<.025f&&j!=count)continue;
   if(ds<.000001f){r->fault="zero metric route span";return false;}
   if(r->n==NAV_ROUTE_MAX){r->fault="path storage exhausted";return false;}
   NavRoutePoint *p=&r->p[r->n++];p->x=q[0];p->y=q[1];p->h=q[2];p->s=a->s+ds;
   p->stop=j==count?nodes[e+1].stop:0;
   /* Doubling previous 75/100 upper bounds. Reserve 12% for feedback. */
   p->cap=(l>=.65f&&fabsf(wrap(nodes[e+1].h-nodes[e].h))<.01f?1.0f:.75f)*r->max_rpm*.88f;
   /* The taught raw pick point has only ~19mm physical platter clearance.
    * Bound the final250mm approach independently of open-corridor speed;
    * never move the taught endpoint or disable the footprint guard. */
   if(destination<nc&&(nodes[destination].stop==1||nodes[destination].stop==4)&&hypotf(q[0]-nodes[destination].x,q[1]-nodes[destination].y)<.25f)p->cap=fminf(p->cap,30.0f*percent*.01f); /* The original250mm dock approach must cross
    * zero-stop guide nodes; inserting a safe junction must not truncate it
    * to a30mm final edge. Round-5:30RPM creep on the last
    * 250 mm of raw pick stops (1/4); measured-pose overshoot at 40 RPM was 16 mm,
    * which entered the raw-fixture 5 mm guard; 30 RPM gives ~9 mm. Safety-neutral
    * (slower), only the final approach zone is affected. */
   if(nodes[e+1].stop==7||nodes[e+1].stop==8)p->cap=fminf(p->cap,25.0f*percent*.01f); /* short zero-margin blue-dock approach only */
   if(p->s<.10f)p->cap=fminf(p->cap,25.0f*percent*.01f); /* continuous blue-dock departure, no stop */
   else if(r->smooth_profile&&p->s<.22f){
    /* Blend out of the dock creep zone over120mm of path metric. Keep the
     * same cruise cap; remove the cap step which excited response feedforward. */
    float u=(p->s-.10f)/.12f,h=u*u*u*(10+u*(-15+6*u));
    float creep=25.0f*percent*.01f;
    p->cap=fminf(p->cap,creep+fmaxf(0,p->cap-creep)*h);
   }
  }
 }
 r->p[0].cap=r->p[1].cap;r->length=r->p[r->n-1].s;
 for(unsigned k=0;k<r->n;k++){
  if(!(k&15U)&&!nav_hw_plan_service()){r->fault="planning interrupted";return false;}
  float d[3],w[4];derivative(r,k,d);wheel_coeff(r,k,w);
  float largest=0;for(unsigned j=0;j<4;j++)largest=fmaxf(largest,fabsf(w[j]));
  float cap=r->p[k].cap/fmaxf(largest,.001f);
  if(fabsf(d[2])>.001f)cap=fminf(cap,(r->yaw_limit*percent*.01f)/fabsf(d[2])); /* 5x old .20 rad/s */
  if(k&&k+1<r->n&&!r->p[k].stop){
   float a[3],b[3];derivative(r,k-1,a);derivative(r,k+1,b);
   float curvature=hypotf(b[0]-a[0],b[1]-a[1])/(r->p[k+1].s-r->p[k-1].s);
   cap=fminf(cap,sqrtf(r->accel_limit/fmaxf(curvature,.001f)));
  }
  r->p[k].v=(!k||r->p[k].stop)?0:cap;
 }
 /* Conservative reachability in squared path speed u=v^2.
  * For each wheel coefficient w(s), a sufficient interval bound is
  * |mean(w)|*|ub-ua| + |wb-wa|*(ua+ub) <= 2*A_w*ds.
  * It bounds the actual finite-difference wheel acceleration using
  * (va+vb)^2 <= 2*(ua+ub), including wheel sign reversals.
  * This replaces repeated arbitrary .97 contractions. */
 for(unsigned k=1;k<r->n;k++){
  float wa[4],wb[4];wheel_coeff(r,k-1,wa);wheel_coeff(r,k,wb);
  float ds=r->p[k].s-r->p[k-1].s;
  float wheel_accel=200*phase_accel(r,(r->p[k].s+r->p[k-1].s)*.5f)/.60f;
  for(unsigned m=0;m<4;m++){
   float d=fabsf(wb[m]-wa[m]);
   if(d>.00001f){float v=sqrtf(wheel_accel*ds/d);r->p[k-1].v=fminf(r->p[k-1].v,v);r->p[k].v=fminf(r->p[k].v,v);}
  }
 }
 for(unsigned k=0;k<r->n;k++)r->p[k].v*=r->p[k].v;
 for(unsigned pass=0;pass<8;pass++)for(unsigned dir=0;dir<2;dir++)for(unsigned j=1;j<r->n;j++){
  if(!(j&15U)&&!nav_hw_plan_service()){r->fault="planning interrupted";return false;}
  unsigned a=dir?r->n-j:j-1,b=dir?a-1:j;
  float ds=fabsf(r->p[b].s-r->p[a].s),ua=r->p[a].v;
  float acceleration=phase_accel(r,(r->p[a].s+r->p[b].s)*.5f);
  float wheel_accel=200*acceleration/.60f;
  float bound=ua+2*acceleration*ds,wa[4],wb[4];wheel_coeff(r,a,wa);wheel_coeff(r,b,wb);
  for(unsigned m=0;m<4;m++){
   float c=fabsf((wa[m]+wb[m])*.5f),d=fabsf(wb[m]-wa[m]);
   if(c+d>.00001f)bound=fminf(bound,(2*wheel_accel*ds+(c-d)*ua)/(c+d));
  }
  r->p[b].v=fminf(r->p[b].v,fmaxf(0,bound));
 }
 for(unsigned k=0;k<r->n;k++)r->p[k].v=sqrtf(r->p[k].v);
 for(unsigned k=1;k<r->n;k++){
  float ds=r->p[k].s-r->p[k-1].s,va=r->p[k-1].v,vb=r->p[k].v;
  float dt=2*ds/fmaxf(.00001f,va+vb),wa[4],wb[4];wheel_coeff(r,k-1,wa);wheel_coeff(r,k,wb);
  float wheel_accel=200*phase_accel(r,(r->p[k].s+r->p[k-1].s)*.5f)/.60f;
  for(unsigned m=0;m<4;m++)if(fabsf(wb[m]*vb-wa[m]*va)>wheel_accel*dt*1.002f){r->fault="wheel acceleration envelope audit failed";return false;}
 }
 for(unsigned k=1;k<r->n;k++){
  float sum=r->p[k-1].v+r->p[k].v;
  if(sum<.00001f){r->fault="unreachable zero-speed edge";return false;}
  r->p[k].t=r->p[k-1].t+2*(r->p[k].s-r->p[k-1].s)/sum;
 }
 r->duration=r->p[r->n-1].t;r->ready=true;r->fault="none";return true;
}
static bool plan_fixed(NavRoute *r,unsigned percent,unsigned px,unsigned py,float lever)
{
 if(percent<25||percent>100||px<5000||px>30000||py<5000||py>30000||!isfinite(lever)||lever<.1f||lever>.5f){memset(r,0,sizeof *r);r->fault="invalid plan arguments";return false;}
 const char *why;nav_map_plan_profile(100,px);
 if(!nav_map_path(nodes,&node_count,&why)){memset(r,0,sizeof *r);r->fault=why;return false;}
 return build_route(r,percent,px,py,lever);
}
static bool plan_sensed_fixed(NavRoute *r,unsigned percent,unsigned px,unsigned py,float lever){
 if(percent<25||percent>100||px<5000||px>30000||py<5000||py>30000||!isfinite(lever)||lever<.1f||lever>.5f){memset(r,0,sizeof *r);r->fault="invalid plan arguments";return false;}
 const char *why;nav_map_plan_profile(100,px);
 if(!nav_map_sensed_path(nodes,&node_count,&why)){memset(r,0,sizeof *r);r->fault=why;return false;}
 return build_route(r,percent,px,py,lever);
}
static bool replan_fixed(NavRoute *r,const NavPose *pose,unsigned px,unsigned py,float lever)
{
 if(!r||!pose||!pose->valid||!r->ready||r->index>=r->n)return false;
 Node remaining[10];unsigned n=0,station=r->station,percent=r->percent;bool manual=r->manual;
 for(unsigned k=r->index;k<r->n;k++)if(r->p[k].stop&&!(k==r->index&&r->p[k].stop==station)){
  if(n==10){r->fault="too many remaining task stops";return false;}
  remaining[n++]=(Node){r->p[k].x,r->p[k].y,r->p[k].h,r->p[k].stop};
 }
 if(!n){r->fault="no remaining stations";return false;}
 Node start={pose->x_m,pose->y_m,pose->yaw_rad,0};const char *why;
 nav_map_plan_profile(100,px);
 if(!nav_map_detour(nodes,&node_count,&start,remaining,n,&why)){r->fault=why;return false;}
 if(!build_route(r,percent,px,py,lever))return false;
 r->manual=manual;r->station=station;return true;
}
static bool finish_plan(NavRoute *r,bool ok){
 bool same=nav_lidar_plan_end();
 if(!ok){r->ready=false;return false;}
 if(!same){r->ready=false;r->fault="lidar frame/config changed during planning";return false;}
 /* No cooperative callback in this bounded pass: the latest world geometry
  * is stable until return. The expensive search above serviced live sensors
  * without changing its frozen edge costs. A newly occupied route is NEVER
  * released just because it was clear in the old planning snapshot. */
 for(unsigned k=0;k<r->n;k++)if(!nav_lidar_clear(r->p[k].x,r->p[k].y,r->p[k].h,.015f)){
  nav_logf("[PLAN_CURRENT_REJECT] sample=%u station=%u; newly occupied after frozen search; no resume\r\n",k,r->p[k].stop);
  nav_lidar_plan_blockers(r->p[k].x,r->p[k].y,r->p[k].h,.015f,true);
  r->ready=false;r->fault="lidar occupancy changed during planning; no resume";return false;
 }
 return true;
}
bool nav_route_plan(NavRoute *r,unsigned percent,unsigned px,unsigned py,float lever){
 if(!nav_lidar_plan_begin()){r->ready=false;r->fault="planner already active";return false;}
 return finish_plan(r,plan_fixed(r,percent,px,py,lever));
}
bool nav_route_plan_sensed(NavRoute *r,unsigned percent,unsigned px,unsigned py,float lever){
 if(!nav_lidar_plan_begin()){r->ready=false;r->fault="planner already active";return false;}
 return finish_plan(r,plan_sensed_fixed(r,percent,px,py,lever));
}
bool nav_route_plan_qr(NavRoute *r,unsigned percent,unsigned px,unsigned py,float lever){
 if(!nav_lidar_plan_begin()){r->ready=false;r->fault="planner already active";return false;}
 bool ok=false;const char *why="invalid QR survey plan arguments";
 if(percent>=25&&percent<=100&&px>=5000&&px<=30000&&py>=5000&&py<=30000&&isfinite(lever)&&lever>=.1f&&lever<=.5f){
  nav_map_plan_profile(100,px);
  if(nav_map_qr_path(nodes,&node_count,&why))ok=build_route(r,percent,px,py,lever);
  else {memset(r,0,sizeof *r);r->fault=why;}
 }else {memset(r,0,sizeof *r);r->fault=why;}
 return finish_plan(r,ok);
}
bool nav_route_replan(NavRoute *r,const NavPose *pose,unsigned px,unsigned py,float lever){
 if(!nav_lidar_plan_begin()){r->ready=false;r->fault="planner already active";return false;}
 return finish_plan(r,replan_fixed(r,pose,px,py,lever));
}
/* Same wheel cap for all terms. Applied-wheel back calculation prevents
 * an integrator from demanding impossible translation+rotation simultaneously. */
bool nav_route_controller(NavRoute *r,unsigned algorithm,float b0_ratio)
{
 if(!r->ready||r->time!=0||r->elapsed!=0||algorithm>3||!isfinite(b0_ratio)||b0_ratio<.5f||b0_ratio>2)return false;
 memset(&r->tracker,0,sizeof r->tracker);r->tracker.algorithm=algorithm;r->tracker.b0_ratio=b0_ratio;
 r->footprint_bad=0;r->predict_age_ms=0;r->guard_log_s=0; /* v2.7.12 guard state per run */
 return true;
}
int nav_route_step(NavRoute *r,const NavPose *p,float gyro,float dt,int wheel[4])
{
 memset(wheel,0,4*sizeof *wheel);
 if(!r->ready||r->percent<25||r->percent>100||!isfinite(r->response_s)||r->response_s<.04f||r->response_s>.50f||!isfinite(r->rpm_mps)||r->rpm_mps<=0||!isfinite(r->scale_y)||r->scale_y<=0||!isfinite(r->lever)||r->lever<.1f||r->lever>.5f||r->n<2||r->n>NAV_ROUTE_MAX||r->index>=r->n||!isfinite(r->time)||!isfinite(r->elapsed)||!isfinite(r->clock_rate)||r->clock_rate<0||r->clock_rate>1.001f||r->finished||!p->valid||!isfinite(dt)||dt<=0||dt>.15f||!isfinite(p->x_m)||!isfinite(p->y_m)||!isfinite(p->yaw_rad)||!isfinite(p->vx_mps)||!isfinite(p->vy_mps)||!isfinite(gyro)||fabsf(gyro)>4.0f){r->fault="invalid route feedback";return -1;}
 for(unsigned k=0;k<3;k++)if(!isfinite(r->integral[k])){r->fault="invalid route integrator";return -1;}
 for(unsigned k=0;k<4;k++)if(!isfinite(r->wheel[k])||fabsf(r->wheel[k])>r->max_rpm+.001f){r->fault="invalid wheel allocation state";return -1;}
 if(r->map_generation!=nav_map_generation()){r->fault="map/footprint changed during route";return -1;}
 if(r->waiting){
  const NavRoutePoint *q=&r->p[r->index];
  r->error_m=hypotf(q->x-p->x_m,q->y-p->y_m);r->yaw_error=wrap(q->h-p->yaw_rad);
  if(r->error_m>.025f||fabsf(r->yaw_error)>.034907f||!nav_route_footprint(p->x_m,p->y_m,p->yaw_rad,.005f)){r->fault="parked pose moved; no automatic resume";return -1;}
  memset(r->wheel,0,sizeof r->wheel);memset(r->wheel_accel,0,sizeof r->wheel_accel);return 0;
 }
 r->elapsed+=dt;if(r->elapsed>r->duration*2+30){r->fault="route deadline";return -1;}
 while(r->index+1<r->n&&r->p[r->index+1].t<=r->time+.00001f&&(!r->p[r->index].stop||r->station==r->p[r->index].stop))r->index++;
 unsigned a=r->index,b=a+1<r->n?a+1:a;bool atstop=r->p[a].stop!=0&&r->station!=r->p[a].stop;
 float ds=r->p[b].s-r->p[a].s,span=r->p[b].t-r->p[a].t;
 float q=span>0?fmaxf(0,fminf(1,(r->time-r->p[a].t)/span)):0;
 float accel=span>0?(r->p[b].v-r->p[a].v)/span:0,tt=q*span;
 float fraction=ds>.000001f?(r->p[a].v*tt+.5f*accel*tt*tt)/ds:0;
 if(atstop)fraction=0;
 float x=r->p[a].x+(r->p[b].x-r->p[a].x)*fraction;
 float y=r->p[a].y+(r->p[b].y-r->p[a].y)*fraction;
 float h=r->p[a].h+wrap(r->p[b].h-r->p[a].h)*fraction;
 float speed=atstop?0:r->p[a].v+accel*tt;
 float ff[3]={0};if(ds>.000001f&&!atstop){ff[0]=(r->p[b].x-r->p[a].x)/ds*speed;ff[1]=(r->p[b].y-r->p[a].y)/ds*speed;ff[2]=wrap(r->p[b].h-r->p[a].h)/ds*speed;}
 float lag_yaw=wrap(h-p->yaw_rad)*(ff[2]>=0?1.0f:-1.0f);
 float desired=fabsf(ff[2])>.05f?fmaxf(.3f,fminf(1,1-fmaxf(0,lag_yaw-.015f)/.07f)):1;
 if(r->smooth_profile){
  /* The same reference clock also waits for actuator-limited translation.
   * Otherwise a gentle wheel ramp is followed by a catch-up impulse against
   * an increasingly distant reference. Cross-track error remains controlled
   * independently and all measured-pose / tracking guards remain active. */
  float v=hypotf(ff[0],ff[1]);
  if(v>.05f){
   float lag=((x-p->x_m)*ff[0]+(y-p->y_m)*ff[1])/v;
   desired=fminf(desired,fmaxf(.35f,1-fmaxf(0,lag-.020f)/.080f));
  }
 }
 /* Statistical wheel inconsistency slows the common SE(2) reference clock.
  * It must not distort individual wheel ratios or restart the trajectory. */
 if(!isfinite(r->traction_scale)||r->traction_scale<.65f||r->traction_scale>1){r->fault="invalid traction speed scale";return -1;}
 desired=fminf(desired,r->traction_scale);
 r->clock_rate+=clip(desired-r->clock_rate,2.0f*dt);
 float feed[3];
 for(unsigned k=0;k<3;k++)feed[k]=ff[k];
 if(!atstop&&b+1<r->n&&!r->p[b].stop){
  float ds2=r->p[b+1].s-r->p[b].s;
  float d2[3]={(r->p[b+1].x-r->p[b].x)/ds2,(r->p[b+1].y-r->p[b].y)/ds2,wrap(r->p[b+1].h-r->p[b].h)/ds2};
  float d1[3]={ds>0?(r->p[b].x-r->p[a].x)/ds:0,ds>0?(r->p[b].y-r->p[a].y)/ds:0,ds>0?wrap(r->p[b].h-r->p[a].h)/ds:0};
  for(unsigned k=0;k<3;k++){
   float acceleration=d1[k]*accel+(d2[k]-d1[k])*speed*speed/fmaxf(.001f,(ds+ds2)*.5f);
   float angular_accel=5.0f;
   if(r->smooth_profile)angular_accel=fminf(5.0f,(200*r->accel_limit/.60f)*r->rpm_mps/r->lever*.65f);
   feed[k]+=r->response_s*clip(acceleration,k==2?angular_accel:.90f);
  }
 }
 for(unsigned k=0;k<3;k++){feed[k]=ff[k]*r->clock_rate+(feed[k]-ff[k])*r->clock_rate*r->clock_rate;ff[k]*=r->clock_rate;}
 float error[3]={x-p->x_m,y-p->y_m,wrap(h-p->yaw_rad)};
 r->error_m=hypotf(error[0],error[1]);r->yaw_error=error[2];
 if(!nav_route_footprint(p->x_m,p->y_m,p->yaw_rad,.000f)){
  /* v2.7.12 anti-false-stop guard. This check runs on the EXTRAPOLATED pose;
   * the planning samples already carry the real 15 mm design margin, so a
   * second 5 mm margin here allowed only ~11 mm of prediction error on the
   * narrow 400 mm corridors (field: v2.7.11 stopped at pose_age 99 ms with
   * 2 mm cross-track error). Real collision protection moves to the measured
   * pose check in nav_app; here a transient predicted excursion is tolerated
   * for up to 2 control ticks and only a sustained one stops the route. */
  unsigned bad=++r->footprint_bad;
  bool tight=r->error_m<.03f&&fabsf(error[2])<.0523599f;
  bool log_now=(bad==1)||(bad>=3)||(r->elapsed-r->guard_log_s>=.50f);
  if(log_now){r->guard_log_s=r->elapsed;
   nav_logf("[ROUTE_GUARD] pred_mm=%d,%d yaw_mdeg=%d age_ms=%lu bad=%u/3 err_mm=%d yaw_err_mdeg=%d cap=%s; predicted pose outside 0mm footprint, measured-pose guard active\r\n",
    (int)roundf(p->x_m*1000),(int)roundf(p->y_m*1000),(int)roundf(p->yaw_rad*57295.7795f),(unsigned long)r->predict_age_ms,bad,
    (int)roundf(r->error_m*1000),(int)roundf(error[2]*57295.7795f),tight?"50%":"100%");
  }
  if(bad>=3){r->fault="estimated body intersects map (sustained)";return -1;}
 }else{
  if(r->footprint_bad){r->guard_log_s=r->elapsed;
   nav_logf("[ROUTE_GUARD] pred_mm=%d,%d yaw_mdeg=%d age_ms=%lu bad=0; predicted pose back inside footprint\r\n",
    (int)roundf(p->x_m*1000),(int)roundf(p->y_m*1000),(int)roundf(p->yaw_rad*57295.7795f),(unsigned long)r->predict_age_ms);
  }
  r->footprint_bad=0;
 }
 /* Fast pose references permit transient phase lag up to10deg.
  * This is a STOP threshold, never an accuracy claim. Full oriented-body
  * collision check above is independent; station acceptance stays0.5deg. */
 float yaw_envelope=.174533f;
 if(r->error_m>.10f||fabsf(error[2])>yaw_envelope){r->fault="route tracking envelope exceeded";return -1;}
 float c=cosf(p->yaw_rad),s=sinf(p->yaw_rad);
 float measured[3]={c*p->vx_mps-s*p->vy_mps,s*p->vx_mps+c*p->vy_mps,gyro},cmd[3];
 if(r->tracker.algorithm>3){r->fault="invalid route controller";return -1;}
 bool near=atstop&&r->error_m<.006f&&fabsf(error[2])<.00872665f&&hypotf(measured[0],measured[1])<.008f&&fabsf(gyro)<.025f;
 for(unsigned k=0;k<3;k++){
  float bound=k==2?.50f:.12f;
  float correction=(k==2?yaw_gain:3.5f)*error[k]+(k==2?.50f:.25f)*(ff[k]-measured[k])+r->integral[k];
  /* Common trajectory, prediction, feedforward, caps and station settling.
   * Only moving-reference feedback changes. Algo0 retains route baseline. */
  if(r->tracker.algorithm&&!atstop){
   correction=nav_control_feedback(&r->tracker,k,-error[k],measured[k]-ff[k],bound,dt);
   if(!isfinite(correction)){r->fault="invalid route controller feedback";return -1;}
  }
  if(k==2&&r->lowyaw&&!atstop&&fabsf(ff[2])<.02f){
   /* A low-speed heading correction must not dominate a5RPM translation.
    * Soft noise bands preserve continuity, not a hard on/off relay. Angular
    * path feedforward remains independent for deliberate blended turns. */
   float blend=fmaxf(0,fminf(1,(hypotf(ff[0],ff[1])-.04f)/.10f));
   blend=blend*blend*(3-2*blend);
   float small=fmaxf(0,fabsf(error[2])-.00174533f)*(error[2]<0?-1.f:1.f);
   float rate=fmaxf(0,fabsf(gyro)-.00698132f)*(gyro<0?-1.f:1.f);
   float quiet=yaw_gain*small-.50f*rate+r->integral[2];
   correction=blend*correction+(1-blend)*quiet;
   bound=.055f+(.50f-.055f)*blend;
   float target=clip(correction,bound),alpha=dt/(.12f+dt);
   r->yaw_correction+=alpha*(target-r->yaw_correction);
   correction=r->yaw_correction;
  }else if(k==2)r->yaw_correction=correction;
  cmd[k]=near?0:atstop?clip((k==2?4.0f:2.5f)*error[k]-.20f*measured[k],bound):feed[k]+clip(correction,bound);
 }
#ifdef NAV_MOTION_SIL
 motion_debug[0]=ff[2];motion_debug[1]=hypotf(ff[0],ff[1]);
 motion_debug[2]=error[2];motion_debug[3]=cmd[2];motion_debug[4]=r->time;
#endif
 float bx=c*cmd[0]+s*cmd[1],by=(-s*cmd[0]+c*cmd[1])*r->scale_y,z=r->lever*cmd[2];
 float raw[4]={(bx-by-z)/r->rpm_mps,(bx+by+z)/r->rpm_mps,(bx+by-z)/r->rpm_mps,(bx-by+z)/r->rpm_mps};
 /* v2.7.12 optional mitigation: while a transient predicted-footprint
  * excursion is active (1-2 ticks) and tracking is otherwise tight, halve the
  * wheel command for those ticks to shrink prediction/tracking error. The
  * measured-pose emergency guard in nav_app remains the hard protection. */
 if(r->footprint_bad&&r->error_m<.03f&&fabsf(error[2])<.0523599f)for(unsigned k=0;k<4;k++)raw[k]*=.5f;
 float largest=0;for(unsigned k=0;k<4;k++)largest=fmaxf(largest,fabsf(raw[k]));
 float factor=fminf(1,r->max_rpm/fmaxf(largest,.001f)),change=0;
 for(unsigned k=0;k<4;k++){raw[k]*=factor;change=fmaxf(change,fabsf(raw[k]-r->wheel[k]));}
 float wheel_limit=240.0f*phase_accel(r,r->p[a].s+fraction*ds)/.60f;
 float slew=fminf(1,(wheel_limit*dt)/fmaxf(change,.001f));
 for(unsigned k=0;k<4;k++){
  if(r->smooth_profile==2){
   NavMotion state={r->wheel[k],r->wheel_accel[k]};
   /* Keep the gentle dock jerk ramp; a deliberate turn uses a shorter ramp
    * so independent wheel profiles do not add excessive yaw phase lag. */
   float turn=fminf(1,fabsf(ff[2])/.25f);
   nav_motion_step(&state,raw[k],wheel_limit,wheel_limit/(.18f-.13f*turn),dt);
   /* A changed request can be inside the braking distance of the profile.
    * Absolute wheel cap is hard even in that case; record boundary accel0. */
   r->wheel[k]=clip(state.rpm,r->max_rpm);
   r->wheel_accel[k]=r->wheel[k]==state.rpm?state.accel:0;
  }else{r->wheel[k]+=(raw[k]-r->wheel[k])*slew;r->wheel_accel[k]=0;}
  wheel[k]=(int)roundf(r->wheel[k]);
 }
 float applied[3];bx=(wheel[0]+wheel[1]+wheel[2]+wheel[3])*r->rpm_mps/4;
 by=(-wheel[0]+wheel[1]+wheel[2]-wheel[3])*r->rpm_mps/(4*r->scale_y);
 applied[0]=c*bx-s*by;applied[1]=s*bx+c*by;applied[2]=(-wheel[0]+wheel[1]-wheel[2]+wheel[3])*r->rpm_mps/(4*r->lever);
 for(unsigned k=0;k<3;k++)r->integral[k]=near?0:clip(r->integral[k]+dt*(.3f*error[k]+(applied[k]-cmd[k])*.7f),k==2?.08f:.035f);
 for(unsigned k=0;k<3;k++)r->tracker.applied[k]=applied[k]-feed[k];
 if(atstop)nav_control_feedback_reset(&r->tracker);
 if(atstop){
  if(near&&hypotf(p->vx_mps,p->vy_mps)<.008f&&fabsf(gyro)<.025f&&change<.5f)r->stable++;else r->stable=0;
  if(r->stable>=3){
   r->station=r->p[a].stop;r->stable=0;r->hold=0;memset(r->integral,0,sizeof r->integral);
   if(a==r->n-1){r->finished=true;return 1;}
   if(r->manual&&(r->station<=6||r->station==9)){r->waiting=true;memset(r->wheel,0,sizeof r->wheel);memset(wheel,0,4*sizeof *wheel);}
  }else if((r->hold+=dt)>8){r->fault="workstation settle deadline";return -1;}
 }else{
  /* Tracking governor advances continuously, never resets the frame or
   * waits at guide nodes. Stop nodes cannot be skipped by a 100ms tick. */
  float advance=dt*r->clock_rate;
  unsigned next=b;while(next+1<r->n&&!r->p[next].stop)next++;
  r->time=fminf(r->time+advance,r->p[next].t);
 }
 return 0;
}
