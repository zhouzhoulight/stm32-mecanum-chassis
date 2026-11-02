/* Common-time, interpolation-only drive-encoder + WT heading odometry.
 * Counts remain int64/Q10 until differenced, so long sessions retain resolution.
 * Neither estimator provides an absolute XY observation. */
#include "nav/nav_odom.h"
#include "nav/nav_platform.h"
#include "nav/nav_time.h"
#include "nav/nav_increment.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define HN 32U
#define DEG (0.0000174532925199433f)
typedef struct { uint32_t t[HN]; int64_t q[HN]; unsigned n; } History;
static NavOdom data;
static History hist[5];
static int64_t previous[5],yaw_unwrapped,yaw_origin;
static int32_t yaw_last;
static uint32_t epoch,last_yaw_at;
static bool yaw_seen;
#define CN 16U
typedef struct {uint32_t t[CN];int rpm[CN];unsigned n;} CommandHistory;
static CommandHistory commands[4];
static float previous_speed[4];static bool speed_seen;
static float command_at(unsigned k,uint32_t at,float fallback);

static const uint8_t ids[4]={4,1,3,2};
static const int signs[4]={1,-1,1,-1};
static void invalidate(uint32_t fault)
{data.fault|=fault;data.base.valid=data.fused.valid=false;}
static void push(History *h,uint32_t t,int64_t q)
{
 if(h->n && (int32_t)(t-h->t[h->n-1])<=0)return;
 if(h->n==HN){memmove(h->t,h->t+1,(HN-1)*sizeof h->t[0]);memmove(h->q,h->q+1,(HN-1)*sizeof h->q[0]);h->n--;}
 h->t[h->n]=t;h->q[h->n++]=q;
}
static bool interpolate(const History *h,uint32_t t,int64_t *q)
{
 if(!h->n || (int32_t)(t-h->t[0])<0 || (int32_t)(t-h->t[h->n-1])>0)return false;
 for(unsigned k=0;k<h->n;k++){
  if(t==h->t[k]){*q=h->q[k];return true;}
  if((int32_t)(t-h->t[k])<0){
   uint32_t span=h->t[k]-h->t[k-1];if(span>400)return false;
   *q=h->q[k-1]+(h->q[k]-h->q[k-1])*(t-h->t[k-1])/span;return true;
  }
 }
 return false;
}
static void seed_filter(void)
{
 data.base.vx_mps=data.base.vy_mps=data.fused.vx_mps=data.fused.vy_mps=0;
}
void nav_odom_init(void)
{
 memset(commands,0,sizeof commands);memset(&data,0,sizeof data);memset(hist,0,sizeof hist);yaw_seen=false;
 speed_seen=false;data.response_s=.16f;data.response_samples=0;
 yaw_unwrapped=yaw_origin=0;data.ppm_x=NAV_FORWARD_PPM;data.ppm_y=NAV_LATERAL_PPM;data.lever_m=.22f;seed_filter();
}
void nav_odom_yaw(int32_t mdeg,uint32_t at)
{
 if(!yaw_seen){yaw_unwrapped=mdeg;yaw_origin=mdeg;yaw_last=mdeg;yaw_seen=true;}
 else{
  if((int32_t)(at-last_yaw_at)<=0)return;
  int32_t delta=mdeg-yaw_last;if(delta>180000)delta-=360000;if(delta< -180000)delta+=360000;
  if(data.seeded && ((at-last_yaw_at)>500U || fabsf((float)delta)>2000+120.0f*(at-last_yaw_at))){invalidate(NAV_YAW_JUMP);return;}
  yaw_unwrapped+=delta;yaw_last=mdeg;
 }
 last_yaw_at=at;push(&hist[4],at-data.imu_delay_ms,(yaw_unwrapped-yaw_origin)*1024);
 /* Accepted raw observations are logged before alignment/filtering for replay. */
 nav_logf("I,%lu,%ld\r\n",(unsigned long)at,(long)mdeg);
}
void nav_odom_wheel(uint8_t id,int32_t count,uint32_t at)
{
 unsigned i=0;while(i<4&&ids[i]!=id)i++;if(i==4)return;
 if(hist[i].n){
  if((int32_t)(at-data.wheel_at[i])<=0)return;
  int64_t change=(int64_t)count-data.raw[i];uint32_t elapsed=at-data.wheel_at[i];
  if(data.seeded && (elapsed>800U || llabs(change)>(int64_t)256+(int64_t)NAV_MAX_WHEEL_RPM*65536*3*elapsed/(2*60000))){invalidate(NAV_ENCODER_JUMP);return;}
 }
 data.raw[i]=count;data.wheel_at[i]=at;
 push(&hist[i],at-data.encoder_delay_ms,(int64_t)count*signs[i]*1024);
}
void nav_odom_rebind(bool origin)
{
 if(origin){
  memset(&data.base,0,sizeof data.base);memset(&data.fused,0,sizeof data.fused);
  /* A previous rejected yaw jump must not bias a newly declared origin.
   * Establish heading from the NEXT fresh raw frame, not the rejected history. */
  yaw_seen=false;yaw_unwrapped=yaw_origin=0;
 }
 data.base.yaw_rad=data.fused.yaw_rad=(yaw_unwrapped-yaw_origin)*DEG;
 memset(hist,0,sizeof hist);if(yaw_seen)push(&hist[4],last_yaw_at-data.imu_delay_ms,(yaw_unwrapped-yaw_origin)*1024);
 speed_seen=false;memset(commands,0,sizeof commands);data.seeded=0;data.quiet=0;data.fault=0;data.base.valid=data.fused.valid=false;data.epoch++;seed_filter();
 data.pair_count=data.max_R=0;data.rejected_mask=data.ambiguous_mask=0;
 memset(data.wheel_mps,0,sizeof data.wheel_mps);
}
bool nav_odom_config(unsigned x,unsigned y,unsigned imu_ms,unsigned enc_ms,float lever)
{
 if(data.fault)return false; /* Configuration is not a physical re-reference. */
 if(x<5000||x>30000||y<5000||y>30000||imu_ms>150||enc_ms>50||!isfinite(lever)||lever<.1f||lever>.5f)return false;
 data.ppm_x=x;data.ppm_y=y;data.imu_delay_ms=imu_ms;data.encoder_delay_ms=enc_ms;data.lever_m=lever;nav_odom_rebind(false);return true;
}
static void se2(NavPose *p,float dx,float dy,float heading)
{
 float dh=heading-p->yaw_rad,a,b;
 if(fabsf(dh)<.001f){a=1-dh*dh/6;b=dh/2-dh*dh*dh/24;}else{a=sinf(dh)/dh;b=(1-cosf(dh))/dh;}
 float x=a*dx-b*dy,y=b*dx+a*dy,c=cosf(p->yaw_rad),s=sinf(p->yaw_rad);
 p->x_m+=c*x-s*y;p->y_m+=s*x+c*y;p->yaw_rad=heading;p->path_m+=hypotf(dx,dy);
 p->vx_mps=dx/.05f;p->vy_mps=dy/.05f;
}
void nav_odom_tick(uint32_t now)
{
 if(data.fault)return;
 for(unsigned k=0;k<5;k++)if(!hist[k].n)return;
 uint32_t latest=hist[4].t[hist[4].n-1];
 for(unsigned k=0;k<4;k++){uint32_t t=hist[k].t[hist[k].n-1];if((int32_t)(t-latest)<0)latest=t;}
 if(nav_age(now,latest)>500U){if(data.seeded)invalidate(NAV_FRAME_LOST);return;}
 if(!data.seeded){
  for(unsigned k=0;k<5;k++)if(!interpolate(&hist[k],latest,&previous[k]))return;
  epoch=latest;data.seeded=1;data.base.yaw_rad=data.fused.yaw_rad=(previous[4]/1024.0f)*DEG;
  data.base.at=data.fused.at=epoch;data.base.valid=data.fused.valid=true;return;
 }
 unsigned updates=0;
 while((int32_t)(latest-epoch)>=(int32_t)NAV_ODOM_MS && updates++<4){
  uint32_t next=epoch+NAV_ODOM_MS;int64_t current[5];
  for(unsigned k=0;k<5;k++)if(!interpolate(&hist[k],next,&current[k])){invalidate(NAV_HISTORY_GAP);return;}
  float ds[4],max_speed=0;
  for(unsigned k=0;k<4;k++){
   ds[k]=(current[k]-previous[k])/(1024.0f*20.48f*data.ppm_x);
   data.wheel_mps[k]=ds[k]/.05f;max_speed=fmaxf(max_speed,fabsf(data.wheel_mps[k]));
  }
  /* Estimate an effective first-order response from real encoder speed
   * increments and matched command history. Never modifies measured travel.
   * Midpoint times match the two 50ms average-speed observations. */
  if(speed_seen){
   float aa=0,ab=0;
   for(unsigned k=0;k<4;k++){
    float acceleration=(data.wheel_mps[k]-previous_speed[k])/.05f;
    float u=command_at(k,next-50U,NAN),v=.5f*(data.wheel_mps[k]+previous_speed[k]);
    if(isfinite(u)&&fabsf(acceleration)>.04f&&fabsf(acceleration)<2.0f&&fabsf(u-v)<.25f){aa+=acceleration*acceleration;ab+=acceleration*(u-v);}
   }
   if(aa>.01f){float estimate=ab/aa;
    if(isfinite(estimate)&&estimate>=.04f&&estimate<=.50f){
     data.response_s+=.035f*(estimate-data.response_s);data.response_s=fmaxf(.06f,fminf(.35f,data.response_s));data.response_samples++;
    }
   }
  }
  memcpy(previous_speed,data.wheel_mps,sizeof previous_speed);speed_seen=true;
  float heading=(current[4]/1024.0f)*DEG,omega=(heading-data.base.yaw_rad)/.05f;
  float dx=(ds[0]+ds[1]+ds[2]+ds[3])/4,dy=(-ds[0]+ds[1]+ds[2]-ds[3])*data.ppm_x/(4.0f*data.ppm_y);
  se2(&data.base,dx,dy,heading);
  if(max_speed<.003f && fabsf(omega)<.02f)data.quiet++;else data.quiet=0;
  float delta[2],diag[4];
  nav_increment(ds,omega*.05f,.05f,(float)data.ppm_y/data.ppm_x,data.lever_m,
                data.fused.vx_mps,data.fused.vy_mps,delta,diag);
  if(!isfinite(delta[0])||!isfinite(delta[1])){invalidate(NAV_NUMERIC_FAULT);return;}
  se2(&data.fused,delta[0],delta[1],heading);
  data.pair_count=diag[0];data.max_R=diag[1];data.rejected_mask=(uint8_t)diag[2];data.ambiguous_mask=(uint8_t)diag[3];
  epoch=next;memcpy(previous,current,sizeof previous);data.base.at=data.fused.at=epoch;data.base.updates++;data.fused.updates++;
 }
}
const NavOdom *nav_odom_data(void){return &data;}
float nav_odom_heading_now(void){return (yaw_unwrapped-yaw_origin)*DEG;}


void nav_odom_command(uint8_t id,int rpm,uint32_t at)
{
 unsigned k=0;while(k<4&&ids[k]!=id)k++;
 if(k==4||rpm<-NAV_MAX_WHEEL_RPM||rpm>NAV_MAX_WHEEL_RPM)return;
 CommandHistory *c=&commands[k];
 if(c->n&&(int32_t)(at-c->t[c->n-1])<=0)return;
 if(c->n==CN){memmove(c->t,c->t+1,(CN-1)*sizeof c->t[0]);memmove(c->rpm,c->rpm+1,(CN-1)*sizeof c->rpm[0]);c->n--;}
 c->t[c->n]=at;c->rpm[c->n++]=rpm;
}
static float command_at(unsigned k,uint32_t at,float fallback)
{
 const CommandHistory *c=&commands[k];
 for(unsigned j=c->n;j>0;j--)if((int32_t)(at-c->t[j-1])>=0)return c->rpm[j-1]*(3200.0f/data.ppm_x/60);
 return fallback;
}
static float heading_at(uint32_t at,float gyro)
{
 int64_t q;
 if(interpolate(&hist[4],at,&q))return (q/1024.0f)*DEG;
 uint32_t aligned=last_yaw_at-data.imu_delay_ms;
 return (yaw_unwrapped-yaw_origin)*DEG+gyro*nav_age(at,aligned)*.001f;
}
bool nav_odom_predict(bool fused,uint32_t now,float gyro,NavPose *out)
{
 const NavPose *p=fused?&data.fused:&data.base;
 *out=*p;uint32_t age=nav_age(now,p->at);
 if(!p->valid||age>250||!isfinite(gyro)||fabsf(gyro)>2.0f)return false;
 /* Control shadow only. ACK history supplies INPUTS to a provisional
  * first-order actuator response, never fake encoder observations. */
 float vel[4];for(unsigned k=0;k<4;k++){
  float measured=data.wheel_mps[k];
  vel[k]=measured+(command_at(k,p->at,measured)-measured)*(1-expf(-.025f/data.response_s));
 }
 uint32_t at=p->at;
 while((int32_t)(now-at)>0){
  uint32_t ms=now-at;if(ms>5)ms=5;float dt=ms*.001f,travel[4];
  for(unsigned k=0;k<4;k++){
   float u=command_at(k,at+ms/2,vel[k]),decay=expf(-dt/data.response_s);
   travel[k]=u*dt+(vel[k]-u)*data.response_s*(1-decay);vel[k]=u+(vel[k]-u)*decay;
  }
  float dx=(travel[0]+travel[1]+travel[2]+travel[3])/4;
  float dy=(-travel[0]+travel[1]+travel[2]-travel[3])*data.ppm_x/(4*data.ppm_y);
  float h=heading_at(at+ms,gyro),dh=h-out->yaw_rad,a,b;
  if(fabsf(dh)<.001f){a=1-dh*dh/6;b=dh/2-dh*dh*dh/24;}else{a=sinf(dh)/dh;b=(1-cosf(dh))/dh;}
  float x=a*dx-b*dy,y=b*dx+a*dy,c=cosf(out->yaw_rad),sn=sinf(out->yaw_rad);
  out->x_m+=c*x-sn*y;out->y_m+=sn*x+c*y;out->yaw_rad=h;at+=ms;
 }
 out->vx_mps=(vel[0]+vel[1]+vel[2]+vel[3])/4;
 out->vy_mps=(-vel[0]+vel[1]+vel[2]-vel[3])*data.ppm_x/(4*data.ppm_y);out->yaw_rad=p->yaw_rad+gyro*age*.001f;out->at=now;
 return isfinite(out->x_m)&&isfinite(out->y_m)&&isfinite(out->yaw_rad)&&isfinite(out->vx_mps)&&isfinite(out->vy_mps);
}
