/* Bounded interboard protocol, delayed XY landmark updates, live obstacles.
 * All geometric coordinates are navigation frame N, metres. No wheel-derived
 * shadow pose is accepted as an independent absolute measurement. */
#include "nav/nav_lidar.h"
#include "nav/nav_lidar_map.h"
#include "nav/nav_map.h"
#include "nav/nav_odom.h"
#include "nav/nav_platform.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define MAX 32U
typedef struct {float x,y,r;uint16_t id,flags;uint8_t kind;} Hit;
typedef struct {NavPose p;} Sample;
static struct {
 NavLidarSend send;uint8_t state,cmd,seq,len,got,xor,p[250],txseq;
 uint32_t frames,bad,reject,last_byte,last_scan,scan,time,clock_sent,offset,clock_at,session,ack_at,now;
 bool sync,acked,configured,obstacles,fuse,anchor_request,selected,self_enabled,mirror_y;
 uint16_t fov_start,fov_end;uint32_t fov_at;bool fov_seen;uint16_t fov2_start,fov2_end;bool memory_enabled;
 float ex,ey,eh,self[4];Hit near_hit[MAX];unsigned nn;Hit pending[MAX],hit[MAX],anchor[MAX];unsigned pn,n,an,batch,total;
 uint32_t geometry_at;NavPose geometry_pose;
 uint32_t clock_attempt,clock_requests,clock_good,clock_old,clock_rtt_bad,clock_work_bad,clock_send_fail,clock_jump;
 uint32_t clock_last_rtt,clock_last_work;
 uint32_t pending_stamp,scan_at;bool have_scan,pending_valid,complete;uint32_t complete_at;
 Sample history[32];unsigned hn;uint32_t accepted,rejected;float last_dx,last_dy;
} l;
/* Static storage avoids using the small MCU stack. Retain the last plan input
 * after failure: subsequent scans must not rewrite the diagnostic evidence. */
static struct {
 Hit hit[MAX];NavLidarMemory memory[NAV_LIDAR_MAP_MAX];unsigned n,m;
 uint32_t id,at,geometry_at,session;
 bool active,valid,obstacles,memory_enabled,full;
} plan;
static struct {
 Hit hit[MAX],near[MAX];NavLidarMemory memory[NAV_LIDAR_MAP_MAX];NavPose pose,scan_pose;
 float ex,ey,eh,self[4];bool mirror_y,self_enabled;
 unsigned n,nn,m;uint32_t id,at,geometry_at,scan_age,clock_age;
 bool valid,full,memory_enabled;unsigned link_flags;
} hazard;
static uint16_t u16(const uint8_t *p){return (uint16_t)(p[0]|p[1]<<8);}
static uint32_t u32(const uint8_t *p){uint32_t v;memcpy(&v,p,4);return v;}
static float fl(const uint8_t *p){float v;memcpy(&v,p,4);return v;}
static bool send(uint8_t cmd,const void *payload,unsigned n){
 uint8_t p[256];if(!l.send||n>250)return false;
 p[0]=0xAA;p[1]=0x55;p[2]=cmd;p[3]=l.txseq++;p[4]=(uint8_t)n;
 if(n)memcpy(p+5,payload,n);
 uint8_t x=0;for(unsigned i=0;i<n+5;i++)x^=p[i];p[n+5]=x;
 return l.send(p,n+6);
}
void nav_lidar_init(NavLidarSend fn){memset(&l,0,sizeof l);memset(&plan,0,sizeof plan);memset(&hazard,0,sizeof hazard);l.send=fn;l.session=1;nav_lidar_reset();}
void nav_lidar_reset(void){nav_lidar_map_clear();l.hn=l.an=l.n=l.nn=0;l.sync=l.acked=false;l.fuse=false;l.have_scan=false;l.complete=false;l.anchor_request=false;l.fov_seen=false;l.session++;
 uint8_t q[5];memcpy(q,&l.session,4);q[4]=0;(void)send(0x68,q,5);}
void nav_lidar_error(void){l.state=0;l.sync=l.acked=false;l.pending_valid=false;l.complete=false;l.bad++;}
static bool at_pose(uint32_t at,NavPose *p){
 for(unsigned i=1;i<l.hn;i++){
  const NavPose *a=&l.history[i-1].p,*b=&l.history[i].p;
  if((int32_t)(at-a->at)<0||(int32_t)(b->at-at)<0)continue;
  uint32_t dt=b->at-a->at;if(!dt||dt>150)return false;
  float q=(float)(at-a->at)/dt;*p=*a;p->at=at;
  p->x_m+=(b->x_m-a->x_m)*q;p->y_m+=(b->y_m-a->y_m)*q;p->yaw_rad+=(b->yaw_rad-a->yaw_rad)*q;
  return true;
 }return false;
}
static void laser_to_body(float x,float y,float *bx,float *by){
 if(l.mirror_y)y=-y;
 float c=cosf(l.eh),s=sinf(l.eh);
 *bx=l.ex+c*x-s*y;*by=l.ey+s*x+c*y;
}
/* A reflected sensor frame cannot be represented by one SE(2) yaw.
 * Supply motion hints in an equally reflected GLOBAL frame instead. The
 * radar's native point/velocity/GUI convention remains unchanged. */
static void motion_hint_frame(const NavPose *p,float *x,float *y,float *h){
 float c=cosf(p->yaw_rad),s=sinf(p->yaw_rad);
 *x=(p->x_m+c*l.ex-s*l.ey)*1000;
 *y=(p->y_m+s*l.ex+c*l.ey)*1000;*h=p->yaw_rad+l.eh;
 if(l.mirror_y){*y=-*y;*h=-*h;}
}
/* Scene ROI is the physical2400mm field in N, not the sensor's range window.
 * Discard only a disk wholly outside. An outside-centred object whose
 * uncertainty/extent enters the field must remain live collision evidence. */
static bool field_intersects(float x,float y,float radius){
 float dx=fmaxf(0,fmaxf(-.15f-x,x-2.25f)),dy=fmaxf(0,fmaxf(-.15f-y,y-2.25f));
 return dx*dx+dy*dy<=radius*radius;
}
static bool body_return_hazard(float bx,float by,float r,float vx,float vy){
 float horizon=.3f+hypotf(vx,vy);
 for(unsigned j=0;j<=12;j++){
  float qx=fmaxf(fabsf(bx-vx*horizon*j/12)-.18f,0),qy=fmaxf(fabsf(by-vy*horizon*j/12)-.18f,0);
  if(qx*qx+qy*qy<r*r)return true;
 }
 return false;
}
static void geometry(uint32_t stamp){
 NavPose pose;if(!at_pose(stamp,&pose)){l.reject++;return;}
 float c=cosf(pose.yaw_rad),s=sinf(pose.yaw_rad);
 l.n=l.nn=0;
 for(unsigned i=0;i<l.pn;i++){
  Hit h=l.pending[i];float bx,by;laser_to_body(h.x,h.y,&bx,&by);
  /* A centroid inside the known body footprint is unsuitable as a world
   * landmark/map object. Keep it in LIVE collision evidence until an explicit
   * measured self mask is configured; never hide it by distance alone. */
  if(fabsf(bx)<.15f&&fabsf(by)<.15f)h.flags|=0x4000U;
  /* No guessed all-near mask. Only an explicitly measured self rectangle. */
  if(l.self_enabled&&bx<l.self[0]&&bx>-l.self[1]&&by<l.self[2]&&by>-l.self[3])continue;
  h.x=pose.x_m+c*bx-s*by;h.y=pose.y_m+s*bx+c*by;
  if(!field_intersects(h.x,h.y,h.r)){
   /* A drifting global pose must NEVER suppress a body-relative collision.
    * Keep such evidence in the local braking layer, not the world map. */
   if(!body_return_hazard(bx,by,h.r,pose.vx_mps,pose.vy_mps))continue;
   h.flags|=0x9000U; /* outside world ROI, retained transient local safety */
  }
  if(h.flags&0x8000U){
   if(l.nn<MAX)l.near_hit[l.nn++]=h;
   continue; /* transient safety evidence, never localization/route occupancy */
  }
  /* Associate returns DEEP inside the modelled turntable with that fixed
   * object, regardless of a surface cluster being split into small fragments.
   * These are not fitted independent circles; duplicating/expanding them
   * blocks the taught pick stop. The fixed fixture guard is unchanged.
   * Outside-core detections (including adjacent poles) stay independent. */
  if(nav_map_fixture_core(h.x,h.y,.055f))h.flags|=0x2000U;
  l.hit[l.n++]=h;
 }
 l.last_scan=l.now;l.geometry_at=stamp;l.geometry_pose=pose;
 /* Remember confirmed compact static candidates, never lone near noise.
  * Line-like fixtures stay in live geometry, not this compact-object map. */
 for(unsigned i=0;i<l.n;i++)if(l.hit[i].kind==0&&!(l.hit[i].flags&(0x10U|0x40U|0x80U|0x2000U|0x4000U|0x8000U)))
  nav_lidar_map_observe(l.hit[i].x,l.hit[i].y,l.hit[i].r,l.now);
 if(l.anchor_request){
  if(nav_odom_data()->quiet<3)return;
  l.an=0;for(unsigned i=0;i<l.n;i++)if(l.hit[i].kind==0&&!(l.hit[i].flags&0x6090)&&l.hit[i].r<=.10f)l.anchor[l.an++]=l.hit[i];
  l.anchor_request=false;nav_logf("[LIDAR_ANCHOR] count=%u; relative N frame, not surveyed global fix\r\n",l.an);return;
 }
 if(!l.fuse||l.an<3)return;
 bool used[MAX]={0};float dx[MAX],dy[MAX],sx=0,sy=0;unsigned k=0;
 float xmin=1e6f,xmax=-1e6f,ymin=1e6f,ymax=-1e6f;
 for(unsigned i=0;i<l.n;i++){
  Hit *h=&l.hit[i];if(h->kind||h->r>.10f||(h->flags&0x6090))continue;
  int best=-1;float bd=.10f,second=.10f;
  for(unsigned j=0;j<l.an;j++){
   float d=hypotf(h->x-l.anchor[j].x,h->y-l.anchor[j].y);
   if(d<bd){second=bd;bd=d;best=(int)j;}else if(d<second)second=d;
  }
  if(best<0||used[best]||second-bd<.025f)continue;
  used[best]=true;dx[k]=l.anchor[best].x-h->x;dy[k]=l.anchor[best].y-h->y;sx+=dx[k];sy+=dy[k];k++;
  xmin=fminf(xmin,h->x);xmax=fmaxf(xmax,h->x);ymin=fminf(ymin,h->y);ymax=fmaxf(ymax,h->y);
 }
 if(k<3||hypotf(xmax-xmin,ymax-ymin)<.5f){l.rejected++;return;}
 sx/=k;sy/=k;float rms=0;for(unsigned i=0;i<k;i++)rms+=(dx[i]-sx)*(dx[i]-sx)+(dy[i]-sy)*(dy[i]-sy);
 rms=sqrtf(rms/k);
 if(rms>.015f||hypotf(sx,sy)>.08f){l.rejected++;return;}
 /* Rate limited observer, 5 mm/scan maximum. Raw base is never corrected. */
 float gain=.15f,norm=hypotf(sx,sy);if(norm*gain>.005f)gain=.005f/norm;
 l.last_dx=sx*gain;l.last_dy=sy*gain;
 if(nav_odom_lidar_shift(l.last_dx,l.last_dy)){l.accepted++;
  for(unsigned i=0;i<l.hn;i++){l.history[i].p.x_m+=l.last_dx;l.history[i].p.y_m+=l.last_dy;}
  for(unsigned i=0;i<l.n;i++){l.hit[i].x+=l.last_dx;l.hit[i].y+=l.last_dy;}
  for(unsigned i=0;i<l.nn;i++){l.near_hit[i].x+=l.last_dx;l.near_hit[i].y+=l.last_dy;}
  l.geometry_pose.x_m+=l.last_dx;l.geometry_pose.y_m+=l.last_dy;
 }
}
static void frame(void){
 if(l.cmd==0x75&&(l.len==8||l.len==12)){
  unsigned start=u16(l.p),end=u16(l.p+2);
  if(start<=36000&&end<=36000&&start!=end){l.fov_start=(uint16_t)start;l.fov_end=(uint16_t)end;l.fov_at=l.now;l.fov_seen=true;l.fov2_start=l.fov2_end=0;
   if(l.len==12){unsigned a=u16(l.p+8),b=u16(l.p+10);if(a>36000||b>36000||a==b){l.fov_seen=false;l.reject++;return;}l.fov2_start=(uint16_t)a;l.fov2_end=(uint16_t)b;}}
  else l.reject++;
  return;
 }
 if(l.cmd==0xE6&&l.len==12){
  if(u32(l.p)!=l.clock_sent){l.clock_old++;l.reject++;return;}
  uint32_t rtt=l.now-l.clock_sent;uint32_t work=u32(l.p+8)-u32(l.p+4);
  l.clock_last_rtt=rtt;l.clock_last_work=work;
  if(rtt>20){l.clock_rtt_bad++;l.reject++;return;}
  if(work>rtt*1000){l.clock_work_bad++;l.reject++;return;}
  uint32_t off=u32(l.p+4)-l.clock_sent*1000-(rtt*1000-work)/2;
  if(l.sync&&((int32_t)(off-l.offset)>20000||(int32_t)(off-l.offset)<-20000)){l.clock_jump++;nav_lidar_reset();return;}
  l.offset=off;l.sync=true;l.clock_at=l.now;l.clock_good++;return;
 }
 if(l.cmd==0xE8&&l.len==5&&u32(l.p)==l.session&&l.p[4]==1){l.acked=true;l.ack_at=l.now;return;}
 if(l.cmd!=0x73||l.len<11||!l.sync||!l.acked||!l.configured||l.now-l.clock_at>3000)return;
 unsigned batch=l.p[8],total=l.p[9],n=l.p[10];uint32_t scan=u32(l.p+4),t=u32(l.p);
 if(total<1||total>5||batch>=total||n>7||l.len!=11+n*32){l.reject++;return;}
 if(l.have_scan&&(int32_t)(scan-l.scan)<=0){l.reject++;return;}
 if(batch==0){l.complete=false;l.scan_at=scan;l.pending_stamp=t;l.pn=0;l.batch=0;l.total=total;l.pending_valid=true;}
 if(!l.pending_valid||l.scan_at!=scan||l.pending_stamp!=t||l.total!=total||l.batch!=batch){l.pending_valid=false;l.reject++;return;}
 l.batch++;
 for(unsigned i=0;i<n;i++){
  const uint8_t *p=l.p+11+i*32;uint16_t flags=u16(p+30),sigma=u16(p+20),extent=u16(p+22);
  float x=fl(p+4)*.001f,y=fl(p+8)*.001f;
  if(!isfinite(x)||!isfinite(y)||hypotf(x,y)>4.0f){l.pending_valid=false;l.reject++;return;}
  if(!(flags&1)||u16(p+26)>150||sigma>40)continue;
  bool regular=p[2]==2&&u16(p+28)>=3&&u16(p+24)>=2&&!(flags&0x40);
  /* Near evidence does not wait for 3-5 complete scans. Two neighbouring
   * points suffice on first observation; a lone point needs a repeated hit. */
  bool near=hypotf(x,y)<.60f&&(u16(p+24)>=2||u16(p+28)>=2);
  if(!regular&&!near)continue;
  if(!regular)flags|=0x8000U;
  if(l.pn<MAX)l.pending[l.pn++]=(Hit){x,y,.025f+extent*.0005f+sigma*.003f,u16(p),flags,p[3]};
 }
 if(l.batch==total){
  l.pending_valid=false;l.scan=scan;l.have_scan=true;
  int32_t age=(int32_t)(l.now*1000-(t-l.offset));
  if(age<0||age>250000){l.reject++;return;}
  l.complete_at=l.now-(uint32_t)age/1000;l.complete=true;
 }
}
void nav_lidar_feed(uint8_t b,uint32_t at){
 l.now=at;if(l.state&&at-l.last_byte>30)l.state=0;l.last_byte=at;
 switch(l.state){
 case 0:if(b==0xAA){l.state=1;l.xor=b;}break;
 case 1:if(b==0x55){l.xor^=b;l.state=2;}else l.state=b==0xAA?1:0;break;
 case 2:l.cmd=b;l.xor^=b;l.state=3;break;
 case 3:l.seq=b;l.xor^=b;l.state=4;break;
 case 4:l.len=b;l.got=0;l.xor^=b;if(b>250){l.bad++;l.state=0;}else l.state=b?5:6;break;
 case 5:l.p[l.got++]=b;l.xor^=b;if(l.got==l.len)l.state=6;break;
 case 6:l.state=0;if(b==l.xor){l.frames++;frame();}else l.bad++;break;
 default:l.state=0;break;
 }
}
void nav_lidar_tick(uint32_t now,bool selected,float gyro){
 if(selected!=l.selected){l.hn=l.an=0;l.fuse=false;l.complete=false;l.selected=selected;nav_lidar_map_clear();}
 l.now=now;const NavOdom *o=nav_odom_data();const NavPose *p=selected?&o->fused:&o->base;
 if(p->valid&&(!l.hn||p->at!=l.history[l.hn-1].p.at)){
  if(l.hn==32){memmove(l.history,l.history+1,31*sizeof l.history[0]);l.hn--;}
  l.history[l.hn++].p=*p;
 }
 if(l.complete){NavPose sample;
  if(at_pose(l.complete_at,&sample)){l.complete=false;geometry(l.complete_at);}
  else if(now-l.complete_at>250){l.complete=false;l.reject++;}
 }
 if(now-l.time<50)return;
 l.time=now;
 if(!l.acked||now-l.ack_at>2000){uint8_t q[5];memcpy(q,&l.session,4);q[4]=1;(void)send(0x68,q,5);}
 /* Keep the same20ms sample-quality and3s stale guards. If a refresh is
  * rejected/dropped, don't wait another full second for the next attempt.
  * The50ms service cadence rounds127ms to~150ms, away from a fixed1s phase
  * against the scanner. Failed TX enqueue is not a transmitted request. */
 if(now-l.clock_at>1000&&now-l.clock_attempt>127){
  l.clock_attempt=now;uint32_t previous=l.clock_sent;l.clock_sent=now;
  if(send(0x66,&now,4))l.clock_requests++;else{l.clock_sent=previous;l.clock_send_fail++;}
 }
 if(!l.sync||!l.acked||!l.configured||!p->valid||now-p->at>100)return;
 /* Motion hint is LASER pose in N at measured epoch, not an observation. */
 NavPose prediction;if(!nav_odom_predict(selected,now,gyro,&prediction))return;p=&prediction;
 uint8_t q[20];uint32_t t=p->at*1000+l.offset;
 float x,y,h;motion_hint_frame(p,&x,&y,&h);
 uint16_t sigma=50,sh=30;memcpy(q,&t,4);memcpy(q+4,&x,4);memcpy(q+8,&y,4);memcpy(q+12,&h,4);memcpy(q+16,&sigma,2);memcpy(q+18,&sh,2);(void)send(0x65,q,20);
}
static bool hit_intersects(float hx,float hy,float radius,float x,float y,float c,float s,float margin){
 float dx=hx-x,dy=hy-y,bx=c*dx+s*dy,by=-s*dx+c*dy;
 float qx=fmaxf(fabsf(bx)-.15f-margin,0),qy=fmaxf(fabsf(by)-.15f-margin,0);
 return qx*qx+qy*qy<radius*radius;
}
static bool hits_clear(const Hit *hit,unsigned n,float x,float y,float yaw,float margin){
 float c=cosf(yaw),s=sinf(yaw);
 for(unsigned i=0;i<n;i++){
  if(hit[i].flags&0x2000U)continue; /* still guarded by the fixed scene map */
  if(hit_intersects(hit[i].x,hit[i].y,hit[i].r,x,y,c,s,margin))return false;
 }return true;
}
bool nav_lidar_clear(float x,float y,float yaw,float margin){
 if(!l.obstacles)return true;
 if(l.memory_enabled&&!nav_lidar_map_safe(x,y,yaw,margin))return false;
 return hits_clear(l.hit,l.n,x,y,yaw,margin);
}
bool nav_lidar_plan_begin(void){
 if(plan.active)return false;
 plan.id++;plan.at=l.now;plan.geometry_at=l.geometry_at;plan.session=l.session;
 plan.n=l.n;plan.m=nav_lidar_map_count();plan.full=nav_lidar_map_full();
 plan.obstacles=l.obstacles;plan.memory_enabled=l.memory_enabled;
 memcpy(plan.hit,l.hit,plan.n*sizeof plan.hit[0]);
 memcpy(plan.memory,nav_lidar_map_items(),plan.m*sizeof plan.memory[0]);
 plan.active=plan.valid=true;
 nav_logf("[PLAN_INPUT] id=%lu at_ms=%lu geometry_ms=%lu live=%u memory=%u full=%u; frozen; live safety stays active\r\n",(unsigned long)plan.id,(unsigned long)plan.at,(unsigned long)plan.geometry_at,plan.n,plan.m,plan.full);
 return true;
}
bool nav_lidar_plan_end(void){
 bool same=plan.active&&plan.session==l.session&&plan.obstacles==l.obstacles&&plan.memory_enabled==l.memory_enabled;
 plan.active=false;
 nav_logf("[PLAN_INPUT_END] id=%lu session_same=%u; last input retained for lidar plan live/memory\r\n",(unsigned long)plan.id,same);
 return same;
}
bool nav_lidar_plan_clear(float x,float y,float yaw,float margin){
 if(!plan.active)return nav_lidar_clear(x,y,yaw,margin);
 if(plan.session!=l.session)return false;
 if(!plan.obstacles)return true;
 if(plan.memory_enabled&&!nav_lidar_map_view_safe(plan.memory,plan.m,plan.full,x,y,yaw,margin))return false;
 return hits_clear(plan.hit,plan.n,x,y,yaw,margin);
}
/* Bounded automatic diagnostics fit the serial log ring. Complete immutable
 * input can be requested later in two bounded, read-only command pages. */
void nav_lidar_plan_blockers(float x,float y,float yaw,float margin,bool live){
 const Hit *hit=live?l.hit:plan.hit;unsigned n=live?l.n:plan.n;
 const NavLidarMemory *mem=live?nav_lidar_map_items():plan.memory;unsigned m=live?nav_lidar_map_count():plan.m;
 bool enabled=live?l.obstacles:plan.obstacles,memory=live?l.memory_enabled:plan.memory_enabled;
 float c=cosf(yaw),s=sinf(yaw);unsigned found=0,shown=0;
 if(!live&&!plan.valid){nav_log("[PLAN_BLOCKERS] no captured plan input; no causal snapshot\r\n");return;}
 if(enabled)for(unsigned i=0;i<n+m;i++){
  bool is_mem=i>=n;unsigned j=is_mem?i-n:i;
  if(is_mem&&!memory)continue;
  if(!is_mem&&(hit[j].flags&0x2000U))continue;
  float hx=is_mem?mem[j].x:hit[j].x,hy=is_mem?mem[j].y:hit[j].y,r=is_mem?mem[j].r+.03f:hit[j].r;
  if(!hit_intersects(hx,hy,r,x,y,c,s,margin))continue;
  found++;
  if(shown++<4)nav_logf("[PLAN_BLOCKER] input=%lu view=%s source=%s index=%u id=%u xy_mm=%d,%d effective_r_mm=%d flags=%04X\r\n",(unsigned long)plan.id,live?"current":"frozen",is_mem?"memory":"scan",j,is_mem?0:hit[j].id,(int)roundf(hx*1000),(int)roundf(hy*1000),(int)roundf(r*1000),is_mem?0:hit[j].flags);
 }
 nav_logf("[PLAN_BLOCKERS] input=%lu view=%s target_mm=%d,%d count=%u; circles only; no automatic removal\r\n",(unsigned long)plan.id,live?"current":"frozen",(int)roundf(x*1000),(int)roundf(y*1000),found);
}
static void hazard_report(void){
 nav_logf("[HAZARD_INPUT] valid=%u id=%lu at_ms=%lu pose_mm=%d,%d yaw_mdeg=%d velocity_mmps=%d,%d geometry_ms=%lu scan_age=%lu clock_age=%lu live=%u near=%u memory=%u full=%u link_bits=%02X; CAPTURED at trigger, not poststop map\r\n",hazard.valid,(unsigned long)hazard.id,(unsigned long)hazard.at,(int)roundf(hazard.pose.x_m*1000),(int)roundf(hazard.pose.y_m*1000),(int)roundf(hazard.pose.yaw_rad*57295.7795f),(int)roundf(hazard.pose.vx_mps*1000),(int)roundf(hazard.pose.vy_mps*1000),(unsigned long)hazard.geometry_at,(unsigned long)hazard.scan_age,(unsigned long)hazard.clock_age,hazard.n,hazard.nn,hazard.m,hazard.full,hazard.link_flags);
 if(hazard.valid)nav_logf("[HAZARD_GEOMETRY] scan_pose_mm=%d,%d yaw_mdeg=%d mount_mm=%d,%d yaw_mdeg=%d mirror=%u self=%u self_mm=%d,%d,%d,%d; use scan pose to reconstruct native returns\r\n",(int)roundf(hazard.scan_pose.x_m*1000),(int)roundf(hazard.scan_pose.y_m*1000),(int)roundf(hazard.scan_pose.yaw_rad*57295.7795f),(int)roundf(hazard.ex*1000),(int)roundf(hazard.ey*1000),(int)roundf(hazard.eh*57295.7795f),hazard.mirror_y,hazard.self_enabled,(int)roundf(hazard.self[0]*1000),(int)roundf(hazard.self[1]*1000),(int)roundf(hazard.self[2]*1000),(int)roundf(hazard.self[3]*1000));
}
void nav_lidar_capture_hazard(uint32_t now,const NavPose *pose){
 if(!pose)return;
 hazard.id++;hazard.at=now;hazard.pose=*pose;hazard.n=l.n;hazard.nn=l.nn;hazard.m=nav_lidar_map_count();
 hazard.geometry_at=l.geometry_at;hazard.scan_age=now-l.last_scan;hazard.clock_age=now-l.clock_at;
 hazard.scan_pose=l.geometry_pose;hazard.ex=l.ex;hazard.ey=l.ey;hazard.eh=l.eh;hazard.mirror_y=l.mirror_y;hazard.self_enabled=l.self_enabled;memcpy(hazard.self,l.self,sizeof hazard.self);
 hazard.full=nav_lidar_map_full();hazard.memory_enabled=l.memory_enabled;
 hazard.link_flags=(l.configured?1U:0)|(l.sync?2U:0)|(l.acked?4U:0)|(l.obstacles?8U:0);
 memcpy(hazard.hit,l.hit,l.n*sizeof l.hit[0]);memcpy(hazard.near,l.near_hit,l.nn*sizeof l.near_hit[0]);
 memcpy(hazard.memory,nav_lidar_map_items(),hazard.m*sizeof hazard.memory[0]);hazard.valid=true;
 hazard_report();unsigned shown=0,total=0;float c=cosf(pose->yaw_rad),s=sinf(pose->yaw_rad);
 /* Identify the exact current-body blockers without a large log burst.
  * Near predictive hazards may be ahead rather than inside this rectangle;
  * their complete captured page remains available after stopping. */
 for(unsigned i=0;i<hazard.n+hazard.m;i++){
  bool memory=i>=hazard.n;unsigned k=memory?i-hazard.n:i;
  if(memory&&!hazard.memory_enabled)continue;
  if(!memory&&(hazard.hit[k].flags&0x2000U))continue;
  float x=memory?hazard.memory[k].x:hazard.hit[k].x,y=memory?hazard.memory[k].y:hazard.hit[k].y;
  float r=memory?hazard.memory[k].r+.03f:hazard.hit[k].r;
  if(!hit_intersects(x,y,r,pose->x_m,pose->y_m,c,s,.005f))continue;
  total++;if(shown++>=4)continue;
  float dx=x-pose->x_m,dy=y-pose->y_m;
  nav_logf("[HAZARD_BODY] source=%s index=%u id=%u body_mm=%d,%d radius_mm=%d flags=%04X kind=%u\r\n",memory?"memory":"scan",k,memory?0:hazard.hit[k].id,(int)roundf((c*dx+s*dy)*1000),(int)roundf((-s*dx+c*dy)*1000),(int)roundf(r*1000),memory?0:hazard.hit[k].flags,memory?0:hazard.hit[k].kind);
 }
 nav_logf("[HAZARD_BODY_COUNT] %u; full pages: lidar hazard live/near/memory\r\n",total);
}
bool nav_lidar_fresh(uint32_t now){return l.configured&&l.sync&&l.acked&&now-l.clock_at<=3000&&now-l.last_scan<=500;}
bool nav_lidar_avoid_ready(uint32_t now){return l.obstacles&&l.memory_enabled&&nav_lidar_fresh(now)&&!nav_lidar_map_full();}
bool nav_lidar_start_ready(uint32_t now,uint32_t since){
 return nav_lidar_avoid_ready(now)&&now-l.clock_at<=1000&&now-l.last_scan<=200&&(int32_t)(l.geometry_at-since)>=0;
}
bool nav_lidar_hazard(uint32_t now,const NavPose *p){
 if(!l.obstacles)return false;
 if(!l.configured||!l.sync||now-l.clock_at>3000||now-l.last_scan>500)return true;
 float bc=cosf(p->yaw_rad),bs=sinf(p->yaw_rad);
 for(unsigned i=0;i<l.nn;i++){
  float dx=l.near_hit[i].x-p->x_m,dy=l.near_hit[i].y-p->y_m;
  float bx=bc*dx+bs*dy,by=-bs*dx+bc*dy;
  /* Use velocity direction and a 300 ms reaction/braking envelope, rather
   * than stopping for every nearby object beside/behind a clear route. */
  if(body_return_hazard(bx,by,l.near_hit[i].r,p->vx_mps,p->vy_mps))return true;
 }
 float c=cosf(p->yaw_rad),s=sinf(p->yaw_rad),vx=c*p->vx_mps-s*p->vy_mps,vy=s*p->vx_mps+c*p->vy_mps;
 float speed=hypotf(vx,vy),horizon=.3f+speed/1.0f; /* provisional braking a=0.5 m/s^2 */
 for(unsigned i=0;i<=12;i++)if(!nav_lidar_clear(p->x_m+vx*horizon*i/12,p->y_m+vy*horizon*i/12,p->yaw_rad,.03f))return true;
 return false;
}
bool nav_lidar_command(const char *s,bool idle,uint32_t now){
 if(strncmp(s,"lidar",5))return false;
 if(!strcmp(s,"lidar hazard")||!strcmp(s,"lidar hazard live")||!strcmp(s,"lidar hazard near")||!strcmp(s,"lidar hazard memory")){
  hazard_report();
  if(!hazard.valid)return true;
  if(!strcmp(s,"lidar hazard live")||!strcmp(s,"lidar hazard near")){
   bool near=!strcmp(s,"lidar hazard near");const Hit *v=near?hazard.near:hazard.hit;unsigned count=near?hazard.nn:hazard.n;
   for(unsigned i=0;i<count;i++)nav_logf("[HAZARD_SCAN_ITEM] source=%s %u id=%u xy_mm=%d,%d radius_mm=%d flags=%04X kind=%u\r\n",near?"near":"scan",i,v[i].id,(int)roundf(v[i].x*1000),(int)roundf(v[i].y*1000),(int)roundf(v[i].r*1000),v[i].flags,v[i].kind);
  }
  if(!strcmp(s,"lidar hazard memory"))for(unsigned i=0;i<hazard.m;i++)nav_logf("[HAZARD_MEMORY_ITEM] %u xy_mm=%d,%d radius_mm=%d seen_ms=%lu hits=%u\r\n",i,(int)roundf(hazard.memory[i].x*1000),(int)roundf(hazard.memory[i].y*1000),(int)roundf(hazard.memory[i].r*1000),(unsigned long)hazard.memory[i].seen,hazard.memory[i].hits);
  return true;
 }
 if(!strcmp(s,"lidar plan")||!strcmp(s,"lidar plan live")||!strcmp(s,"lidar plan memory")){
  nav_logf("[PLAN_SNAPSHOT] valid=%u active=%u id=%lu at_ms=%lu geometry_ms=%lu session=%lu obs=%u memory_on=%u live=%u memory=%u full=%u; retained immutable input, not current map\r\n",plan.valid,plan.active,(unsigned long)plan.id,(unsigned long)plan.at,(unsigned long)plan.geometry_at,(unsigned long)plan.session,plan.obstacles,plan.memory_enabled,plan.n,plan.m,plan.full);
  if(plan.valid&&!strcmp(s,"lidar plan live"))for(unsigned i=0;i<plan.n;i++)nav_logf("[PLAN_SCAN_ITEM] %u id=%u xy_mm=%d,%d radius_mm=%d flags=%04X kind=%u\r\n",i,plan.hit[i].id,(int)roundf(plan.hit[i].x*1000),(int)roundf(plan.hit[i].y*1000),(int)roundf(plan.hit[i].r*1000),plan.hit[i].flags,plan.hit[i].kind);
  if(plan.valid&&!strcmp(s,"lidar plan memory"))for(unsigned i=0;i<plan.m;i++)nav_logf("[PLAN_MEMORY_ITEM] %u xy_mm=%d,%d radius_mm=%d seen_ms=%lu hits=%u\r\n",i,(int)roundf(plan.memory[i].x*1000),(int)roundf(plan.memory[i].y*1000),(int)roundf(plan.memory[i].r*1000),(unsigned long)plan.memory[i].seen,plan.memory[i].hits);
  return true;
 }
 if(!strcmp(s,"lidar status")){
  nav_logf("[LIDAR_CLOCK] tx=%lu accepted=%lu rtt_reject=%lu work_reject=%lu old=%lu enqueue_fail=%lu jump=%lu last_rtt_ms=%lu last_work_us=%lu; quality20ms stale3000ms unchanged\r\n",(unsigned long)l.clock_requests,(unsigned long)l.clock_good,(unsigned long)l.clock_rtt_bad,(unsigned long)l.clock_work_bad,(unsigned long)l.clock_old,(unsigned long)l.clock_send_fail,(unsigned long)l.clock_jump,(unsigned long)l.clock_last_rtt,(unsigned long)l.clock_last_work);
  nav_logf("[LIDAR] cfg=%u sync=%u ack=%u obs=%u fuse=%u frames=%lu bad=%lu reject=%lu scan_age=%lu tracks=%u anchors=%u accepted=%lu rejected=%lu shift_um=%d,%d; RAM only\r\n",l.configured,l.sync,l.acked,l.obstacles,l.fuse,(unsigned long)l.frames,(unsigned long)l.bad,(unsigned long)l.reject,(unsigned long)(now-l.last_scan),l.n,l.an,(unsigned long)l.accepted,(unsigned long)l.rejected,(int)(l.last_dx*1e6f),(int)(l.last_dy*1e6f));nav_logf("[LIDAR_NEAR] temporary=%u self_mask=%u; near is safety evidence, not permanent occupancy\r\n",l.nn,l.self_enabled);nav_logf("[LIDAR_TIMING] clock_age_ms=%lu geometry_at_ms=%lu last_scan_age_ms=%lu pose_samples=%u pending=%u complete=%u\r\n",(unsigned long)(now-l.clock_at),(unsigned long)l.geometry_at,(unsigned long)(now-l.last_scan),l.hn,l.pending_valid,l.complete);nav_logf("[LIDAR_MOUNT] x_mm=%d y_mm=%d yaw_mdeg=%d mirror_y=%u; RAM only\r\n",(int)roundf(l.ex*1000),(int)roundf(l.ey*1000),(int)roundf(l.eh*57295.7795f),l.mirror_y);nav_logf("[LIDAR_FOV] known=%u native_cdeg=%u..%u age_ms=%lu; outside unobserved\r\n",l.fov_seen,l.fov_start,l.fov_end,(unsigned long)(now-l.fov_at));nav_logf("[LIDAR_REAR_WINDOW] native_cdeg=%u..%u; narrow coverage only\r\n",l.fov2_start,l.fov2_end);return true;
 }
 if(!strcmp(s,"lidar tracks")){
  for(unsigned i=0;i<l.n;i++)nav_logf("[LIDAR_TRACK] id=%u x_mm=%d y_mm=%d radius_mm=%d flags=%04X kind=%u\r\n",l.hit[i].id,(int)(l.hit[i].x*1000),(int)(l.hit[i].y*1000),(int)(l.hit[i].r*1000),l.hit[i].flags,l.hit[i].kind);
  for(unsigned i=0;i<l.nn;i++)nav_logf("[LIDAR_TEMP] id=%u x_mm=%d y_mm=%d radius_mm=%d flags=%04X\r\n",l.near_hit[i].id,(int)(l.near_hit[i].x*1000),(int)(l.near_hit[i].y*1000),(int)(l.near_hit[i].r*1000),l.near_hit[i].flags);
  return true;
 }
 if(!strcmp(s,"lidar memory status")||!strcmp(s,"lidar memory list")){
  nav_logf("[LIDAR_MEMORY] enabled=%u count=%u overflow=%u; RAM current frame; occluded!=free\r\n",l.memory_enabled,nav_lidar_map_count(),nav_lidar_map_full());
  if(!strcmp(s,"lidar memory list")){const NavLidarMemory *p=nav_lidar_map_items();for(unsigned i=0;i<nav_lidar_map_count();i++)nav_logf("[LIDAR_MEMORY_ITEM] %u x_mm=%d y_mm=%d radius_mm=%d age_ms=%lu hits=%u\r\n",i,(int)(p[i].x*1000),(int)(p[i].y*1000),(int)(p[i].r*1000),(unsigned long)(now-p[i].seen),p[i].hits);}
  return true;
 }
 if(!idle){nav_log("[E] lidar configuration requires chassis idle\r\n");return true;}
 if(!strcmp(s,"lidar mirror on")||!strcmp(s,"lidar mirror off")){
  nav_lidar_reset();l.mirror_y=!strcmp(s,"lidar mirror on");
  nav_log("[LIDAR] scan handedness updated; hints use matching virtual frame; anchors cleared; no move\r\n");return true;
 }
 if(!strcmp(s,"lidar memory on")||!strcmp(s,"lidar memory off")){
  l.memory_enabled=!strcmp(s,"lidar memory on");nav_log("[LIDAR_MEMORY] planning use toggled; no move\r\n");return true;
 }
 if(!strcmp(s,"lidar memory clear")){nav_lidar_map_clear();nav_log("[LIDAR_MEMORY] cleared explicitly; no move\r\n");return true;}
 int f,b,left,right;
 if(!strcmp(s,"lidar self off")){l.self_enabled=false;nav_log("[LIDAR] explicit self mask disabled\r\n");return true;}
 if(sscanf(s,"lidar self %d %d %d %d %c",&f,&b,&left,&right,&(char){0})==4&&f>0&&f<=150&&b>0&&b<=150&&left>0&&left<=150&&right>0&&right<=150){
  l.self[0]=f*.001f;l.self[1]=b*.001f;l.self[2]=left*.001f;l.self[3]=right*.001f;l.self_enabled=true;
  nav_log("[LIDAR] self mask set; measured occupied interior only; never extend outside chassis\r\n");return true;
 }
 int x,y,h;char tail;
 if(sscanf(s,"lidar mount %d %d %d %c",&x,&y,&h,&tail)==3&&x>=-150&&x<=150&&y>=-150&&y<=150&&h>=-180000&&h<=180000){
  nav_lidar_reset();l.ex=x*.001f;l.ey=y*.001f;l.eh=h*.00001745329252f;l.configured=true;
 }else if(!strcmp(s,"lidar obstacle on")&&l.configured&&l.sync&&now-l.last_scan<500)l.obstacles=true;
 else if(!strcmp(s,"lidar obstacle off"))l.obstacles=false;
 else if(!strcmp(s,"lidar anchor")&&l.configured&&nav_odom_data()->quiet>=3)l.anchor_request=true;
 else if(!strcmp(s,"lidar fusion on")&&l.an>=3&&l.selected)l.fuse=true;
 else if(!strcmp(s,"lidar fusion off"))l.fuse=false;
 else {nav_log("[E] lidar: status | mount Xmm Ymm yaw_mdeg | mirror on/off | obstacle on/off | anchor | fusion on/off; valid mount/fresh scan/3 anchors required\r\n");return true;}
 nav_log("[LIDAR] configuration accepted; no motor command\r\n");return true;
}
