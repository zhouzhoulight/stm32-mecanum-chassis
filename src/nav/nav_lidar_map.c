/* Static compact-obstacle memory in the current navigation frame, RAM only.
 * No age-based deletion: occlusion is not free-space evidence. Frame changes
 * must clear this map. Odometry drift also affects these remembered positions. */
#include "nav/nav_lidar_map.h"
#include <math.h>
#include <string.h>
static NavLidarMemory entries[NAV_LIDAR_MAP_MAX];static unsigned n;static bool overflow;
typedef struct {float x,y,r,seed_x,seed_y;uint32_t first,last,scan;uint16_t id,hits;bool used;} Candidate;
static Candidate candidates[NAV_LIDAR_MAP_MAX];
static uint16_t identities[NAV_LIDAR_MAP_MAX];
static uint32_t scans[NAV_LIDAR_MAP_MAX];
static bool frozen;static unsigned outliers;
void nav_lidar_map_clear(void){memset(entries,0,sizeof entries);memset(candidates,0,sizeof candidates);memset(identities,0,sizeof identities);memset(scans,0,sizeof scans);n=0;overflow=frozen=false;outliers=0;}
void nav_lidar_map_freeze(bool value){frozen=value;memset(candidates,0,sizeof candidates);}
bool nav_lidar_map_frozen(void){return frozen;}
unsigned nav_lidar_map_outliers(void){return outliers;}
unsigned nav_lidar_map_candidates(void){unsigned count=0;for(unsigned i=0;i<NAV_LIDAR_MAP_MAX;i++)if(candidates[i].used)count++;return count;}
unsigned nav_lidar_map_count(void){return n;}
bool nav_lidar_map_full(void){return overflow;}
const NavLidarMemory *nav_lidar_map_items(void){return entries;}
void nav_lidar_map_observe(float x,float y,float r,uint32_t now){
 if(frozen)return;
 if(!isfinite(x)||!isfinite(y)||!isfinite(r)||r<=0||r>.25f||x<-.15f||x>2.25f||y<-.15f||y>2.25f)return;
 int best=-1;float distance=.10f;
 for(unsigned i=0;i<n;i++){float d=hypotf(entries[i].x-x,entries[i].y-y);if(d<distance){distance=d;best=(int)i;}}
 if(best<0){if(n==NAV_LIDAR_MAP_MAX){overflow=true;return;}entries[n++]=(NavLidarMemory){x,y,r,now,1};return;}
 NavLidarMemory *p=&entries[best];p->seen=now;p->hits++;
 /* Enclose both disks; don't shrink/erase a previous observation merely
  * because a different view exposes a smaller visible surface. */
 if(p->r>=distance+r)return;
 if(r>=distance+p->r){p->x=x;p->y=y;p->r=r;return;}
 if(distance>1e-6f){float radius=(distance+p->r+r)*.5f,q=(radius-p->r)/distance;
  p->x+=(x-p->x)*q;p->y+=(y-p->y)*q;p->r=radius;}
}
/* Optical observations are uncertain centres, not additional physical disks
 * which must be unioned forever. Confirm a stable track across >=6 distinct
 * scans and >=450ms; a gap >500ms or a >60mm displacement restarts evidence.
 * Confirmed unseen objects remain occupied; occlusion never clears a map. */
void nav_lidar_map_track(uint16_t id,float x,float y,float r,uint32_t scan,uint32_t now){
 if(frozen||!id||!isfinite(x)||!isfinite(y)||!isfinite(r)||r<=0||r>.15f||x<-.15f||x>2.25f||y<-.15f||y>2.25f)return;
 for(unsigned i=0;i<n;i++)if(identities[i]==id){
  if(scans[i]==scan)return;
  scans[i]=scan;float dx=x-entries[i].x,dy=y-entries[i].y;
  if(dx*dx+dy*dy>.08f*.08f){outliers++;return;}
  entries[i].x+=.15f*dx;entries[i].y+=.15f*dy;
  entries[i].r=fmaxf(.05f,fmaxf(r,entries[i].r*.98f));
  entries[i].seen=now;entries[i].hits++;return;
 }
 Candidate *slot=NULL,*oldest=NULL;
 for(unsigned i=0;i<NAV_LIDAR_MAP_MAX;i++){
  Candidate *c=&candidates[i];
  if(c->used&&now-c->last>500)c->used=false;
  if(c->used&&c->id==id)slot=c;
  if(!c->used&&!oldest)oldest=c;
 }
 if(!slot){
  if(!oldest){oldest=&candidates[0];for(unsigned i=1;i<NAV_LIDAR_MAP_MAX;i++)if((int32_t)(candidates[i].last-oldest->last)<0)oldest=&candidates[i];}
  slot=oldest;*slot=(Candidate){.x=x,.y=y,.r=r,.seed_x=x,.seed_y=y,.first=now,.last=now,.scan=scan,.id=id,.hits=1,.used=true};return;
 }
 if(slot->scan==scan)return;
 float dx=x-slot->seed_x,dy=y-slot->seed_y;
 if(dx*dx+dy*dy>.06f*.06f){
  outliers++;*slot=(Candidate){.x=x,.y=y,.r=r,.seed_x=x,.seed_y=y,.first=now,.last=now,.scan=scan,.id=id,.hits=1,.used=true};return;
 }
 slot->scan=scan;slot->last=now;if(slot->hits<65535)slot->hits++;
 float gain=1.0f/slot->hits;slot->x+=(x-slot->x)*gain;slot->y+=(y-slot->y)*gain;slot->r=fmaxf(slot->r,r);
 if(slot->hits<6||now-slot->first<450)return;
 int match=-1;float best=.075f*.075f;
 for(unsigned i=0;i<n;i++){float mx=entries[i].x-slot->x,my=entries[i].y-slot->y,d=mx*mx+my*my;if(d<best&&sqrtf(d)+slot->r<=.15f){best=d;match=(int)i;}}
 if(match>=0){
  NavLidarMemory *p=&entries[match];uint32_t age=now-p->seen;p->r=fminf(.15f,fmaxf(p->r,slot->r+sqrtf(best)));p->seen=now;p->hits+=slot->hits;
  /* Preserve the original identity while it is observed; an old identity may
   * be replaced after a tracker-ID handover, without creating another disk. */
  if(age>150){identities[match]=id;scans[match]=scan;}
 }else{
  if(n==NAV_LIDAR_MAP_MAX){overflow=true;slot->used=false;return;}
  identities[n]=id;scans[n]=scan;entries[n++]=(NavLidarMemory){slot->x,slot->y,fmaxf(.05f,slot->r),now,slot->hits};
 }
 slot->used=false;
}
bool nav_lidar_map_view_safe(const NavLidarMemory *view,unsigned count,bool full,float x,float y,float yaw,float margin){
 if(full)return false;
 float c=cosf(yaw),s=sinf(yaw);
 for(unsigned i=0;i<count;i++){
  float dx=view[i].x-x,dy=view[i].y-y;
  float bx=c*dx+s*dy,by=-s*dx+c*dy;
  float qx=fmaxf(fabsf(bx)-.15f-margin,0),qy=fmaxf(fabsf(by)-.15f-margin,0);
  float radius=view[i].r+.03f; /* provisional memory uncertainty margin */
  if(qx*qx+qy*qy<radius*radius)return false;
 }return true;
}
bool nav_lidar_map_safe(float x,float y,float yaw,float margin){
 return nav_lidar_map_view_safe(entries,n,overflow,x,y,yaw,margin);
}
