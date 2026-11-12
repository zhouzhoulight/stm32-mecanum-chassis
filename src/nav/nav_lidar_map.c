/* Static compact-obstacle memory in the current navigation frame, RAM only.
 * No age-based deletion: occlusion is not free-space evidence. Frame changes
 * must clear this map. Odometry drift also affects these remembered positions. */
#include "nav/nav_lidar_map.h"
#include <math.h>
#include <string.h>
static NavLidarMemory entries[NAV_LIDAR_MAP_MAX];static unsigned n;static bool overflow;
void nav_lidar_map_clear(void){memset(entries,0,sizeof entries);n=0;overflow=false;}
unsigned nav_lidar_map_count(void){return n;}
bool nav_lidar_map_full(void){return overflow;}
const NavLidarMemory *nav_lidar_map_items(void){return entries;}
void nav_lidar_map_observe(float x,float y,float r,uint32_t now){
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
