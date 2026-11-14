#pragma once
#include <stdbool.h>
#include <math.h>

/* Exact distance of a segment to a closed, axis-aligned rectangle. All units
 * are metres. The closest pair is either an endpoint/box or corner/segment,
 * unless the segment enters the box. This avoids sampling thin obstacles. */
static inline float nav_lidar_point_box_d2(float x,float y,float a,float b){
 float dx=fmaxf(fabsf(x)-a,0),dy=fmaxf(fabsf(y)-b,0);return dx*dx+dy*dy;
}
static inline float nav_lidar_segment_box_d2(float ax,float ay,float bx,float by,float a,float b){
 float d[2]={bx-ax,by-ay},p[2]={ax,ay},extent[2]={a,b},enter=0,leave=1;
 bool hit=true;
 for(unsigned k=0;k<2;k++){
  if(fabsf(d[k])<1e-8f){if(fabsf(p[k])>extent[k])hit=false;}
  else{
   float lo=(-extent[k]-p[k])/d[k],hi=(extent[k]-p[k])/d[k];
   if(lo>hi){float v=lo;lo=hi;hi=v;}enter=fmaxf(enter,lo);leave=fminf(leave,hi);
  }
 }
 if(hit&&enter<=leave)return 0;
 float best=fminf(nav_lidar_point_box_d2(ax,ay,a,b),nav_lidar_point_box_d2(bx,by,a,b));
 float norm=d[0]*d[0]+d[1]*d[1];
 for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++){
  float x=(i?1:-1)*a,y=(j?1:-1)*b;
  float t=norm>1e-12f?fmaxf(0,fminf(1,((x-ax)*d[0]+(y-ay)*d[1])/norm)):0;
  float dx=x-ax-t*d[0],dy=y-ay-t*d[1];best=fminf(best,dx*dx+dy*dy);
 }
 return best;
}
/* Capsule centre/half-vector/radius against a rotated box. A zero half-vector
 * is the legacy disk. Tangency is occupied; no reduction in safety margins. */
static inline bool nav_lidar_capsule_box(float x,float y,float dx,float dy,float r,
 float px,float py,float c,float s,float a,float b){
 float qx=x-px,qy=y-py,bx=c*qx+s*qy,by=-s*qx+c*qy;
 float ux=c*dx+s*dy,uy=-s*dx+c*dy;
 return nav_lidar_segment_box_d2(bx-ux,by-uy,bx+ux,by+uy,a,b)<=r*r;
}
