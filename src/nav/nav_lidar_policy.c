#include "nav/nav_lidar_policy.h"
#include <math.h>
#include <string.h>
#define TAN75 3.732050807568877f
typedef struct {uint16_t id;uint32_t at;float cutoff;bool born_near;} Echo;
static Echo history[32];
void nav_lidar_echo_reset(void){memset(history,0,sizeof history);}
bool nav_lidar_front150(float ex,float ey,float *x,float *y,float *dx,float *dy,bool segment){
 float ax=*x-*dx-ex,ay=*y-*dy-ey,bx=*x+*dx-ex,by=*y+*dy-ey;
 if(!segment)return *x-ex>=-1e-6f&&fabsf(*y-ey)<=TAN75*(*x-ex)+1e-6f;
 /* Clip the actual centreline: a visible endpoint must not disappear merely
  * because the original long-surface centroid was outside the cone. */
 float lower=0,upper=1;
 const float nx[3]={1,TAN75,TAN75},ny[3]={0,-1,1};
 for(unsigned k=0;k<3;k++){
  float a=nx[k]*ax+ny[k]*ay,b=nx[k]*bx+ny[k]*by,d=b-a;
  if(fabsf(d)<1e-8f){if(a< -1e-6f)return false;continue;}
  float cross=-a/d;
  if(d>0)lower=fmaxf(lower,cross);else upper=fminf(upper,cross);
  if(lower>upper)return false;
 }
 float vx=bx-ax,vy=by-ay,xa=ax+lower*vx,ya=ay+lower*vy,xb=ax+upper*vx,yb=ay+upper*vy;
 *x=ex+(xa+xb)*.5f;*y=ey+(ya+yb)*.5f;*dx=(xb-xa)*.5f;*dy=(yb-ya)*.5f;return true;
}
bool nav_lidar_echo_accept(uint16_t id,float bx,float by,uint32_t stamp,bool departure,bool known){
 float x=fmaxf(0,fabsf(bx)-.15f),y=fmaxf(0,fabsf(by)-.15f);
 float distance2=x*x+y*y,cutoff=departure?.080f:.050f;
 bool close=distance2<=cutoff*cutoff;
 if(known)return true;
 Echo *slot=NULL,*oldest=&history[0];
 for(unsigned k=0;k<32;k++){
  Echo *e=&history[k];
  if(e->id==id&&id&&stamp-e->at<=1000U){slot=e;break;}
  if(!e->id||stamp-e->at>stamp-oldest->at)oldest=e;
 }
 if(!slot){slot=oldest;slot->id=id;slot->born_near=close;slot->cutoff=cutoff;}
 if(slot->born_near){float c=fmaxf(cutoff,slot->cutoff);close=distance2<=c*c;}
 slot->at=stamp;
 if(!close)slot->born_near=false;
 return !close||!slot->born_near;
}
