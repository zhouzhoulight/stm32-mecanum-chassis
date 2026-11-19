#include "nav/nav_lidar_policy.h"
#include <math.h>
#include <string.h>
#define TAN75 3.732050807568877f
typedef struct {uint16_t id;uint32_t at;float cutoff;bool born_near;} Echo;
static Echo history[32];
typedef struct {float x,y,bx,by,px,py,yaw,first_x,first_y,hx,hy;
 uint32_t first,last,scan,bits,far;uint16_t id;bool used,locked;} Evidence;
static Evidence evidence[32];
void nav_lidar_evidence_reset(void){memset(evidence,0,sizeof evidence);}
static unsigned count16(uint32_t bits){unsigned n=0;for(bits&=65535;bits;bits>>=1)n+=bits&1U;return n;}
static Evidence *find_evidence(uint16_t id,float x,float y,uint32_t stamp){
 Evidence *best=NULL;float distance=.075f;
 for(unsigned k=0;k<32;k++){
  Evidence *e=evidence+k;if(!e->used||stamp-e->last>1500U)continue;
  float d=hypotf(e->x-x,e->y-y);
  if(e->id==id&&d<.50f&&stamp-e->last<=300U)return e;
  if(d<distance){distance=d;best=e;}
 }return best;
}
void nav_lidar_evidence_observe(uint16_t id,float x,float y,float bx,float by,float px,float py,float yaw,bool segment,float hx,float hy,uint32_t scan,uint32_t stamp){
 Evidence *e=find_evidence(id,x,y,stamp);
 if(e&&e->scan==scan)return;
 if(!e){
  e=evidence;
  for(unsigned k=0;k<32;k++){if(!evidence[k].used){e=evidence+k;break;}if((int32_t)(evidence[k].last-e->last)<0)e=evidence+k;}
  *e=(Evidence){.x=x,.y=y,.bx=bx,.by=by,.px=px,.py=py,.yaw=yaw,.first_x=x,.first_y=y,.first=stamp,.last=stamp,.scan=scan-1U,.id=id,.used=true};
 }
 if(stamp-e->last>300U){e->locked=false;e->bx=bx;e->by=by;e->px=px;e->py=py;e->yaw=yaw;e->first_x=x;e->first_y=y;}
 if(e->locked&&hypotf(bx-e->bx,by-e->by)>.060f)e->locked=false;
 float move=hypotf(px-e->px,py-e->py),turn=fabsf(atan2f(sinf(yaw-e->yaw),cosf(yaw-e->yaw)));
 float edge=hypotf(fmaxf(0,fabsf(bx)-.15f),fmaxf(0,fabsf(by)-.15f));
 if(!segment&&edge<.25f&&hypotf(bx-e->bx,by-e->by)<.025f&&(move>.075f||turn>.35f)&&hypotf(x-e->first_x,y-e->first_y)>.060f)e->locked=true;
 unsigned gap=scan-e->scan;bool stable=hypotf(x-e->x,y-e->y)<.080f;
 float len=hypotf(hx,hy),oldlen=hypotf(e->hx,e->hy);
 if(segment&&len>.08f&&oldlen>.08f)
  stable=fabsf((hx*e->hx+hy*e->hy)/(len*oldlen))>.98480775f&&fabsf((-hy*(x-e->x)+hx*(y-e->y))/len)<.060f;
 if(stamp-e->last>300U||gap>16){e->bits=e->far=0;e->first=stamp;gap=1;}
 e->bits=((e->bits<<gap)|(stable?1U:0U))&65535U;
 e->far=((e->far<<gap)|(edge>.25f&&stable?1U:0U))&65535U;
 if(!stable&&!e->locked){e->first=stamp;e->bits=1;e->far=0;}
 e->x=x;e->y=y;e->hx=hx;e->hy=hy;e->last=stamp;e->scan=scan;e->id=id;
}
NavEchoEvidence nav_lidar_evidence(uint16_t id,float x,float y,uint32_t scan,uint32_t stamp){
 NavEchoEvidence r={0};Evidence *e=find_evidence(id,x,y,stamp);if(!e)return r;
 unsigned gap=scan-e->scan;uint32_t bits=gap<16?e->bits<<gap:0,far=gap<16?e->far<<gap:0;
 r.hits=count16(bits);r.far_hits=count16(far);r.age_ms=stamp-e->first;r.body_locked=e->locked;return r;
}
void nav_lidar_evidence_shift(float dx,float dy){for(unsigned i=0;i<32;i++)if(evidence[i].used){evidence[i].x+=dx;evidence[i].y+=dy;evidence[i].first_x+=dx;evidence[i].first_y+=dy;evidence[i].px+=dx;evidence[i].py+=dy;}}
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
