#include "nav/nav_lidar_observer.h"
#include <math.h>
#include <string.h>
float nav_lidar_feature_cost(const NavLidarFeature *a,const NavLidarFeature *b){
 bool sa=(a->flags&NAV_LIDAR_SEGMENT)!=0,sb=(b->flags&NAV_LIDAR_SEGMENT)!=0;
 if(sa!=sb)return 1e6f;
 float x=a->x-b->x,y=a->y-b->y;
 if(!sa)return hypotf(x,y);
 float la=hypotf(a->dx,a->dy),lb=hypotf(b->dx,b->dy);
 if(la<.08f||lb<.08f)return 1e6f;
 float dot=(a->dx*b->dx+a->dy*b->dy)/(la*lb);
 if(fabsf(dot)<.98480775f)return 1e6f; /* ten degrees, direction sign irrelevant */
 float along=fabsf((x*a->dx+y*a->dy)/la);
 if(along>la+lb+.10f)return 1e6f;
 return fabsf((-a->dy*x+a->dx*y)/la);
}
static float sigma(const NavLidarFeature *p){return fmaxf(.010f,fminf(.040f,(p->r-.025f)/3));}
bool nav_lidar_observer_fit(const NavLidarFeature *a,unsigned na,const NavLidarFeature *o,unsigned no,NavLidarFit *f){
 memset(f,0,sizeof *f);if(na>32||no>32)return false;
 int match[32];bool used[32]={0};unsigned m=0;float span=0;
 for(unsigned i=0;i<no;i++){
  match[i]=-1;float best=.12f,second=.12f;int pick=-1;
  if(!isfinite(o[i].x)||!isfinite(o[i].y)||!isfinite(o[i].r))continue;
  for(unsigned j=0;j<na;j++)if(!used[j]){
   float d=nav_lidar_feature_cost(a+j,o+i);
   if(d<best){second=best;best=d;pick=(int)j;}else if(d<second)second=d;
  }
  if(pick<0||second-best<.020f)continue;
  /* Two fragments of one surface must not pretend to be two landmarks. */
  bool independent=true;
  for(unsigned k=0;k<i;k++)if(match[k]>=0){
   const NavLidarFeature *p=a+match[k];float d=hypotf(p->x-a[pick].x,p->y-a[pick].y);
   if(d<.18f)independent=false;
   span=fmaxf(span,d);
  }
  if(!independent)continue;
  used[pick]=true;match[i]=pick;m++;
  if(o[i].flags&NAV_LIDAR_SEGMENT)f->lines++;else f->points++;
 }
 f->matches=m;if(m<2||span<.30f)return false;
 float dx=0,dy=0,A=0,B=0,C=0;
 /* Three tiny2x2 IRLS passes. No heap, generated double math, or QP. */
 for(unsigned pass=0;pass<3;pass++){
  float bx=0,by=0;A=B=C=0;
  for(unsigned i=0;i<no;i++)if(match[i]>=0){
   const NavLidarFeature *p=a+match[i];float ex=p->x-o[i].x,ey=p->y-o[i].y;
   float s=hypotf(sigma(p),sigma(o+i)),w=1/(s*s);
   if(o[i].flags&NAV_LIDAR_SEGMENT){
    float len=hypotf(p->dx,p->dy),nx=-p->dy/len,ny=p->dx/len;
    float z=nx*ex+ny*ey,res=fabsf(z-nx*dx-ny*dy);
    w*=fminf(1,2*s/fmaxf(res,1e-6f));
    A+=w*nx*nx;B+=w*nx*ny;C+=w*ny*ny;bx+=w*nx*z;by+=w*ny*z;
   }else{
    float res=hypotf(ex-dx,ey-dy);w*=fminf(1,2*s/fmaxf(res,1e-6f));
    A+=w;C+=w;bx+=w*ex;by+=w*ey;
   }
  }
  float tr=A+C,disc=sqrtf(fmaxf(0,(A-C)*(A-C)+4*B*B));
  f->rank=(tr-disc)/fmaxf(1e-9f,tr+disc);
  float det=A*C-B*B;if(f->rank<.08f||det<1e-3f)return false;
  dx=(C*bx-B*by)/det;dy=(A*by-B*bx)/det;
 }
 float sum=0;unsigned equations=0;
 for(unsigned i=0;i<no;i++)if(match[i]>=0){
  const NavLidarFeature *p=a+match[i];float ex=p->x-o[i].x-dx,ey=p->y-o[i].y-dy;
  float s=hypotf(sigma(p),sigma(o+i)),r;
  if(o[i].flags&NAV_LIDAR_SEGMENT){float len=hypotf(p->dx,p->dy);r=fabsf((-p->dy*ex+p->dx*ey)/len);sum+=r*r;equations++;}
  else {r=hypotf(ex,ey);sum+=r*r;equations+=2;}
  if(r>fminf(.050f,3*s+.005f))return false;
 }
 f->dx=dx;f->dy=dy;f->rms=sqrtf(sum/fmaxf(1,(float)equations));
 return isfinite(dx)&&isfinite(dy)&&hypotf(dx,dy)<=.080f;
}
