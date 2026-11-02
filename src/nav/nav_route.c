/* Firmware-native continuous SE(2) path, sampled speed-envelope and tracker.
 * This is NOT TOPPRA or a contact model. Only explicit task stops have v=0.
 * Static 300mm square vehicle / four 450mm obstacles; no random-obstacle sensor.
 */
#include "nav/nav_route.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846f
typedef struct {float x,y,h;unsigned stop;} Node;
/* Meter local frame: map=(2.25-x,2.25-y), yaw0=WEST. Intermediate nodes
 * deliberately put rotations in the central cross, NOT narrow straight arms. */
static const Node nodes[]={
 {0,0,0,0},{.05f,.048262f,0,0},{.125f,.120655f,0,0},{.25f,.125f,0,0},{1.05f,.125f,0,1},
 {1.05f,.90f,0,0},{1.05f,1.20f,PI,0},{1.05f,1.975f,PI,2},
 {1.05f,1.20f,PI,0},{1.05f,1.05f,PI*.75f,0},{1.20f,1.05f,PI*.5f,0},{1.975f,1.05f,PI*.5f,3},
 {1.20f,1.05f,PI*.5f,0},{1.05f,1.05f,PI*.25f,0},{1.05f,.90f,0,0},{1.05f,.125f,0,4},
 {1.05f,.90f,0,0},{1.05f,1.20f,PI,0},{1.05f,1.975f,PI,5},
 {1.05f,1.20f,PI,0},{1.05f,1.05f,PI*.75f,0},{1.20f,1.05f,PI*.5f,0},{1.975f,1.05f,PI*.5f,6},
 {1.975f,1.975f,PI*.5f,0},{.125f,1.975f,0,0},{.020f,2.080f,0,7},
 {.125f,1.975f,0,0},{.125f,.125f,0,0},{.020f,.020f,0,8}
};
const char *nav_route_station(unsigned n)
{
 static const char *names[]={"guide","raw1","rough1","left1","raw2","rough2","left2","zone2","zone1"};
 return n<=8?names[n]:"invalid";
}
static float wrap(float a){return atan2f(sinf(a),cosf(a));}
static float clip(float a,float b){return fmaxf(-b,fminf(b,a));}
/* OBB / AABB separating axes. Expanded chassis includes explicit margin.
 * The field outline uses physical size: blue docks have zero outline margin. */
bool nav_route_footprint(float lx,float ly,float yaw,float margin)
{
 if(!isfinite(lx)||!isfinite(ly)||!isfinite(yaw)||!isfinite(margin)||margin<0)return false;
 float x=2.25f-lx,y=2.25f-ly,c=cosf(yaw),s=sinf(yaw),ac=fabsf(c),as=fabsf(s);
 float physical=.15f*(ac+as);
 if(x-physical<-.0001f||x+physical>2.4001f||y-physical<-.0001f||y+physical>2.4001f)return false;
 float half=.15f+margin;
 const float centers[2]={.775f,1.625f};
 for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++){
  float dx=x-centers[i],dy=y-centers[j];
  if(fabsf(dx)>=.225f+half*(ac+as)||fabsf(dy)>=.225f+half*(ac+as))continue;
  if(fabsf(c*dx+s*dy)>=half+.225f*(ac+as)||fabsf(-s*dx+c*dy)>=half+.225f*(ac+as))continue;
  return false;
 }
 return true;
}
static void tangent(unsigned k,float t[2])
{
 unsigned n=sizeof nodes/sizeof nodes[0];t[0]=t[1]=0;
 if(!k||k==n-1||nodes[k].stop)return;
 float a[2]={nodes[k].x-nodes[k-1].x,nodes[k].y-nodes[k-1].y};
 float b[2]={nodes[k+1].x-nodes[k].x,nodes[k+1].y-nodes[k].y};
 float la=hypotf(a[0],a[1]),lb=hypotf(b[0],b[1]);
 float x=a[0]/la+b[0]/lb,y=a[1]/la+b[1]/lb,l=hypotf(x,y);
 if(l>.01f){float size=fminf(.30f,fminf(la,lb))*.8f;t[0]=x/l*size;t[1]=y/l*size;}
}
static void geometry(unsigned edge,float u,float out[3])
{
 const Node *a=&nodes[edge],*b=a+1;float ta[2],tb[2];tangent(edge,ta);tangent(edge+1,tb);
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
bool nav_route_plan(NavRoute *r,unsigned percent,unsigned px,unsigned py,float lever)
{
 memset(r,0,sizeof *r);r->fault="invalid plan arguments";
 if(percent<25||percent>100||px<5000||px>30000||py<5000||py>30000||!isfinite(lever)||lever<.1f||lever>.5f)return false;
 r->response_s=.16f;r->clock_rate=1;r->percent=percent;r->lever=lever;r->rpm_mps=3200.0f/px/60;r->scale_y=(float)py/px;
 r->p[0]=(NavRoutePoint){0};r->n=1;
 unsigned nc=sizeof nodes/sizeof nodes[0];
 for(unsigned e=0;e<nc-1;e++){
  float l=hypotf(nodes[e+1].x-nodes[e].x,nodes[e+1].y-nodes[e].y);
  unsigned count=(unsigned)ceilf(l/.004f); /* geometry checked at <=~6mm, even skipped storage points */
  for(unsigned j=1;j<=count;j++){
   float q[3];geometry(e,(float)j/count,q);
   if(!nav_route_footprint(q[0],q[1],q[2],.015f)){r->fault="sampled footprint intersects obstacle/outline";return false;}
   NavRoutePoint *a=&r->p[r->n-1];float ds=hypotf(q[0]-a->x,q[1]-a->y);
   if(ds<.012f&&j!=count)continue;
   if(r->n==NAV_ROUTE_MAX){r->fault="path storage exhausted";return false;}
   NavRoutePoint *p=&r->p[r->n++];p->x=q[0];p->y=q[1];p->h=q[2];p->s=a->s+ds;
   p->stop=j==count?nodes[e+1].stop:0;
   /* Doubling previous 75/100 upper bounds. Reserve 12% for feedback. */
   p->cap=(l>=.65f&&fabsf(wrap(nodes[e+1].h-nodes[e].h))<.01f?200.0f:150.0f)*percent*.01f*.88f;
   if(p->s<.10f)p->cap=fminf(p->cap,25.0f*percent*.01f); /* continuous blue-dock departure, no stop */
  }
 }
 r->p[0].cap=r->p[1].cap;r->length=r->p[r->n-1].s;
 for(unsigned k=0;k<r->n;k++){
  float d[3],w[4];derivative(r,k,d);wheel_coeff(r,k,w);
  float largest=0;for(unsigned j=0;j<4;j++)largest=fmaxf(largest,fabsf(w[j]));
  float cap=r->p[k].cap/fmaxf(largest,.001f);
  if(fabsf(d[2])>.001f)cap=fminf(cap,(1.0f*percent*.01f)/fabsf(d[2])); /* 5x old .20 rad/s */
  if(k&&k+1<r->n&&!r->p[k].stop){
   float a[3],b[3];derivative(r,k-1,a);derivative(r,k+1,b);
   float curvature=hypotf(b[0]-a[0],b[1]-a[1])/(r->p[k+1].s-r->p[k-1].s);
   cap=fminf(cap,sqrtf(.60f/fmaxf(curvature,.001f)));
  }
  r->p[k].v=(!k||r->p[k].stop)?0:cap;
 }
 /* Forward/backward reachable speed envelope. Includes wheel acceleration
  * from changing geometry/yaw, not just center acceleration. */
 for(unsigned pass=0;pass<4;pass++)for(unsigned dir=0;dir<2;dir++){
  for(unsigned j=1;j<r->n;j++){
   unsigned a=dir?r->n-j:j-1,b=dir?a-1:j;
   float ds=fabsf(r->p[b].s-r->p[a].s);
   r->p[b].v=fminf(r->p[b].v,sqrtf(r->p[a].v*r->p[a].v+2*.60f*ds));
   float wa[4],wb[4];wheel_coeff(r,a,wa);wheel_coeff(r,b,wb);
   for(unsigned it=0;it<25;it++){
    float dt=2*ds/fmaxf(.001f,r->p[a].v+r->p[b].v),ratio=1;
    for(unsigned k=0;k<4;k++)ratio=fmaxf(ratio,fabsf(wb[k]*r->p[b].v-wa[k]*r->p[a].v)/(200.0f*dt));
    if(ratio<=1.001f)break;
    /* If lowering b cannot reduce the largest delta, the reverse sweep
     * lowers a; retain finite monotone contraction rather than divide by 0. */
    r->p[b].v*=.97f;
   }
  }
 }
 for(unsigned k=1;k<r->n;k++){
  float sum=r->p[k-1].v+r->p[k].v;
  if(sum<.00001f){r->fault="unreachable zero-speed edge";return false;}
  r->p[k].t=r->p[k-1].t+2*(r->p[k].s-r->p[k-1].s)/sum;
 }
 r->duration=r->p[r->n-1].t;r->ready=true;r->fault="none";return true;
}
/* Same wheel cap for all terms. Applied-wheel back calculation prevents
 * an integrator from demanding impossible translation+rotation simultaneously. */
int nav_route_step(NavRoute *r,const NavPose *p,float gyro,float dt,int wheel[4])
{
 memset(wheel,0,4*sizeof *wheel);
 if(!r->ready||r->percent<25||r->percent>100||!isfinite(r->response_s)||r->response_s<.04f||r->response_s>.50f||!isfinite(r->rpm_mps)||r->rpm_mps<=0||!isfinite(r->scale_y)||r->scale_y<=0||!isfinite(r->lever)||r->lever<.1f||r->lever>.5f||r->n<2||r->n>NAV_ROUTE_MAX||r->index>=r->n||!isfinite(r->time)||!isfinite(r->elapsed)||!isfinite(r->clock_rate)||r->clock_rate<0||r->clock_rate>1.001f||r->finished||!p->valid||!isfinite(dt)||dt<=0||dt>.15f||!isfinite(p->x_m)||!isfinite(p->y_m)||!isfinite(p->yaw_rad)||!isfinite(p->vx_mps)||!isfinite(p->vy_mps)||!isfinite(gyro)||fabsf(gyro)>2.0f){r->fault="invalid route feedback";return -1;}
 for(unsigned k=0;k<3;k++)if(!isfinite(r->integral[k])){r->fault="invalid route integrator";return -1;}
 for(unsigned k=0;k<4;k++)if(!isfinite(r->wheel[k])||fabsf(r->wheel[k])>200.001f){r->fault="invalid wheel allocation state";return -1;}
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
 r->clock_rate+=clip(desired-r->clock_rate,2.0f*dt);
 float feed[3];
 for(unsigned k=0;k<3;k++)feed[k]=ff[k];
 if(!atstop&&b+1<r->n&&!r->p[b].stop){
  float ds2=r->p[b+1].s-r->p[b].s;
  float d2[3]={(r->p[b+1].x-r->p[b].x)/ds2,(r->p[b+1].y-r->p[b].y)/ds2,wrap(r->p[b+1].h-r->p[b].h)/ds2};
  float d1[3]={ds>0?(r->p[b].x-r->p[a].x)/ds:0,ds>0?(r->p[b].y-r->p[a].y)/ds:0,ds>0?wrap(r->p[b].h-r->p[a].h)/ds:0};
  for(unsigned k=0;k<3;k++){
   float acceleration=d1[k]*accel+(d2[k]-d1[k])*speed*speed/fmaxf(.001f,(ds+ds2)*.5f);
   feed[k]+=r->response_s*clip(acceleration,k==2?5.0f:.90f);
  }
 }
 for(unsigned k=0;k<3;k++){feed[k]=ff[k]*r->clock_rate+(feed[k]-ff[k])*r->clock_rate*r->clock_rate;ff[k]*=r->clock_rate;}
 float error[3]={x-p->x_m,y-p->y_m,wrap(h-p->yaw_rad)};
 r->error_m=hypotf(error[0],error[1]);r->yaw_error=error[2];
 if(!nav_route_footprint(p->x_m,p->y_m,p->yaw_rad,.005f)){r->fault="estimated body intersects map";return -1;}
 /* Fast pose references permit transient phase lag up to10deg.
  * This is a STOP threshold, never an accuracy claim. Full oriented-body
  * collision check above is independent; station acceptance stays0.5deg. */
 float yaw_envelope=.174533f;
 if(r->error_m>.10f||fabsf(error[2])>yaw_envelope){r->fault="route tracking envelope exceeded";return -1;}
 float c=cosf(p->yaw_rad),s=sinf(p->yaw_rad);
 float measured[3]={c*p->vx_mps-s*p->vy_mps,s*p->vx_mps+c*p->vy_mps,gyro},cmd[3];
 bool near=atstop&&r->error_m<.006f&&fabsf(error[2])<.00872665f;
 for(unsigned k=0;k<3;k++){
  float bound=k==2?.50f:.12f;
  float correction=(k==2?5.0f:3.5f)*error[k]+(k==2?.20f:.25f)*(ff[k]-measured[k])+r->integral[k];
  cmd[k]=near?0:atstop?clip((k==2?4.0f:2.5f)*error[k]-.20f*measured[k],bound):feed[k]+clip(correction,bound);
 }
 float bx=c*cmd[0]+s*cmd[1],by=(-s*cmd[0]+c*cmd[1])*r->scale_y,z=r->lever*cmd[2];
 float raw[4]={(bx-by-z)/r->rpm_mps,(bx+by+z)/r->rpm_mps,(bx+by-z)/r->rpm_mps,(bx-by+z)/r->rpm_mps};
 float largest=0;for(unsigned k=0;k<4;k++)largest=fmaxf(largest,fabsf(raw[k]));
 float factor=fminf(1,200.0f*r->percent*.01f/fmaxf(largest,.001f)),change=0;
 for(unsigned k=0;k<4;k++){raw[k]*=factor;change=fmaxf(change,fabsf(raw[k]-r->wheel[k]));}
 float slew=fminf(1,(240.0f*dt)/fmaxf(change,.001f));
 for(unsigned k=0;k<4;k++){r->wheel[k]+=(raw[k]-r->wheel[k])*slew;wheel[k]=(int)roundf(r->wheel[k]);}
 float applied[3];bx=(wheel[0]+wheel[1]+wheel[2]+wheel[3])*r->rpm_mps/4;
 by=(-wheel[0]+wheel[1]+wheel[2]-wheel[3])*r->rpm_mps/(4*r->scale_y);
 applied[0]=c*bx-s*by;applied[1]=s*bx+c*by;applied[2]=(-wheel[0]+wheel[1]-wheel[2]+wheel[3])*r->rpm_mps/(4*r->lever);
 for(unsigned k=0;k<3;k++)r->integral[k]=near?0:clip(r->integral[k]+dt*(.3f*error[k]+(applied[k]-cmd[k])*.7f),k==2?.08f:.035f);
 if(atstop){
  if(near&&hypotf(p->vx_mps,p->vy_mps)<.008f&&fabsf(gyro)<.025f&&change<.5f)r->stable++;else r->stable=0;
  if(r->stable>=3){
   r->station=r->p[a].stop;r->stable=0;r->hold=0;memset(r->integral,0,sizeof r->integral);
   if(a==r->n-1){r->finished=true;return 1;}
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
