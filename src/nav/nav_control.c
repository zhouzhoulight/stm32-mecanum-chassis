/* Cascade / LADRC / LQR / active-set MPC; Ts=100ms, SI units.
 * Shared trajectory and allocator for hold-only / XY ablations.
 * No actuator access. 0.22m effective lever is provisional, explicitly reported.
 */
#include "nav/nav_control.h"
#include <math.h>
#include <string.h>
#include "nav/control_coefficients.h"
#include "qp/official_qp.h"
#define RPM_MPS (3200.0f/12924.0f/60.0f)
#define QP_ACCEPT_TOL .0002f
static float clip(float x,float b){return x>b?b:x< -b?-b:x;}
static float mx(float a,float b){return a>b?a:b;}
static float mn(float a,float b){return a<b?a:b;}
void nav_control_begin(NavControl *s,const NavPose *p,float x,float y,unsigned rpm,bool xy)
{
 memset(s,0,sizeof *s);s->goal[0]=x;s->goal[1]=y;
 float dx=x-p->x_m,dy=y-p->y_m;s->length=hypotf(dx,dy);
 if(s->length>.001f){s->direction[0]=dx/s->length;s->direction[1]=dy/s->length;}
 s->rpm=rpm;s->xy=xy;
 s->lever_m=.22f;s->scale_y=(float)NAV_LATERAL_PPM/NAV_FORWARD_PPM;s->rpm_mps=RPM_MPS;s->b0_ratio=1;
}
void nav_control_config(NavControl *s,unsigned algorithm,unsigned x,unsigned y,float lever,float ratio)
{s->algorithm=algorithm;s->rpm_mps=x?3200.0f/x/60:0;s->scale_y=x?(float)y/x:0;s->lever_m=lever;s->b0_ratio=ratio;}
static float qp_correction(NavControl *s,unsigned axis,float error,float velocity,float bound,bool *ok)
{
 float delta=axis==2?.04f:.008f;
 /* u0<=bound and u0>=previous-delta cannot both meet the original
  * violation<TOL acceptance when their gap exceeds 2*TOL. Keep a 5%
  * rounding margin, retain EXACT same PI fallback/active-set reset. */
 if(fabsf(s->applied[axis])>bound+delta+2.1f*QP_ACCEPT_TOL){
  *ok=false;s->qp_fallbacks++;s->qp_skips++;
  memset(s->active_set[axis],0,sizeof s->active_set[axis]);return 0;
 }
 float f[8],b[32],solution[8];boolean_T active[32],initial[32];int32_T status;
 for(unsigned k=0;k<8;k++)f[k]=CTRL_F[k][0]*error+CTRL_F[k][1]*velocity+CTRL_PREV[k]*s->applied[axis];
 for(unsigned k=0;k<32;k++){b[k]=k<16?bound:(axis==2?.04f:.008f);initial[k]=s->active_set[axis][k]!=0;}
 b[16]+=s->applied[axis];b[24]-=s->applied[axis];
 official_qp(f,b,initial,solution,&status,active);
 float violation=0;for(unsigned k=0;k<32;k++){float v=-b[k];for(unsigned j=0;j<8;j++)v+=CTRL_A[k][j]*solution[j];violation=mx(violation,v);s->active_set[axis][k]=(uint8_t)active[k];}
 *ok=status>0&&violation<QP_ACCEPT_TOL;
 for(unsigned k=0;k<8;k++)if(!isfinite(solution[k]))*ok=false;
 if(status>0)s->qp_steps+=(unsigned)status;
 if(!*ok){s->qp_fallbacks++;memset(s->active_set[axis],0,sizeof s->active_set[axis]);}
 return solution[0];
}
int nav_control_step(NavControl *s,const NavPose *p,float yaw,float gyro,float dt,int wheel[4])
{
 for(unsigned k=0;k<4;k++)wheel[k]=0;
 if(!isfinite(s->yaw_ff)||fabsf(s->yaw_ff)>.25f||!isfinite(s->translation_limit)||s->translation_limit<0||!p->valid||!isfinite(dt)||dt<=0||dt>.25f||!isfinite(s->length)||s->length<.001f||
    !isfinite(p->x_m)||!isfinite(p->y_m)||!isfinite(p->yaw_rad)||!isfinite(p->vx_mps)||!isfinite(p->vy_mps)||
    !isfinite(yaw)||!isfinite(gyro)||!isfinite(s->goal[0])||!isfinite(s->goal[1])||
    s->algorithm>3||s->rpm<5||s->rpm>100||!isfinite(s->rpm_mps)||s->rpm_mps<=0||
    !isfinite(s->scale_y)||s->scale_y<=0||!isfinite(s->lever_m)||s->lever_m<.1f||s->lever_m>.5f||
    !isfinite(s->b0_ratio)||s->b0_ratio<.5f||s->b0_ratio>2||!isfinite(s->v)||!isfinite(s->a))return -1;
 for(unsigned j=0;j<3;j++)if(!isfinite(s->integral[j])||!isfinite(s->applied[j])||!isfinite(s->eso[j][0])||!isfinite(s->eso[j][1]))return -1;
 float dx=s->goal[0]-p->x_m,dy=s->goal[1]-p->y_m;
 float remaining=dx*s->direction[0]+dy*s->direction[1];s->remaining=remaining;
 if(remaining< -.015f)return -2;
 float c=cosf(p->yaw_rad),sn=sinf(p->yaw_rad);
 float velocity[2]={c*p->vx_mps-sn*p->vy_mps,sn*p->vx_mps+c*p->vy_mps};
 float along=velocity[0]*s->direction[0]+velocity[1]*s->direction[1];
 float forecast=mx(s->v,mx(along,0));float effective=mx(0,remaining-.733333f*forecast-.005f);
 float accel=s->rpm>50?.10f:.06f,jerk=s->rpm>50?.30f:.18f;
 float target=mn(s->rpm*s->rpm_mps,mn(sqrtf(2*accel*effective),mx(0,2*(remaining-.005f))));
 if(s->translation_limit>0)target=mn(target,s->translation_limit);
 float ta=clip((target-s->v)/.2f,accel);s->a+=clip(ta-s->a,jerk*dt);s->a=clip(s->a,accel);
 s->v=mn(mx(0,s->v+s->a*dt),s->rpm*s->rpm_mps);
 if(remaining<=.0075f&&s->v<.004f){s->v=0;s->a=0;}
 float cross[2]={-dx+remaining*s->direction[0],-dy+remaining*s->direction[1]};
 s->cross_error=hypotf(cross[0],cross[1]);
 if(fabsf(remaining)<=.008f&&(!s->xy||s->cross_error<.008f)&&s->v<.004f&&fabsf(yaw)<.00872665f&&fabsf(s->yaw_ff)<.00001f){
  memset(s->wheel,0,sizeof s->wheel);memset(s->integral,0,sizeof s->integral);memset(s->applied,0,sizeof s->applied);
  if(hypotf(p->vx_mps,p->vy_mps)<.007f&&fabsf(gyro)<.0174533f)s->stable++;else s->stable=0;
  return s->stable>=3?1:0;
 }
 float ff[3]={s->v*s->direction[0],s->v*s->direction[1],s->yaw_ff};
 float error[3]={s->xy?cross[0]:0,s->xy?cross[1]:0,yaw};
 float vel[3]={velocity[0]-ff[0],velocity[1]-ff[1],gyro-ff[2]},cmd[3];
 for(unsigned j=0;j<3;j++){
  float bound=j==2?.15f:.035f,desired=-1.8f*error[j],er=desired-vel[j];
  if(fabsf(s->applied[j])<.95f*bound)s->integral[j]=clip(s->integral[j]+.5f*er*dt,j==2?.04f:.015f);
  float corr=desired+.6f*er+s->integral[j];
  if(s->algorithm==1){
   float b0=6.25f*s->b0_ratio;
   for(unsigned k=0;k<10;k++){float e=s->eso[j][0]-vel[j];s->eso[j][0]+=(dt/10)*(b0*s->applied[j]+s->eso[j][1]-8*e);s->eso[j][1]=clip(s->eso[j][1]-(dt/10)*16*e,j==2?.5f:.2f);}
   corr=(3*(desired-s->eso[j][0])-s->eso[j][1])/b0;
  }else if(s->algorithm==2)corr=-CTRL_K[0]*error[j]-CTRL_K[1]*vel[j];
  else if(s->algorithm==3){bool ok;float q=qp_correction(s,j,error[j],vel[j],bound,&ok);if(ok)corr=q;}
  /* Hold-only has no cross-track correction, but retains along-speed feedback. */
  if(!s->xy&&j<2){float projected=vel[0]*s->direction[0]+vel[1]*s->direction[1];corr=-.6f*projected*s->direction[j];}
  cmd[j]=ff[j]+clip(corr,bound);
 }
 float x=c*cmd[0]+sn*cmd[1],y=-sn*cmd[0]+c*cmd[1],r=s->lever_m*cmd[2];
 float raw[4]={(x-s->scale_y*y-r)/s->rpm_mps,(x+s->scale_y*y+r)/s->rpm_mps,
               (x+s->scale_y*y-r)/s->rpm_mps,(x-s->scale_y*y+r)/s->rpm_mps};
 float magnitude=0;for(unsigned i=0;i<4;i++)magnitude=mx(magnitude,fabsf(raw[i]));
 /* Declared RPM is an actual wheel cap, not merely a nominal cruise value. */
 float scale=mn(1,(float)s->rpm/mx(.001f,magnitude));
 float change=0;for(unsigned i=0;i<4;i++){raw[i]*=scale;change=mx(change,fabsf(raw[i]-s->wheel[i]));}
 scale=mn(1,((s->rpm>50?30:20)*dt)/mx(.001f,change));
 for(unsigned i=0;i<4;i++){
  s->wheel[i]+=scale*(raw[i]-s->wheel[i]);
  if(!isfinite(s->wheel[i])||fabsf(s->wheel[i])>s->rpm+.001f){memset(wheel,0,4*sizeof *wheel);return -1;}
  wheel[i]=(int)roundf(s->wheel[i]);
 }
 float bx=(wheel[0]+wheel[1]+wheel[2]+wheel[3])*s->rpm_mps/4;
 float by=(-wheel[0]+wheel[1]+wheel[2]-wheel[3])*s->rpm_mps/(4*s->scale_y);
 s->applied[0]=c*bx-sn*by-ff[0];s->applied[1]=sn*bx+c*by-ff[1];
 s->applied[2]=(-wheel[0]+wheel[1]-wheel[2]+wheel[3])*s->rpm_mps/(4*s->lever_m)-ff[2];
 bool position=fabsf(remaining)<=.008f&&(!s->xy||s->cross_error<.008f);
 bool slow=hypotf(p->vx_mps,p->vy_mps)<.007f&&s->v<.004f&&fabsf(yaw)<.00872665f&&fabsf(gyro)<.0174533f&&fabsf(s->yaw_ff)<.00001f;
 if(position&&slow)s->stable++;else s->stable=0;
 return s->stable>=3?1:0;
}
