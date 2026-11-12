/* Displacement-preserving robust mecanum odometry. No velocity low-pass
 * lies on the position integration path. Normal equal-weight solve is the
 * encoder increment solution; common-mode slip remains unobservable. */
#include "nav/nav_increment.h"
#include <math.h>
bool nav_increment_angular_model(const float ds[4],float dtheta,float dt,float lever)
{
 /* e03+e12 is the wheel-only left-null residual and cancels BOTH
  * translation and rotation, independently of IMU delay/effective lever.
  * Opposite gyro-pair innovations with a small left-null residual cannot
  * identify a bad wheel; retain equal-weight XY in that observable case.
  * Paired/common slip remains unobservable without external XY sensing. */
 float a=ds[0]-ds[3]+2*lever*dtheta,b=ds[1]-ds[2]-2*lever*dtheta;
 return fabsf(dtheta)>.001f&&a*b<0&&fabsf(a+b)<=.03f*dt&&
        (fabsf(a)>.03f*dt||fabsf(b)>.03f*dt);
}
void nav_increment(const float ds[4],float dtheta,float dt,float ys,float lever,
                   float previous_vx,float previous_vy,float out[2],float diag[4])
{
 if(nav_increment_angular_model(ds,dtheta,dt,lever)){
  out[0]=(ds[0]+ds[1]+ds[2]+ds[3])*.25f;
  out[1]=(-ds[0]+ds[1]+ds[2]-ds[3])/(4*ys);
  diag[0]=2;diag[1]=1;diag[2]=diag[3]=0;return;
 }
 const float side[4]={-1,1,1,-1},spin[4]={-1,1,-1,1};
 float z[4],w[4]={1,1,1,1};unsigned rejected=0,ambiguous=0,pairs=0;
 for(unsigned k=0;k<4;k++)z[k]=ds[k]-spin[k]*lever*dtheta;
 const unsigned pair[2][2]={{0,3},{1,2}};
 for(unsigned p=0;p<2;p++){
  unsigned a=pair[p][0],b=pair[p][1];
  if(fabsf(z[a]-z[b])>.03f*dt){
   pairs++;float predicted=(previous_vx+side[a]*ys*previous_vy)*dt;
   float ea=fabsf(z[a]-predicted),eb=fabsf(z[b]-predicted);
   if(fabsf(ea-eb)>.015f*dt){unsigned bad=ea>eb?a:b;w[bad]=.0001f;rejected|=1U<<bad;}
   else{w[a]=w[b]=.04f;ambiguous|=1U<<p;}
  }
 }
 float a=0,b=0,c=0,u=0,v=0,minw=1;
 for(unsigned k=0;k<4;k++){float h=side[k]*ys;a+=w[k];b+=w[k]*h;c+=w[k]*h*h;u+=w[k]*z[k];v+=w[k]*h*z[k];minw=fminf(minw,w[k]);}
 float det=a*c-b*b;
 out[0]=(c*u-b*v)/det;out[1]=(a*v-b*u)/det;
 diag[0]=(float)pairs;diag[1]=1/minw;diag[2]=(float)rejected;diag[3]=(float)ambiguous;
}
