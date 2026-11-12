#include "pose_match.h"
#include <math.h>
#include <string.h>
static float residual(const PosePair *p,const PoseMatch *t){
    float c=cosf(t->yaw_rad),s=sinf(t->yaw_rad);
    return hypotf(c*p->sx-s*p->sy+t->x_mm-p->tx,s*p->sx+c*p->sy+t->y_mm-p->ty);
}
bool pose_match(const PosePair *p,unsigned n,PoseMatch *out){
    if(!out)return false;
    memset(out,0,sizeof *out);
    if(!p||n<3||n>POSE_PAIR_MAX)return false;
    for(unsigned i=0;i<n;i++)if(!isfinite(p[i].sx)||!isfinite(p[i].sy)||
        !isfinite(p[i].tx)||!isfinite(p[i].ty)||!isfinite(p[i].sigma_mm)||p[i].sigma_mm<1)return false;
    PoseMatch best={0};unsigned best_count=0;float best_error=1e30f;
    for(unsigned i=0;i<n;i++)for(unsigned j=i+1;j<n;j++){
        float sx=p[j].sx-p[i].sx,sy=p[j].sy-p[i].sy;
        float tx=p[j].tx-p[i].tx,ty=p[j].ty-p[i].ty;
        float sl=hypotf(sx,sy),tl=hypotf(tx,ty);
        if(sl<200||fabsf(sl-tl)>60)continue;
        PoseMatch trial={0};trial.yaw_rad=atan2f(ty,tx)-atan2f(sy,sx);
        float c=cosf(trial.yaw_rad),s=sinf(trial.yaw_rad);
        trial.x_mm=p[i].tx-c*p[i].sx+s*p[i].sy;
        trial.y_mm=p[i].ty-s*p[i].sx-c*p[i].sy;
        unsigned in=0;float sum=0;
        for(unsigned k=0;k<n;k++){
            float e=residual(&p[k],&trial);if(e<=30){in++;sum+=e*e;}
        }
        if(in>best_count||(in==best_count&&sum<best_error)){best=trial;best_count=in;best_error=sum;}
    }
    if(best_count<3)return false;
    for(unsigned iteration=0;iteration<5;iteration++){
        float sw=0,sx=0,sy=0,tx=0,ty=0,w[POSE_PAIR_MAX];
        for(unsigned i=0;i<n;i++){
            float e=residual(&p[i],&best);
            w[i]=e>40?0:(e>15?15/e:1)/(p[i].sigma_mm*p[i].sigma_mm);
            sw+=w[i];sx+=w[i]*p[i].sx;sy+=w[i]*p[i].sy;tx+=w[i]*p[i].tx;ty+=w[i]*p[i].ty;
        }
        if(sw<=0)return false;
        sx/=sw;sy/=sw;tx/=sw;ty/=sw;
        float a=0,b=0,spread=0;
        for(unsigned i=0;i<n;i++){
            float x=p[i].sx-sx,y=p[i].sy-sy;
            a+=w[i]*(x*(p[i].tx-tx)+y*(p[i].ty-ty));
            b+=w[i]*(x*(p[i].ty-ty)-y*(p[i].tx-tx));
            spread+=w[i]*(x*x+y*y);
        }
        if(spread/sw<10000 || hypotf(a,b)<1e-6f)return false;
        best.yaw_rad=atan2f(b,a);
        float c=cosf(best.yaw_rad),s=sinf(best.yaw_rad);
        best.x_mm=tx-c*sx+s*sy;best.y_mm=ty-s*sx-c*sy;
    }
    float sum=0,spread=0,sumx=0,sumy=0;unsigned in=0;
    for(unsigned i=0;i<n;i++){
        float e=residual(&p[i],&best);if(e<=30){in++;sum+=e*e;spread+=p[i].sx*p[i].sx+p[i].sy*p[i].sy;sumx+=p[i].sx;sumy+=p[i].sy;}
    }
    if(in<3 || in*2<n)return false;
    spread=fmaxf(0,spread-(sumx*sumx+sumy*sumy)/in);
    best.inliers=(uint16_t)in;best.rms_mm=sqrtf(sum/in);
    best.sigma_mm=fmaxf(10,best.rms_mm);
    best.sigma_yaw_rad=fmaxf(0.005f,best.sigma_mm/sqrtf(fmaxf(spread,1)));
    best.valid=true;*out=best;return true;
}
