#include "shadow_localizer.h"
#include <math.h>
#include <string.h>
void shadow_reset(ShadowLocalizer *s){memset(s,0,sizeof *s);}
void shadow_request_anchor(ShadowLocalizer *s){shadow_reset(s);s->pending=true;}
static bool usable(const TrackSlot *s){
    return s->used&&s->matched&&s->confirmed&&s->value.kind==0&&
        s->value.extent_mm<=150&&s->value.npoints>=2&&s->value.sigma_mm<=50&&
        !(s->value.flags&(TRACK_DYNAMIC|TRACK_WEAK_SUPPORT|TRACK_FOV_EDGE));
}
void shadow_update(ShadowLocalizer *s,const FeatureTracker *t){
    if(s->pending){
        ShadowAnchor candidate[POSE_PAIR_MAX];unsigned n=0;
        for(unsigned i=0;i<TRACK_MAX && n<POSE_PAIR_MAX;i++)if(usable(&t->slots[i])){
            const LidarTrack *v=&t->slots[i].value;
            candidate[n++]=(ShadowAnchor){v->id,v->x_mm,v->y_mm,v->sigma_mm};
        }
        PosePair check[POSE_PAIR_MAX];
        for(unsigned i=0;i<n;i++)check[i]=(PosePair){candidate[i].x,candidate[i].y,candidate[i].x,candidate[i].y,candidate[i].sigma};
        PoseMatch identity;
        if(!pose_match(check,n,&identity)){s->result.valid=false;return;}
        memcpy(s->anchors,candidate,n*sizeof candidate[0]);s->count=(uint16_t)n;s->pending=false;
    }
    if(!s->count){s->result.valid=false;return;}
    PosePair p[POSE_PAIR_MAX];unsigned n=0;
    for(unsigned i=0;i<s->count;i++)for(unsigned j=0;j<TRACK_MAX;j++){
        const TrackSlot *slot=&t->slots[j];
        if(usable(slot)&&slot->value.id==s->anchors[i].id){
            const LidarTrack *v=&slot->value;
            p[n++]=(PosePair){v->x_mm,v->y_mm,s->anchors[i].x,s->anchors[i].y,
                hypotf(v->sigma_mm,s->anchors[i].sigma)};break;
        }
    }
    (void)pose_match(p,n,&s->result);
}
static void u16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
uint32_t shadow_payload(const ShadowLocalizer *s,uint32_t time,uint8_t out[26]){
    memcpy(out,&time,4);memcpy(out+4,&s->result.x_mm,4);memcpy(out+8,&s->result.y_mm,4);
    memcpy(out+12,&s->result.yaw_rad,4);
    u16(out+16,(uint16_t)fminf(65535,ceilf(s->result.sigma_mm)));
    u16(out+18,(uint16_t)fminf(65535,ceilf(s->result.sigma_yaw_rad*1000)));
    u16(out+20,s->result.inliers);u16(out+22,(uint16_t)fminf(65535,ceilf(s->result.rms_mm)));
    u16(out+24,(s->result.valid?1U:0U)|(s->pending?2U:0U)|(s->count?4U:0U)|8U);
    return 26;
}
