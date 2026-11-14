/* Bounded geometry and temporal tracks. A point cluster is NOT a black-pole
 * classification. Raw scans stay untouched; no motion input means no deskew. */
#include "features.h"
#include <math.h>
#include <string.h>
#define CANDIDATE_MAX 48U
typedef struct {
    float x,y,range,sigma;
    float ax,ay,bx,by;
    uint16_t halfwidth;
    uint16_t extent,n;
    uint8_t kind,intensity;
    bool line,edge;
} Candidate;
void features_init(FeatureTracker *t) { memset(t,0,sizeof *t); t->next_id=1;t->fov_span_cdeg=36000; }
void features_init_installed(FeatureTracker *t){
 (void)features_set_fov(t,INSTALLED_FOV_START_CDEG,INSTALLED_FOV_END_CDEG);
}
/* FOV applies in native raw angle BEFORE deskew or clustering. End 36000
 * includes raw zero; outside is unobserved, not a free-space observation. */
bool features_set_fov(FeatureTracker *t,unsigned start,unsigned end){
 if(!t||start>36000||end>36000||start==end)return false;
 unsigned span=end>=start?end-start:36000-start+end;
 if(!span)return false;
 features_init(t);t->fov_start_cdeg=(uint16_t)(start%36000);t->fov_span_cdeg=(uint16_t)span;return true;
}
bool features_add_fov(FeatureTracker *t,unsigned start,unsigned end){
 if(!t||start>36000||end>36000||start==end)return false;
 unsigned span=end>=start?end-start:36000-start+end;if(!span)return false;
 t->fov2_start_cdeg=(uint16_t)(start%36000);t->fov2_span_cdeg=(uint16_t)span;return true;
}
static bool in_window(unsigned angle,unsigned start,unsigned span){
 return span && (span==36000 || (angle+36000-start)%36000<=span);
}
static bool visible(const FeatureTracker *t,unsigned angle){
 return in_window(angle,t->fov_start_cdeg,t->fov_span_cdeg)||in_window(angle,t->fov2_start_cdeg,t->fov2_span_cdeg);
}
static bool edge_window(unsigned angle,unsigned start,unsigned span){
 if(!span||span==36000)return false;
 unsigned d=(angle+36000-start)%36000;
 return d<=span && (d<=300 || span-d<=300);
}
static bool at_edge(const FeatureTracker *t,unsigned angle){
 return edge_window(angle,t->fov_start_cdeg,t->fov_span_cdeg)||edge_window(angle,t->fov2_start_cdeg,t->fov2_span_cdeg);
}
static void insert(Candidate *out,unsigned *n,const Candidate *c) {
    unsigned at=*n;
    if(at==CANDIDATE_MAX) {
        if(c->range>=out[at-1].range)return;
        at--;
    } else (*n)++;
    while(at && out[at-1].range>c->range){out[at]=out[at-1];at--;}
    out[at]=*c;
}
static void cluster(FeatureTracker *t,const Ld19Scan *s,unsigned lo,unsigned hi,
                    unsigned start,unsigned total,Candidate *out,unsigned *count) {
    if(hi<=lo)return;
    Candidate c={0}; float xx=0,yy=0,xy=0;
    unsigned first=(start+lo)%total;
    float xmin=t->px[first],xmax=xmin,ymin=t->py[first],ymax=ymin;
    unsigned intensity=0;
    for(unsigned k=lo;k<hi;k++){
        unsigned i=(start+k)%total;
        float x=t->px[i],y=t->py[i];c.x+=x;c.y+=y;
        if(at_edge(t,s->points[t->point_index[i]].angle_cdeg))c.edge=true;
        intensity+=s->points[t->point_index[i]].intensity;
        if(x<xmin)xmin=x;
        if(x>xmax)xmax=x;
        if(y<ymin)ymin=y;
        if(y>ymax)ymax=y;
    }
    c.n=(uint16_t)(hi-lo);c.x/=c.n;c.y/=c.n;
    for(unsigned k=lo;k<hi;k++){
        unsigned i=(start+k)%total;
        float x=t->px[i]-c.x,y=t->py[i]-c.y;xx+=x*x;yy+=y*y;xy+=x*y;
    }
    xx/=c.n;yy/=c.n;xy/=c.n;
    float disc=sqrtf((xx-yy)*(xx-yy)+4*xy*xy);
    float minor=fmaxf(0,(xx+yy-disc)*0.5f),major=(xx+yy+disc)*0.5f;
    c.range=hypotf(c.x,c.y);
    c.extent=(uint16_t)fminf(24000,hypotf(xmax-xmin,ymax-ymin));
    c.line=c.n>=5 && c.extent>=120 && major>1 && minor/major<0.08f;
    c.kind=c.line?2:0;
    if(c.line){
        float angle=.5f*atan2f(2*xy,xx-yy),ux=cosf(angle),uy=sinf(angle);
        float low_proj=1e30f,high_proj=-1e30f,width=0;
        /* Use every actual sample to enclose the fitted strip, including
         * deviations from the PCA line; sigma alone is not a bound. */
        for(unsigned k=lo;k<hi;k++){
            unsigned i=(start+k)%total;
            float x=t->px[i]-c.x,y=t->py[i]-c.y,along=ux*x+uy*y;
            low_proj=fminf(low_proj,along);high_proj=fmaxf(high_proj,along);
            width=fmaxf(width,fabsf(-uy*x+ux*y));
        }
        c.ax=c.x+ux*low_proj;c.ay=c.y+uy*low_proj;
        c.bx=c.x+ux*high_proj;c.by=c.y+uy*high_proj;
        /* Endpoint rounding is <=0.71mm; retain1mm extra enclosure. */
        c.halfwidth=(uint16_t)fminf(24000,ceilf(width)+1);
    }
    c.sigma=fmaxf(10.0f,sqrtf(minor)+(c.n<3?15.0f:5.0f));
    c.intensity=(uint8_t)(intensity/c.n);insert(out,count,&c);
}
static unsigned points(FeatureTracker *t,const Ld19Scan *s,const FilterConfig *f,
                        const MotionHistory *motion,MotionPose *reference,bool *pose_ok){
    *pose_ok=motion && motion_history_at(motion,s->t1_us,reference);
    *pose_ok=*pose_ok && reference->sigma_mm<=100 && reference->sigma_yaw_mrad<=100;
    t->deskew_valid=*pose_ok && !s->partial;
    if(t->deskew_valid){
        MotionPose p;
        for(unsigned i=0;i<s->n;i++)if(visible(t,s->points[i].angle_cdeg) && (!motion_history_at(motion,s->points[i].t_us,&p) || p.sigma_mm>100 || p.sigma_yaw_mrad>100)){
            t->deskew_valid=false;break;
        }
    }
    unsigned n=0;t->masked_points=0;
    for(unsigned i=0;i<s->n;i++){
        const ScanPoint *p=&s->points[i];
        if(!visible(t,p->angle_cdeg)){t->masked_points++;continue;}
        if(!p->distance_mm || p->distance_mm<f->r_min_mm ||
           p->distance_mm>f->r_max_mm || p->intensity<f->intensity_min)continue;
        float a=p->angle_cdeg*0.000174532925f;
        float x=p->distance_mm*cosf(a),y=-p->distance_mm*sinf(a);
        if(t->deskew_valid){
            MotionPose at; (void)motion_history_at(motion,p->t_us,&at);
            motion_transform_point(&at,reference,x,y,&x,&y);
        }
        t->px[n]=x;t->py[n]=y;t->point_index[n++]=(uint16_t)i;
    }
    return n;
}
static uint16_t newid(FeatureTracker *t){
    for(unsigned guard=0;guard<TRACK_MAX+1;guard++){
        uint16_t id=t->next_id++;
        if(!id)id=t->next_id++;
        bool clash=false;
        for(unsigned i=0;i<TRACK_MAX;i++)if(t->slots[i].used&&t->slots[i].value.id==id)clash=true;
        if(!clash)return id;
    }
    return 0;
}
uint32_t features_extract_motion(FeatureTracker *t,const Ld19Scan *s,const FilterConfig *f,
                          const MotionHistory *motion,PbFeature *out,uint32_t cap){
    if(!t||!s||!f||!out||cap==0||s->n>SCAN_MAX_POINTS)return 0;
    if(t->have_scan && (int32_t)(s->scan_id-t->last_scan_id)<=0)return 0;
    Candidate cand[CANDIDATE_MAX];unsigned nc=0;
    MotionPose pose={0};bool pose_ok=false;
    unsigned n=points(t,s,f,motion,&pose,&pose_ok),start=0;
    /* Complete circles can join across zero degrees: start at the largest
     * geometric gap so one object is not split at the arbitrary angle seam. */
    if(!s->partial && n>1){
        float largest=-1;
        for(unsigned i=0;i<n;i++){
            unsigned j=(i+1)%n;
            const ScanPoint *a=&s->points[t->point_index[i]],*b=&s->points[t->point_index[j]];
            float angle=((b->angle_cdeg+36000U-a->angle_cdeg)%36000U)*0.000174532925f;
            float gap=hypotf(t->px[i]-t->px[j],t->py[i]-t->py[j])+
                      angle*fminf(a->distance_mm,b->distance_mm);
            if(gap>largest){largest=gap;start=j;}
        }
    }
    unsigned lo=0;
    for(unsigned k=1;k<n;k++){
        unsigned i=(start+k)%n,j=(start+k-1)%n;
        const ScanPoint *a=&s->points[t->point_index[j]],*b=&s->points[t->point_index[i]];
        float gap=hypotf(t->px[i]-t->px[j],t->py[i]-t->py[j]);
        float angle=((b->angle_cdeg+36000U-a->angle_cdeg)%36000U)*0.000174532925f;
        float range=fminf(a->distance_mm,b->distance_mm);
        float threshold=fminf(f->cluster_gap_mm,25.0f+.02f*range)+0.5f*range*angle;
        if(angle>0.087266463f || gap>threshold){cluster(t,s,lo,k,start,n,cand,&nc);lo=k;}
    }
    if(n)cluster(t,s,lo,n,start,n,cand,&nc);
    bool ego=pose_ok&&t->previous_pose_valid;
    float dt=t->have_scan?(float)(uint32_t)(s->t1_us-t->last_time_us)*1e-6f:0;
    for(unsigned i=0;i<TRACK_MAX;i++){
        TrackSlot *slot=&t->slots[i];
        if(!slot->used)continue;
        slot->matched=false;
        slot->value.flags&=(uint16_t)~(TRACK_OBSERVED|TRACK_EGO_COMPENSATED|TRACK_DESKEWED|TRACK_NEAR_RETURN);
        uint32_t age=s->t1_us-slot->last_seen_us;
        if(age>500000U){slot->used=false;continue;}
        if(ego){
            float x,y;
            motion_transform_point(&t->previous_pose,&pose,slot->value.x_mm,slot->value.y_mm,&x,&y);
            slot->value.x_mm=x;slot->value.y_mm=y;
            float a=t->previous_pose.yaw_rad-pose.yaw_rad,c=cosf(a),sn=sinf(a);
            x=slot->value.vx_mm_s;y=slot->value.vy_mm_s;
            slot->value.vx_mm_s=c*x-sn*y;slot->value.vy_mm_s=sn*x+c*y;
            slot->value.flags|=TRACK_EGO_COMPENSATED;
        }
        slot->value.seen_age_ms=(uint16_t)(age/1000U);
    }
    for(unsigned ci=0;ci<nc;ci++){
        Candidate *c=&cand[ci];int best=-1;float bestd=1e30f;
        for(unsigned i=0;i<TRACK_MAX;i++){
            TrackSlot *slot=&t->slots[i];if(!slot->used||slot->matched)continue;
            float d=hypotf(c->x-slot->value.x_mm,c->y-slot->value.y_mm);
            float gate=fminf(150,60+2*(c->sigma+slot->value.sigma_mm));
            if(d<gate && d<bestd && fabsf((float)c->extent-slot->value.extent_mm)<200){best=(int)i;bestd=d;}
        }
        if(best<0){
            for(unsigned i=0;i<TRACK_MAX;i++)if(!t->slots[i].used){best=(int)i;break;}
            if(best<0){t->capacity_drops++;continue;}
            TrackSlot *slot=&t->slots[best];memset(slot,0,sizeof *slot);
            slot->value.id=newid(t);slot->used=true;
            slot->value.x_mm=c->x;slot->value.y_mm=c->y;
        }
        TrackSlot *slot=&t->slots[best];LidarTrack *v=&slot->value;
        float dx=c->x-v->x_mm,dy=c->y-v->y_mm;
        float innovation=hypotf(dx,dy);
        slot->innovation_mm=v->hits?.65f*slot->innovation_mm+.35f*innovation:0;
        bool weak=c->n<f->min_points || slot->innovation_mm>30.0f;
        /* New noisy matches cannot inherit an old permanent confirmation.
         * Moving compact targets remain observable for near-field braking. */
        if(slot->innovation_mm>30.0f){slot->streak=0;slot->confirmed=false;}
        if(ego&&dt>0.005f&&dt<0.3f&&v->hits){
            v->vx_mm_s=0.7f*v->vx_mm_s+0.3f*dx/dt;
            v->vy_mm_s=0.7f*v->vy_mm_s+0.3f*dy/dt;
        }else{v->vx_mm_s=0;v->vy_mm_s=0;}
        float alpha=ego?0.65f:0.85f;v->x_mm+=alpha*dx;v->y_mm+=alpha*dy;
        v->kind=c->kind;v->extent_mm=c->extent;v->npoints=c->n;
        slot->ax_mm=c->ax;slot->ay_mm=c->ay;slot->bx_mm=c->bx;slot->by_mm=c->by;
        slot->halfwidth_mm=c->halfwidth;
        v->sigma_mm=(uint16_t)ceilf(c->sigma);v->seen_age_ms=0;
        if(v->hits<65535)v->hits++;
        if(!s->partial && slot->streak<65535)slot->streak++;
        unsigned needed=f->confirm_scans;
        if(weak && needed<5)needed=5;
        if(slot->streak>=needed)slot->confirmed=true;
        v->state=slot->confirmed?TRACK_CONFIRMED:TRACK_TENTATIVE;
        v->flags=TRACK_OBSERVED|(ego?TRACK_EGO_COMPENSATED:0)|(t->deskew_valid?TRACK_DESKEWED:0)|
            (c->range<450?TRACK_NEAR_RETURN:0)|(c->line?(TRACK_LINE_FIT|TRACK_SEGMENT):0)|(weak?TRACK_WEAK_SUPPORT:0)|(c->edge?TRACK_FOV_EDGE:0);
        if(ego&&v->hits>=3&&hypotf(v->vx_mm_s,v->vy_mm_s)>150)v->flags|=TRACK_DYNAMIC;
        slot->intensity=c->intensity;slot->matched=true;slot->last_seen_us=s->t1_us;
    }
    t->count=0;
    for(unsigned i=0;i<TRACK_MAX;i++){
        TrackSlot *slot=&t->slots[i];if(!slot->used)continue;
        if(!slot->matched){slot->streak=0;slot->confirmed=false;slot->value.state=TRACK_COASTING;}
        t->count++;
    }
    t->last_scan_id=s->scan_id;t->last_time_us=s->t1_us;t->have_scan=true;
    t->previous_pose=pose;t->previous_pose_valid=pose_ok;t->candidate_count=(uint16_t)nc;
    unsigned made=0;
    if(cap>FEATURE_MAX)cap=FEATURE_MAX;
    bool selected[TRACK_MAX]={0};
    while(made<cap){
        int best=-1;float range=1e30f;
        for(unsigned i=0;i<TRACK_MAX;i++)if(t->slots[i].used&&t->slots[i].matched&&!selected[i]){
            float r=hypotf(t->slots[i].value.x_mm,t->slots[i].value.y_mm);
            if(r<range){range=r;best=(int)i;}
        }
        if(best<0)break;
        selected[best]=true;
        LidarTrack *v=&t->slots[best].value;PbFeature *p=&out[made++];memset(p,0,sizeof *p);
        p->id=v->id;p->kind=v->kind;p->npoints=(uint8_t)(v->npoints>255?255:v->npoints);
        p->x_mm=(int16_t)lroundf(v->x_mm);p->y_mm=(int16_t)lroundf(v->y_mm);
        p->intensity=t->slots[best].intensity;p->fit_error_mm=65535;
    }
    return made;
}
uint32_t features_extract(FeatureTracker *t,const Ld19Scan *s,const FilterConfig *f,PbFeature *o,uint32_t cap){
    return features_extract_motion(t,s,f,NULL,o,cap);
}
uint8_t features_track_batches(const FeatureTracker *t){return (uint8_t)(t->count?(t->count+TRACK_BATCH_MAX-1)/TRACK_BATCH_MAX:1);}
static void u16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void u32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
uint32_t features_tracks_payload(const FeatureTracker *t,uint32_t time,uint32_t scan,uint8_t batch,uint8_t *out,uint32_t cap){
    uint8_t batches=features_track_batches(t);
    if(!out||batch>=batches||cap<11)return 0;
    u32(out,time);u32(out+4,scan);out[8]=batch;out[9]=batches;out[10]=0;
    unsigned seen=0,n=0;
    for(unsigned i=0;i<TRACK_MAX;i++){
        const TrackSlot *slot=&t->slots[i];if(!slot->used)continue;
        if(seen++<batch*TRACK_BATCH_MAX)continue;
        if(n>=TRACK_BATCH_MAX)break;
        if(11+(n+1)*TRACK_RECORD_BYTES>cap)return 0;
        const LidarTrack *v=&slot->value;uint8_t *p=out+11+n*TRACK_RECORD_BYTES;
        u16(p,v->id);p[2]=v->state;p[3]=v->kind;memcpy(p+4,&v->x_mm,4);memcpy(p+8,&v->y_mm,4);
        memcpy(p+12,&v->vx_mm_s,4);memcpy(p+16,&v->vy_mm_s,4);
        u16(p+20,v->sigma_mm);u16(p+22,v->extent_mm);u16(p+24,v->npoints);
        u16(p+26,v->seen_age_ms);u16(p+28,v->hits);u16(p+30,v->flags);n++;
    }
    out[10]=(uint8_t)n;return 11+n*TRACK_RECORD_BYTES;
}
/* Geometry precedes the matching TRACKS scan. Legacy receivers can ignore
 * 0x76 and retain their original (conservative) disk; no field is repurposed. */
static bool has_segment(const TrackSlot *s){
    return s->used&&s->matched&&(s->value.flags&TRACK_SEGMENT)&&s->value.kind==2;
}
uint8_t features_segment_batches(const FeatureTracker *t){
    unsigned n=0;for(unsigned i=0;i<TRACK_MAX;i++)if(has_segment(&t->slots[i]))n++;
    return (uint8_t)(n?(n+SEGMENT_BATCH_MAX-1)/SEGMENT_BATCH_MAX:1);
}
uint32_t features_segments_payload(const FeatureTracker *t,uint32_t time,uint32_t scan,uint8_t batch,uint8_t *out,uint32_t cap){
    uint8_t batches=features_segment_batches(t);
    if(!out||batch>=batches||cap<11)return 0;
    u32(out,time);u32(out+4,scan);out[8]=batch;out[9]=batches;out[10]=0;
    unsigned seen=0,n=0;
    for(unsigned i=0;i<TRACK_MAX;i++){
        const TrackSlot *slot=&t->slots[i];if(!has_segment(slot))continue;
        if(seen++<batch*SEGMENT_BATCH_MAX)continue;
        if(n>=SEGMENT_BATCH_MAX)break;
        if(11+(n+1)*SEGMENT_RECORD_BYTES>cap)return 0;
        const float xy[]={slot->ax_mm,slot->ay_mm,slot->bx_mm,slot->by_mm};
        for(unsigned j=0;j<4;j++)if(!isfinite(xy[j])||fabsf(xy[j])>32760)return 0;
        uint8_t *p=out+11+n*SEGMENT_RECORD_BYTES;u16(p,slot->value.id);
        for(unsigned j=0;j<4;j++)u16(p+2+j*2,(uint16_t)(int16_t)lroundf(xy[j]));
        u16(p+10,slot->halfwidth_mm);n++;
    }
    out[10]=(uint8_t)n;return 11+n*SEGMENT_RECORD_BYTES;
}
