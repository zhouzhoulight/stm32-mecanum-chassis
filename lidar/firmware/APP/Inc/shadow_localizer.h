#ifndef SHADOW_LOCALIZER_H
#define SHADOW_LOCALIZER_H
#include "features.h"
#include "pose_match.h"
typedef struct { uint16_t id;float x,y,sigma; } ShadowAnchor;
typedef struct {
    ShadowAnchor anchors[POSE_PAIR_MAX];
    uint16_t count;
    bool pending;
    PoseMatch result;
} ShadowLocalizer;
void shadow_reset(ShadowLocalizer *s);
void shadow_request_anchor(ShadowLocalizer *s);
void shadow_update(ShadowLocalizer *s,const FeatureTracker *t);
uint32_t shadow_payload(const ShadowLocalizer *s,uint32_t time_us,uint8_t out[26]);
#endif
