#include "motion_history.h"
#include <math.h>
#include <string.h>
static float wrap(float a) {
    while (a > 3.141592654f) a -= 6.283185307f;
    while (a < -3.141592654f) a += 6.283185307f;
    return a;
}
void motion_history_init(MotionHistory *h) { memset(h, 0, sizeof *h); }
static const MotionPose *sample(const MotionHistory *h, unsigned i) {
    return &h->samples[(h->head + MOTION_HISTORY_CAP - h->count + i) % MOTION_HISTORY_CAP];
}
bool motion_history_add(MotionHistory *h, const MotionPose *p) {
    if (!h || !p || !isfinite(p->x_mm) || !isfinite(p->y_mm) ||
        !isfinite(p->yaw_rad) || fabsf(p->x_mm) > 1000000.0f ||
        fabsf(p->y_mm) > 1000000.0f || fabsf(p->yaw_rad) > 100.0f ||
        p->sigma_mm == 0U || p->sigma_mm > 5000U ||
        p->sigma_yaw_mrad == 0U || p->sigma_yaw_mrad > 3142U) {
        if (h) h->rejected++;
        return false;
    }
    if (h->count) {
        const MotionPose *last=sample(h,h->count-1U);
        int32_t dt=(int32_t)(p->t_us-last->t_us);
        if(dt<=0 || hypotf(p->x_mm-last->x_mm,p->y_mm-last->y_mm) > 5000.0f*dt*1e-6f ||
            fabsf(wrap(p->yaw_rad-last->yaw_rad))>20.0f*dt*1e-6f){
            h->rejected++; return false;
        }
    }
    h->samples[h->head] = *p;
    h->samples[h->head].yaw_rad = wrap(p->yaw_rad);
    h->head = (uint16_t)((h->head + 1U) % MOTION_HISTORY_CAP);
    if (h->count < MOTION_HISTORY_CAP) h->count++;
    return true;
}
bool motion_history_at(const MotionHistory *h, uint32_t t, MotionPose *out) {
    if (!h || !out || h->count == 0U) return false;
    const MotionPose *a = sample(h, 0), *b = a;
    if ((int32_t)(t - a->t_us) < 0) return false;
    for (unsigned i = 1; i < h->count; i++) {
        b = sample(h, i);
        if ((int32_t)(t - b->t_us) <= 0) break;
        a = b;
    }
    const MotionPose *last = sample(h, h->count - 1U);
    if ((int32_t)(t - last->t_us) > 0) {
        if (h->count < 2U || (uint32_t)(t - last->t_us) > MOTION_EXTRAPOLATE_US) return false;
        a = sample(h, h->count - 2U); b = last;
    }
    if (a == b) {
        if (t != a->t_us) return false;
        *out = *a; return true;
    }
    uint32_t dt = b->t_us - a->t_us;
    if (dt == 0U || dt > MOTION_MAX_GAP_US) return false;
    float u = (float)(t - a->t_us) / (float)dt;
    out->t_us = t;
    out->x_mm = a->x_mm + u * (b->x_mm - a->x_mm);
    out->y_mm = a->y_mm + u * (b->y_mm - a->y_mm);
    out->yaw_rad = wrap(a->yaw_rad + u * wrap(b->yaw_rad - a->yaw_rad));
    unsigned extra = u > 1.0f ? 10U : 0U;
    out->sigma_mm = (uint16_t)((a->sigma_mm > b->sigma_mm ? a->sigma_mm : b->sigma_mm) + extra);
    out->sigma_yaw_mrad = (uint16_t)((a->sigma_yaw_mrad > b->sigma_yaw_mrad ? a->sigma_yaw_mrad : b->sigma_yaw_mrad) + extra);
    return true;
}
void motion_transform_point(const MotionPose *from, const MotionPose *to,
                            float x, float y, float *ox, float *oy) {
    float c = cosf(from->yaw_rad), s = sinf(from->yaw_rad);
    float wx = from->x_mm + c*x - s*y - to->x_mm;
    float wy = from->y_mm + s*x + c*y - to->y_mm;
    c = cosf(to->yaw_rad); s = sinf(to->yaw_rad);
    *ox = c*wx + s*wy; *oy = -s*wx + c*wy;
}
