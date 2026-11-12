#ifndef POSE_MATCH_H
#define POSE_MATCH_H
#include <stdbool.h>
#include <stdint.h>
#define POSE_PAIR_MAX 16U
typedef struct { float sx,sy,tx,ty,sigma_mm; } PosePair;
typedef struct {
    float x_mm,y_mm,yaw_rad,rms_mm,sigma_mm,sigma_yaw_rad;
    uint16_t inliers;
    bool valid;
} PoseMatch;
/* Explicit correspondences only; bounded consensus + robust weighted SE(2).
 * This is a local shadow estimate, not an independently surveyed global fix. */
bool pose_match(const PosePair *pairs,unsigned count,PoseMatch *out);
#endif
