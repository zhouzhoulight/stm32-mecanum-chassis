#ifndef FEATURES_H
#define FEATURES_H
#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"
#include "scan_builder.h"
#include "protocol.h"
#include "motion_history.h"
#define TRACK_MAX 32U
#define TRACK_BATCH_MAX 7U
#define TRACK_RECORD_BYTES 32U
#define TRACK_OBSERVED 0x01U
#define TRACK_EGO_COMPENSATED 0x02U
#define TRACK_NEAR_RETURN 0x04U
#define TRACK_LINE_FIT 0x08U
#define TRACK_DYNAMIC 0x10U
#define TRACK_DESKEWED 0x20U
#define TRACK_FOV_EDGE 0x80U /* partially visible near angular mask boundary; no localization */
#define TRACK_WEAK_SUPPORT 0x40U /* sparse or spatially inconsistent; not a permanent obstacle */
enum { TRACK_TENTATIVE=1, TRACK_CONFIRMED=2, TRACK_COASTING=3 };
typedef struct {
    uint16_t id;
    uint8_t state, kind;
    float x_mm, y_mm, vx_mm_s, vy_mm_s;
    uint16_t sigma_mm, extent_mm, npoints, seen_age_ms, hits, flags;
} LidarTrack;
typedef struct {
    LidarTrack value;
    uint32_t last_seen_us;
    uint16_t streak;
    float innovation_mm;
    uint8_t intensity;
    bool used, matched, confirmed;
} TrackSlot;
typedef struct {
    uint16_t next_id;
    TrackSlot slots[TRACK_MAX];
    MotionPose previous_pose;
    uint32_t last_scan_id, last_time_us;
    uint16_t count, candidate_count;
    bool have_scan, previous_pose_valid, deskew_valid;
    uint32_t capacity_drops;
    uint16_t fov_start_cdeg,fov_span_cdeg,masked_points;
    uint16_t fov2_start_cdeg,fov2_span_cdeg;
    float px[SCAN_MAX_POINTS], py[SCAN_MAX_POINTS];
    uint16_t point_index[SCAN_MAX_POINTS];
} FeatureTracker;
void features_init(FeatureTracker *t);
bool features_add_fov(FeatureTracker *,unsigned start_cdeg,unsigned end_cdeg);
bool features_set_fov(FeatureTracker *t,unsigned start_cdeg,unsigned end_cdeg);
uint32_t features_extract(FeatureTracker *t, const Ld19Scan *scan,
                          const FilterConfig *filter, PbFeature *out, uint32_t cap);
uint32_t features_extract_motion(FeatureTracker *t, const Ld19Scan *scan,
                          const FilterConfig *filter, const MotionHistory *motion,
                          PbFeature *out, uint32_t cap);
uint32_t features_tracks_payload(const FeatureTracker *t, uint32_t t_us, uint32_t scan_id,
                                 uint8_t batch, uint8_t *out, uint32_t cap);
uint8_t features_track_batches(const FeatureTracker *t);
#endif
