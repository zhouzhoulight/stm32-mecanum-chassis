#ifndef MOTION_HISTORY_H
#define MOTION_HISTORY_H
#include <stdbool.h>
#include <stdint.h>
#define MOTION_HISTORY_CAP 128U
#define MOTION_MAX_GAP_US 100000U
#define MOTION_EXTRAPOLATE_US 30000U
/* LASER pose in a fixed local frame, time in MCU TIM2 us. Sender must apply
 * calibrated base->laser extrinsics and an explicit clock mapping. */
typedef struct {
    uint32_t t_us;
    float x_mm, y_mm, yaw_rad;
    uint16_t sigma_mm, sigma_yaw_mrad;
} MotionPose;
typedef struct {
    MotionPose samples[MOTION_HISTORY_CAP];
    uint16_t head, count;
    uint32_t rejected;
} MotionHistory;
void motion_history_init(MotionHistory *h);
bool motion_history_add(MotionHistory *h, const MotionPose *p);
bool motion_history_at(const MotionHistory *h, uint32_t t_us, MotionPose *out);
void motion_transform_point(const MotionPose *from, const MotionPose *to,
                            float x, float y, float *out_x, float *out_y);
#endif
