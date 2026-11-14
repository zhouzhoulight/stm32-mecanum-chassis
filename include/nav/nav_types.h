#ifndef NAV_TYPES_H
#define NAV_TYPES_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define NAV_VERSION "v2.7.38-scene-20261114"
#define NAV_FORWARD_PPM 12924U
#define NAV_LATERAL_PPM 13389U
#define NAV_MAX_WHEEL_RPM 400
#define NAV_TICK_MS 100U
#define NAV_ODOM_MS 50U
#define NAV_FRAME_LOST 1U
#define NAV_ENCODER_JUMP 2U
#define NAV_YAW_JUMP 4U
#define NAV_HISTORY_GAP 8U
#define NAV_NUMERIC_FAULT 16U
/* All arrays are FL,FR,RL,RR. Encoder raw IDs/signs are 4+,1-,3+,2-. */
typedef struct {float x_m,y_m,yaw_rad,vx_mps,vy_mps,path_m;uint32_t at,updates;bool valid;} NavPose;
typedef struct {
 NavPose base,fused;uint32_t fault,epoch,seeded,quiet;
 float pair_count,max_R,lever_m;uint8_t rejected_mask,ambiguous_mask;
 unsigned ppm_x,ppm_y,imu_delay_ms,encoder_delay_ms;
 float response_s;unsigned response_samples;
 float null_residual_mps;unsigned angular_model_samples;bool angular_model;
 int32_t raw[4];uint32_t wheel_at[4];float wheel_mps[4];
} NavOdom;
typedef struct {
 float goal[2],direction[2],length,v,a,integral[3],applied[3],wheel[4];
 float yaw_ff,translation_limit;
 float remaining,cross_error,eso[3][2],lever_m,scale_y,rpm_mps,b0_ratio;
 float work_time,work_duration;
 float work_accel,work_jerk;
 unsigned work_mode;
 uint8_t active_set[3][32];unsigned rpm,stable,algorithm,qp_fallbacks,qp_steps,qp_skips;bool xy,work_move;
} NavControl;
#endif
