#pragma once
#include <stdbool.h>
#include <stdint.h>
bool nav_lidar_front150(float ex,float ey,float *x,float *y,float *dx,float *dy,bool segment);
void nav_lidar_echo_reset(void);
bool nav_lidar_echo_accept(uint16_t id,float bx,float by,uint32_t stamp,bool departure,bool known_obstacle);
typedef struct {unsigned hits,far_hits;uint32_t age_ms;bool body_locked;} NavEchoEvidence;
void nav_lidar_evidence_reset(void);
void nav_lidar_evidence_observe(uint16_t id,float x,float y,float bx,float by,
 float px,float py,float yaw,bool segment,float hx,float hy,uint32_t scan,uint32_t stamp);
NavEchoEvidence nav_lidar_evidence(uint16_t id,float x,float y,uint32_t scan,uint32_t stamp);
void nav_lidar_evidence_shift(float dx,float dy);
