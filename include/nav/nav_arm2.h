#ifndef NAV_ARM2_H
#define NAV_ARM2_H
#include <stdbool.h>
#include <stdint.h>

/* Body X forward, Y left, Z up. Lengths in mm, joint yaw in radians. */
typedef struct {float x,y,z;} Arm2Point;
typedef struct {float yaw,reach,z;} Arm2Pose;
typedef struct {
 float pivot_x,pivot_y,tool_side;
 float yaw_min,yaw_max,reach_min,reach_max,z_min,z_max;
 float safe_z,sweep_halfwidth,tool_below_tcp;
 float velocity[3],accel[3];
 float settle_s;
} Arm2Geometry;
typedef struct {float xmin,xmax,ymin,ymax,height;} Arm2Box;
typedef struct {Arm2Pose pose;float error_mm;} Arm2Solution;
typedef struct {Arm2Pose target;float seconds;uint8_t axes;} Arm2Stage;
typedef struct {Arm2Stage stage[5];unsigned count;float seconds;} Arm2Plan;
extern const Arm2Geometry nav_arm2_preview;

bool nav_arm2_valid(const Arm2Geometry *g);
Arm2Point nav_arm2_tcp(const Arm2Geometry *g,Arm2Pose pose);
bool nav_arm2_solve(const Arm2Geometry *g,Arm2Point target,float near_yaw,
                    bool lock_yaw,float tolerance_mm,Arm2Solution *out);
bool nav_arm2_transfer(const Arm2Geometry *g,Arm2Pose from,Arm2Pose to,
                       const Arm2Box *boxes,unsigned count,bool parallel,Arm2Plan *out);
float nav_arm2_axis_time(float distance,float vmax,float accel);
bool nav_arm2_command(const char *line);
#endif
