#include "nav/nav_arm2.h"

/* CAD-based preview plus explicit scenario assumptions. All field mounting
 * and motor timing values require calibration on the final integration. */
const Arm2Geometry nav_arm2_preview={
 .pivot_x=0,.pivot_y=-60,.tool_side=0,
 .yaw_min=-3.14159265f,.yaw_max=3.14159265f,
 .reach_min=149,.reach_max=275,.z_min=35,.z_max=255,
 .safe_z=145,.sweep_halfwidth=32,.tool_below_tcp=38,
 .velocity={3.20f,170,65},.accel={7,600,250},.settle_s=.12f
};
