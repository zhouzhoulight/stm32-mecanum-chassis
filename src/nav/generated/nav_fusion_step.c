/*
 * File: nav_fusion_step.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 21:26:50
 */

/* Include Files */
#include "nav_fusion_step.h"
#include "eye.h"
#include "minOrMax.h"
#include "nav_fusion_step_data.h"
#include "nav_fusion_step_initialize.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static real32_T rt_powf_snf(real32_T u0, real32_T u1);

/* Function Definitions */
/*
 * Arguments    : real32_T u0
 *                real32_T u1
 * Return Type  : real32_T
 */
static real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T f;
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else {
    y = (real32_T)fabsf(u0);
    f = (real32_T)fabsf(u1);
    if (rtIsInfF(u1)) {
      if (y == 1.0F) {
        y = 1.0F;
      } else if (y > 1.0F) {
        if (u1 > 0.0F) {
          y = rtInfF;
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = rtInfF;
      }
    } else if (f == 0.0F) {
      y = 1.0F;
    } else if (f == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (real32_T)sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > (real32_T)floorf(u1))) {
      y = rtNaNF;
    } else {
      y = (real32_T)powf(u0, u1);
    }
  }
  return y;
}

/*
 * Proposed hybrid: WT heading + SE2 + robust velocity KF (not a full INS EKF).
 *  Effective state: XY/yaw + two body velocities; x6=rate, x7=elapsed time.
 *  Only velocity covariance is a KF covariance; pose covariance is a heuristic.
 *
 * Arguments    : real32_T x[7]
 *                real32_T P[49]
 *                const real32_T u[9]
 *                const real32_T geometry[2]
 *                real32_T d[4]
 * Return Type  : void
 */
void nav_fusion_step(real32_T x[7], real32_T P[49], const real32_T u[9],
                     const real32_T geometry[2], real32_T d[4])
{
  int32_T ambiguous;
  int32_T b_i;
  int32_T flag;
  int32_T i;
  int32_T j;
  int32_T rejected;
  real32_T H[8];
  real32_T C[4];
  real32_T C_tmp[4];
  real32_T b_I[4];
  real32_T c_I[4];
  real32_T rv[4];
  real32_T z[4];
  real32_T K[2];
  real32_T v[2];
  real32_T a;
  real32_T b;
  real32_T dh;
  real32_T ea;
  real32_T eb;
  real32_T f;
  real32_T f1;
  real32_T f2;
  real32_T f3;
  real32_T omega;
  real32_T z_tmp;
  if (!isInitialized_nav_fusion_step) {
    nav_fusion_step_initialize();
  }
  dh = u[5] - x[2];
  omega = dh / u[6];
  H[0] = 1.0F;
  H[1] = 1.0F;
  H[2] = 1.0F;
  H[3] = 1.0F;
  H[4] = -geometry[0];
  H[5] = geometry[0];
  H[6] = geometry[0];
  H[7] = -geometry[0];
  eb = -geometry[1] * omega;
  z[0] = u[0] - eb;
  z_tmp = geometry[1] * omega;
  z[1] = u[1] - z_tmp;
  z[2] = u[2] - eb;
  z[3] = u[3] - z_tmp;
  eye(C_tmp);
  a = 0.25F * u[6];
  b = a * a;
  v[0] = x[3];
  C[0] = P[24] + C_tmp[0] * b;
  C[1] = P[25] + C_tmp[1] * b;
  v[1] = x[4];
  C[2] = P[31] + C_tmp[2] * b;
  C[3] = P[32] + C_tmp[3] * b;
  rv[0] = 6.4E-5F;
  rv[1] = 6.4E-5F;
  rv[2] = 6.4E-5F;
  rv[3] = 6.4E-5F;
  flag = 0;
  rejected = 0;
  ambiguous = 0;
  if ((real32_T)fabsf(z[0] - z[3]) > 0.03) {
    f = x[3];
    z_tmp = f;
    eb = f;
    f = x[4];
    z_tmp += -geometry[0] * f;
    eb += -geometry[0] * f;
    ea = (real32_T)fabsf(z[0] - z_tmp);
    eb = (real32_T)fabsf(z[3] - eb);
    if ((real32_T)fabsf(ea - eb) > 0.015) {
      if (ea > eb) {
        rv[0] = 0.64F;
        rejected = 1;
      } else {
        rv[3] = 0.64F;
        rejected = 8;
      }
    } else {
      rv[0] = 0.0016F;
      rv[3] = 0.0016F;
      ambiguous = 1;
    }
    flag = 1;
  }
  if ((real32_T)fabsf(z[1] - z[2]) > 0.03) {
    f = x[3];
    z_tmp = geometry[0] * x[4];
    ea = (real32_T)fabsf(z[1] - (f + z_tmp));
    eb = (real32_T)fabsf(z[2] - (f + z_tmp));
    if ((real32_T)fabsf(ea - eb) > 0.015) {
      if (ea > eb) {
        rv[1] = 0.64F;
        rejected = (int32_T)((uint32_T)rejected | 2U);
      } else {
        rv[2] = 0.64F;
        rejected = (int32_T)((uint32_T)rejected | 4U);
      }
    } else {
      rv[1] = 0.0016F;
      rv[2] = 0.0016F;
      ambiguous = (int32_T)((uint32_T)ambiguous | 2U);
    }
    flag++;
  }
  for (j = 0; j < 4; j++) {
    f = H[j + 4];
    f1 = C[3];
    f2 = f * C[3];
    f3 = C[0];
    eb = C[1];
    ea = C[2];
    a = rv[j];
    z_tmp = ((C[0] + f * C[1]) + (C[2] + f2) * f) + a;
    K[0] = (C[0] + C[2] * f) / z_tmp;
    K[1] = (C[1] + f2) / z_tmp;
    b = z[j] - (v[0] + f * v[1]);
    v[0] += K[0] * b;
    b_I[0] = C_tmp[0] - K[0];
    b_I[1] = C_tmp[1] - K[1];
    v[1] += K[1] * b;
    b_I[2] = C_tmp[2] - K[0] * f;
    b_I[3] = C_tmp[3] - K[1] * f;
    for (i = 0; i < 2; i++) {
      f = b_I[i + 2];
      f2 = b_I[i];
      c_I[i] = f2 * f3 + f * eb;
      c_I[i + 2] = f2 * ea + f * f1;
    }
    f = b_I[0];
    f1 = b_I[2];
    f2 = b_I[1];
    f3 = b_I[3];
    for (i = 0; i < 2; i++) {
      eb = c_I[i + 2];
      ea = c_I[i];
      C[i] = ea * f + eb * f1;
      C[i + 2] = ea * f2 + eb * f3;
    }
    eb = K[0] * a;
    z_tmp = K[1] * a;
    C[0] += eb * K[0];
    C[1] += z_tmp * K[0];
    C[2] += eb * K[1];
    C[3] += z_tmp * K[1];
  }
  if (u[8] > 0.0F) {
    v[0] *= 0.1F;
    v[1] *= 0.1F;
  }
  if ((real32_T)fabsf(dh) < 0.001) {
    a = 1.0F - dh * dh / 6.0F;
    b = dh / 2.0F - rt_powf_snf(dh, 3.0F) / 24.0F;
  } else {
    a = (real32_T)sinf(dh) / dh;
    b = (1.0F - (real32_T)cosf(dh)) / dh;
  }
  eb = (real32_T)sinf(x[2]);
  ea = (real32_T)cosf(x[2]);
  z[0] = ea;
  z[1] = eb;
  z[2] = -eb;
  z[3] = ea;
  x[2] = u[5];
  x[5] = omega;
  x[6] += u[6];
  z_tmp = u[6] * u[6];
  for (b_i = 0; b_i < 2; b_i++) {
    x[b_i + 3] = v[b_i];
    f = z[b_i + 2];
    f1 = z[b_i];
    i = b_i << 1;
    j = 7 * (b_i + 3);
    P[j + 3] = C[i];
    f2 = f1 * C[0] + f * C[1];
    P[j + 4] = C[i + 1];
    f3 = f1 * C[2] + f * C[3];
    x[b_i] += ((f1 * a + f * b) * v[0] + (f1 * -b + f * a) * v[1]) * u[6];
    b_I[b_i] = f2 * ea + f3 * -eb;
    b_I[b_i + 2] = f2 * eb + f3 * ea;
  }
  P[0] += b_I[0] * z_tmp;
  P[1] += b_I[1] * z_tmp;
  P[7] += b_I[2] * z_tmp;
  P[8] += b_I[3] * z_tmp;
  P[16] = 6.85389205E-6F;
  d[0] = (real32_T)flag;
  d[1] = maximum(rv) / 6.4E-5F;
  d[2] = (real32_T)rejected;
  d[3] = (real32_T)ambiguous;
}

/*
 * File trailer for nav_fusion_step.c
 *
 * [EOF]
 */
