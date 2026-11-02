/*
 * File: norm.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "norm.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const real32_T x[8]
 * Return Type  : real32_T
 */
real32_T b_norm(const real32_T x[8])
{
  int32_T k;
  real32_T absxk;
  real32_T scale;
  real32_T t;
  real32_T y;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (k = 0; k < 8; k++) {
    absxk = (real32_T)fabsf(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }
  return scale * (real32_T)sqrtf(y);
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
