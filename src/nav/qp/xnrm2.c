/*
 * File: xnrm2.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : int32_T n
 *                const real32_T x[64]
 *                int32_T ix0
 * Return Type  : real32_T
 */
real32_T xnrm2(int32_T n, const real32_T x[64], int32_T ix0)
{
  int32_T k;
  int32_T kend;
  real32_T absxk;
  real32_T scale;
  real32_T t;
  real32_T y;
  y = 0.0F;
  if (n >= 1) {
    if (n == 1) {
      y = (real32_T)fabsf(x[ix0 - 1]);
    } else {
      scale = 1.29246971E-26F;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        absxk = (real32_T)fabsf(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * (real32_T)sqrtf(y);
    }
  }
  return y;
}

/*
 * File trailer for xnrm2.c
 *
 * [EOF]
 */
