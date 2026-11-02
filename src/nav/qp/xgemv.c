/*
 * File: xgemv.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "xgemv.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int32_T m
 *                int32_T n
 *                const real32_T A[64]
 *                int32_T ia0
 *                const real32_T x[64]
 *                int32_T ix0
 *                real32_T y[8]
 * Return Type  : void
 */
void xgemv(int32_T m, int32_T n, const real32_T A[64], int32_T ia0,
           const real32_T x[64], int32_T ix0, real32_T y[8])
{
  int32_T i;
  int32_T i1;
  int32_T ia;
  int32_T iac;
  real32_T c;
  if (n != 0) {
    i = (uint8_T)n;
    memset(&y[0], 0, (uint32_T)i * sizeof(real32_T));
    i = ia0 + ((n - 1) << 3);
    for (iac = ia0; iac <= i; iac += 8) {
      c = 0.0F;
      i1 = iac + m;
      for (ia = iac; ia < i1; ia++) {
        c += A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      i1 = (iac - ia0) >> 3;
      y[i1] += c;
    }
  }
}

/*
 * File trailer for xgemv.c
 *
 * [EOF]
 */
