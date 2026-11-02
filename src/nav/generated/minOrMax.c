/*
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 21:26:50
 */

/* Include Files */
#include "minOrMax.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const real32_T x[4]
 * Return Type  : real32_T
 */
real32_T maximum(const real32_T x[4])
{
  real32_T ex;
  ex = x[0];
  if (x[0] < x[1]) {
    ex = x[1];
  }
  if (ex < x[2]) {
    ex = x[2];
  }
  if (ex < x[3]) {
    ex = x[3];
  }
  return ex;
}

/*
 * File trailer for minOrMax.c
 *
 * [EOF]
 */
