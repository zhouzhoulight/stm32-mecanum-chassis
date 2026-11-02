/*
 * File: eye.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 21:26:50
 */

/* Include Files */
#include "eye.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : real32_T b_I[4]
 * Return Type  : void
 */
void eye(real32_T b_I[4])
{
  b_I[1] = 0.0F;
  b_I[2] = 0.0F;
  b_I[0] = 1.0F;
  b_I[3] = 1.0F;
}

/*
 * File trailer for eye.c
 *
 * [EOF]
 */
