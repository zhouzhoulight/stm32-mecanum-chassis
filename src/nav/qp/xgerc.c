/*
 * File: xgerc.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "xgerc.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : int32_T m
 *                int32_T n
 *                real32_T alpha1
 *                int32_T ix0
 *                const real32_T y[8]
 *                real32_T A[64]
 *                int32_T ia0
 * Return Type  : void
 */
void xgerc(int32_T m, int32_T n, real32_T alpha1, int32_T ix0,
           const real32_T y[8], real32_T A[64], int32_T ia0)
{
  int32_T i;
  int32_T i1;
  int32_T ijA;
  int32_T j;
  int32_T jA;
  real32_T temp;
  if (!(alpha1 == 0.0F)) {
    jA = ia0;
    i = (uint8_T)n;
    for (j = 0; j < i; j++) {
      temp = y[j];
      if (temp != 0.0F) {
        temp *= alpha1;
        i1 = m + jA;
        for (ijA = jA; ijA < i1; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += 8;
    }
  }
}

/*
 * File trailer for xgerc.c
 *
 * [EOF]
 */
