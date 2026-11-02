/*
 * File: xgemv.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

#ifndef XGEMV_H
#define XGEMV_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void xgemv(int32_T m, int32_T n, const real32_T A[64], int32_T ia0,
           const real32_T x[64], int32_T ix0, real32_T y[8]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgemv.h
 *
 * [EOF]
 */
