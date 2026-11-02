/*
 * File: mpcActiveSetSolver.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

#ifndef MPCACTIVESETSOLVER_H
#define MPCACTIVESETSOLVER_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int32_T mpcActiveSetSolver(const real32_T f[8], const real32_T b[32],
                           boolean_T iA0[32], real32_T x[8]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for mpcActiveSetSolver.h
 *
 * [EOF]
 */
