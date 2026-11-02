/*
 * File: qpkwik.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

#ifndef QPKWIK_H
#define QPKWIK_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void DropConstraint(int32_T kDrop, boolean_T iA[32], int32_T *nA,
                    int32_T iC[32]);

real32_T KWIKfactor(const real32_T Ac[256], const int32_T iC[32], int32_T nA,
                    const real32_T Linv[64], real32_T RLinv[64], real32_T D[64],
                    real32_T H[64]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for qpkwik.h
 *
 * [EOF]
 */
