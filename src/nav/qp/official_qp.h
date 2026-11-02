/*
 * File: official_qp.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

#ifndef OFFICIAL_QP_H
#define OFFICIAL_QP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void official_qp(const real32_T f[8], const real32_T b[32],
                        const boolean_T active0[32], real32_T x[8],
                        int32_T *status, boolean_T active[32]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for official_qp.h
 *
 * [EOF]
 */
