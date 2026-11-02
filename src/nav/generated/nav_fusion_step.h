/*
 * File: nav_fusion_step.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 21:26:50
 */

#ifndef NAV_FUSION_STEP_H
#define NAV_FUSION_STEP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void nav_fusion_step(real32_T x[7], real32_T P[49], const real32_T u[9],
                            const real32_T geometry[2], real32_T d[4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for nav_fusion_step.h
 *
 * [EOF]
 */
