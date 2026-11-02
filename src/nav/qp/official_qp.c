/*
 * File: official_qp.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "official_qp.h"
#include "mpcActiveSetSolver.h"
#include "official_qp_data.h"
#include "official_qp_initialize.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Fixed-size, single-precision official KWIK solver. No online factorization.
 *
 * Arguments    : const real32_T f[8]
 *                const real32_T b[32]
 *                const boolean_T active0[32]
 *                real32_T x[8]
 *                int32_T *status
 *                boolean_T active[32]
 * Return Type  : void
 */
void official_qp(const real32_T f[8], const real32_T b[32],
                 const boolean_T active0[32], real32_T x[8], int32_T *status,
                 boolean_T active[32])
{
  int32_T i;
  if (!isInitialized_official_qp) {
    official_qp_initialize();
  }
  for (i = 0; i < 32; i++) {
    active[i] = active0[i];
  }
  *status = mpcActiveSetSolver(f, b, active, x);
}

/*
 * File trailer for official_qp.c
 *
 * [EOF]
 */
