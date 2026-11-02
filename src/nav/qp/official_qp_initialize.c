/*
 * File: official_qp_initialize.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "official_qp_initialize.h"
#include "official_qp_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void official_qp_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_official_qp = true;
}

/*
 * File trailer for official_qp_initialize.c
 *
 * [EOF]
 */
