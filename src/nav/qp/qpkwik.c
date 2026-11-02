/*
 * File: qpkwik.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "qpkwik.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int32_T kDrop
 *                boolean_T iA[32]
 *                int32_T *nA
 *                int32_T iC[32]
 * Return Type  : void
 */
void DropConstraint(int32_T kDrop, boolean_T iA[32], int32_T *nA,
                    int32_T iC[32])
{
  int32_T i;
  int32_T q0;
  if (kDrop > 0) {
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      q0 = *nA;
      if (q0 < -2147483647) {
        q0 = MIN_int32_T;
      } else {
        q0--;
      }
      for (i = kDrop; i <= q0; i++) {
        iC[i - 1] = iC[i];
      }
    }
    iC[*nA - 1] = 0;
    q0 = *nA;
    if (q0 < -2147483647) {
      *nA = MIN_int32_T;
    } else {
      *nA = q0 - 1;
    }
  }
}

/*
 * Arguments    : const real32_T Ac[256]
 *                const int32_T iC[32]
 *                int32_T nA
 *                const real32_T Linv[64]
 *                real32_T RLinv[64]
 *                real32_T D[64]
 *                real32_T H[64]
 * Return Type  : real32_T
 */
real32_T KWIKfactor(const real32_T Ac[256], const int32_T iC[32], int32_T nA,
                    const real32_T Linv[64], real32_T RLinv[64], real32_T D[64],
                    real32_T H[64])
{
  int32_T b_i;
  int32_T coltop;
  int32_T exitg1;
  int32_T i;
  int32_T ia;
  int32_T ii;
  int32_T itau;
  int32_T k;
  int32_T knt;
  int32_T lastv;
  real32_T Q[64];
  real32_T R[64];
  real32_T TL[64];
  real32_T tau[8];
  real32_T work[8];
  real32_T Status;
  real32_T a;
  real32_T atmp;
  real32_T beta1;
  real32_T f;
  boolean_T exitg2;
  Status = 1.0F;
  memset(&RLinv[0], 0, 64U * sizeof(real32_T));
  i = (uint8_T)nA;
  for (b_i = 0; b_i < i; b_i++) {
    for (lastv = 0; lastv < 8; lastv++) {
      f = 0.0F;
      for (coltop = 0; coltop < 8; coltop++) {
        f += Linv[lastv + (coltop << 3)] * Ac[(iC[b_i] + (coltop << 5)) - 1];
      }
      RLinv[lastv + (b_i << 3)] = f;
    }
  }
  memcpy(&TL[0], &RLinv[0], 64U * sizeof(real32_T));
  for (b_i = 0; b_i < 8; b_i++) {
    tau[b_i] = 0.0F;
    work[b_i] = 0.0F;
  }
  for (b_i = 0; b_i < 8; b_i++) {
    ii = (b_i << 3) + b_i;
    if (b_i + 1 < 8) {
      atmp = TL[ii];
      coltop = ii + 2;
      tau[b_i] = 0.0F;
      beta1 = xnrm2(7 - b_i, TL, ii + 2);
      if (beta1 != 0.0F) {
        f = TL[ii];
        a = (real32_T)fabsf(f);
        beta1 = (real32_T)fabsf(beta1);
        if (a < beta1) {
          a /= beta1;
          beta1 *= (real32_T)sqrtf(a * a + 1.0F);
        } else if (a > beta1) {
          beta1 /= a;
          beta1 = a * (real32_T)sqrtf(beta1 * beta1 + 1.0F);
        } else if (rtIsNaNF(beta1)) {
          beta1 = rtNaNF;
        } else {
          beta1 = a * 1.41421354F;
        }
        if (f >= 0.0F) {
          beta1 = -beta1;
        }
        if ((real32_T)fabsf(beta1) < 9.86076132E-32F) {
          knt = 0;
          lastv = (ii - b_i) + 8;
          do {
            knt++;
            for (k = coltop; k <= lastv; k++) {
              TL[k - 1] *= 1.01412048E+31F;
            }
            beta1 *= 1.01412048E+31F;
            atmp *= 1.01412048E+31F;
          } while (((real32_T)fabsf(beta1) < 9.86076132E-32F) && (knt < 20));
          a = (real32_T)fabsf(atmp);
          beta1 = (real32_T)fabsf(xnrm2(7 - b_i, TL, ii + 2));
          if (a < beta1) {
            a /= beta1;
            beta1 *= (real32_T)sqrtf(a * a + 1.0F);
          } else if (a > beta1) {
            beta1 /= a;
            beta1 = a * (real32_T)sqrtf(beta1 * beta1 + 1.0F);
          } else if (rtIsNaNF(beta1)) {
            beta1 = rtNaNF;
          } else {
            beta1 = a * 1.41421354F;
          }
          if (atmp >= 0.0F) {
            beta1 = -beta1;
          }
          tau[b_i] = (beta1 - atmp) / beta1;
          a = 1.0F / (atmp - beta1);
          for (k = coltop; k <= lastv; k++) {
            TL[k - 1] *= a;
          }
          for (k = 0; k < knt; k++) {
            beta1 *= 9.86076132E-32F;
          }
          atmp = beta1;
        } else {
          tau[b_i] = (beta1 - f) / beta1;
          a = 1.0F / (f - beta1);
          lastv = (ii - b_i) + 8;
          for (k = coltop; k <= lastv; k++) {
            TL[k - 1] *= a;
          }
          atmp = beta1;
        }
      }
      TL[ii] = 1.0F;
      if (tau[b_i] != 0.0F) {
        lastv = 8 - b_i;
        knt = (ii - b_i) + 7;
        while ((lastv > 0) && (TL[knt] == 0.0F)) {
          lastv--;
          knt--;
        }
        knt = 7 - b_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          coltop = (ii + ((knt - 1) << 3)) + 8;
          ia = coltop;
          do {
            exitg1 = 0;
            if (ia + 1 <= coltop + lastv) {
              if (TL[ia] != 0.0F) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = 0;
      }
      if (lastv > 0) {
        xgemv(lastv, knt, TL, ii + 9, TL, ii + 1, work);
        xgerc(lastv, knt, -tau[b_i], ii + 1, work, TL, ii + 9);
      }
      TL[ii] = atmp;
    } else {
      tau[7] = 0.0F;
    }
  }
  itau = 7;
  for (ia = 0; ia < 8; ia++) {
    for (b_i = 0; b_i <= ia; b_i++) {
      knt = b_i + (ia << 3);
      R[knt] = TL[knt];
    }
    lastv = ia + 2;
    if (lastv <= 8) {
      memset(&R[(ia * 8 + lastv) + -1], 0,
             (uint32_T)(-lastv + 9) * sizeof(real32_T));
    }
    work[ia] = 0.0F;
  }
  for (b_i = 7; b_i >= 0; b_i--) {
    ii = (b_i + (b_i << 3)) + 9;
    if (b_i + 1 < 8) {
      TL[ii - 9] = 1.0F;
      if (tau[itau] != 0.0F) {
        lastv = 8 - b_i;
        knt = ii - b_i;
        while ((lastv > 0) && (TL[knt - 2] == 0.0F)) {
          lastv--;
          knt--;
        }
        knt = 7 - b_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          coltop = ii + ((knt - 1) << 3);
          ia = coltop;
          do {
            exitg1 = 0;
            if (ia <= (coltop + lastv) - 1) {
              if (TL[ia - 1] != 0.0F) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = 0;
      }
      if (lastv > 0) {
        xgemv(lastv, knt, TL, ii, TL, ii - 8, work);
        xgerc(lastv, knt, -tau[itau], ii - 8, work, TL, ii);
      }
      coltop = ii - 7;
      lastv = ii - b_i;
      for (k = coltop; k < lastv; k++) {
        TL[k - 1] *= -tau[itau];
      }
    }
    TL[ii - 9] = 1.0F - tau[itau];
    for (ia = 0; ia < b_i; ia++) {
      TL[(ii - ia) - 10] = 0.0F;
    }
    itau = b_i - 1;
  }
  memcpy(&Q[0], &TL[0], 64U * sizeof(real32_T));
  b_i = 0;
  do {
    exitg1 = 0;
    if (b_i <= (uint8_T)nA - 1) {
      if ((real32_T)fabsf(R[b_i + (b_i << 3)]) < 1.0E-12F) {
        Status = -2.0F;
        exitg1 = 1;
      } else {
        b_i++;
      }
    } else {
      for (b_i = 0; b_i < 8; b_i++) {
        for (ia = 0; ia < 8; ia++) {
          beta1 = 0.0F;
          for (lastv = 0; lastv < 8; lastv++) {
            beta1 += Linv[lastv + (b_i << 3)] * Q[lastv + (ia << 3)];
          }
          TL[b_i + (ia << 3)] = beta1;
        }
      }
      memset(&RLinv[0], 0, 64U * sizeof(real32_T));
      for (ia = nA; ia >= 1; ia--) {
        lastv = (ia - 1) << 3;
        coltop = (ia + lastv) - 1;
        RLinv[coltop] = 1.0F;
        for (k = ia; k <= nA; k++) {
          knt = (ia + ((k - 1) << 3)) - 1;
          RLinv[knt] /= R[coltop];
        }
        if (ia > 1) {
          for (b_i = 0; b_i <= ia - 2; b_i++) {
            for (k = ia; k <= nA; k++) {
              coltop = (k - 1) << 3;
              knt = b_i + coltop;
              RLinv[knt] -= R[b_i + lastv] * RLinv[(ia + coltop) - 1];
            }
          }
        }
      }
      if (nA > 2147483646) {
        knt = MAX_int32_T;
      } else {
        knt = nA + 1;
      }
      for (b_i = 0; b_i < 8; b_i++) {
        for (ia = b_i + 1; ia < 9; ia++) {
          lastv = b_i + ((ia - 1) << 3);
          H[lastv] = 0.0F;
          for (k = knt; k < 9; k++) {
            coltop = (k - 1) << 3;
            H[lastv] -= TL[b_i + coltop] * TL[(ia + coltop) - 1];
          }
          H[(ia + (b_i << 3)) - 1] = H[lastv];
        }
      }
      for (ia = 0; ia < i; ia++) {
        for (b_i = 0; b_i < 8; b_i++) {
          lastv = b_i + (ia << 3);
          D[lastv] = 0.0F;
          for (k = ia + 1; k <= nA; k++) {
            coltop = (k - 1) << 3;
            D[lastv] += TL[b_i + coltop] * RLinv[ia + coltop];
          }
        }
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return Status;
}

/*
 * File trailer for qpkwik.c
 *
 * [EOF]
 */
