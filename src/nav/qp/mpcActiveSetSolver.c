/*
 * File: mpcActiveSetSolver.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 17:04:16
 */

/* Include Files */
#include "mpcActiveSetSolver.h"
#include "minOrMax.h"
#include "norm.h"
#include "qpkwik.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static real32_T rt_roundf_snf(real32_T u);

/* Function Definitions */
/*
 * Arguments    : real32_T u
 * Return Type  : real32_T
 */
static real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if ((real32_T)fabsf(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floorf(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (real32_T)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }
  return y;
}

/*
 * Arguments    : const real32_T f[8]
 *                const real32_T b[32]
 *                boolean_T iA0[32]
 *                real32_T x[8]
 * Return Type  : int32_T
 */
int32_T mpcActiveSetSolver(const real32_T f[8], const real32_T b[32],
                           boolean_T iA0[32], real32_T x[8])
{
  static const real32_T Ac[256] = {
      -1.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 1.0F,  0.0F,
      0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  -1.0F, 1.0F,  -0.0F, -0.0F,
      -0.0F, -0.0F, -0.0F, -0.0F, 1.0F,  -1.0F, 0.0F,  0.0F,  0.0F,  0.0F,
      0.0F,  0.0F,  -0.0F, -1.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
      0.0F,  1.0F,  0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  -0.0F, -1.0F,
      1.0F,  -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0F,  1.0F,  -1.0F, 0.0F,
      0.0F,  0.0F,  0.0F,  0.0F,  -0.0F, -0.0F, -1.0F, -0.0F, -0.0F, -0.0F,
      -0.0F, -0.0F, 0.0F,  0.0F,  1.0F,  0.0F,  0.0F,  0.0F,  0.0F,  0.0F,
      -0.0F, -0.0F, -1.0F, 1.0F,  -0.0F, -0.0F, -0.0F, -0.0F, 0.0F,  0.0F,
      1.0F,  -1.0F, 0.0F,  0.0F,  0.0F,  0.0F,  -0.0F, -0.0F, -0.0F, -1.0F,
      -0.0F, -0.0F, -0.0F, -0.0F, 0.0F,  0.0F,  0.0F,  1.0F,  0.0F,  0.0F,
      0.0F,  0.0F,  -0.0F, -0.0F, -0.0F, -1.0F, 1.0F,  -0.0F, -0.0F, -0.0F,
      0.0F,  0.0F,  0.0F,  1.0F,  -1.0F, 0.0F,  0.0F,  0.0F,  -0.0F, -0.0F,
      -0.0F, -0.0F, -1.0F, -0.0F, -0.0F, -0.0F, 0.0F,  0.0F,  0.0F,  0.0F,
      1.0F,  0.0F,  0.0F,  0.0F,  -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, 1.0F,
      -0.0F, -0.0F, 0.0F,  0.0F,  0.0F,  0.0F,  1.0F,  -1.0F, 0.0F,  0.0F,
      -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, -0.0F, -0.0F, 0.0F,  0.0F,
      0.0F,  0.0F,  0.0F,  1.0F,  0.0F,  0.0F,  -0.0F, -0.0F, -0.0F, -0.0F,
      -0.0F, -1.0F, 1.0F,  -0.0F, 0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  1.0F,
      -1.0F, 0.0F,  -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, -0.0F,
      0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  1.0F,  0.0F,  -0.0F, -0.0F,
      -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, 1.0F,  0.0F,  0.0F,  0.0F,  0.0F,
      0.0F,  0.0F,  1.0F,  -1.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
      -0.0F, -1.0F, 0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  1.0F,
      -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -1.0F, 0.0F,  0.0F,
      0.0F,  0.0F,  0.0F,  0.0F,  0.0F,  1.0F};
  static const real32_T Hinv[64] = {
      0.841829658F,   -0.0407234281F,  -0.168504268F,  -0.141614F,
      -0.0953062177F, -0.0560022295F,  -0.0272553917F, -0.0104660094F,
      -0.0407234281F, 0.853158653F,    -0.0207256842F, -0.148217529F,
      -0.121781111F,  -0.0757922381F,  -0.0380139761F, -0.0152640613F,
      -0.168504268F,  -0.0207256842F,  0.901893497F,   0.024665216F,
      -0.1097835F,    -0.0891225F,     -0.0489516295F, -0.0211946145F,
      -0.141614F,     -0.148217529F,   0.024665216F,   0.9451285F,
      0.0628441721F,  -0.0756400898F,  -0.059591271F,  -0.0297117885F,
      -0.0953062177F, -0.121781111F,   -0.1097835F,    0.0628441721F,
      0.981387F,      0.0980258F,      -0.0425958447F, -0.0354638137F,
      -0.0560022295F, -0.0757922381F,  -0.0891225F,    -0.0756400898F,
      0.0980258F,     1.01876688F,     0.13755922F,    -0.00380166876F,
      -0.0272553917F, -0.0380139761F,  -0.0489516295F, -0.059591271F,
      -0.0425958447F, 0.13755922F,     1.07183444F,    0.229079083F,
      -0.0104660094F, -0.0152640613F,  -0.0211946145F, -0.0297117885F,
      -0.0354638137F, -0.00380166876F, 0.229079083F,   1.37121391F};
  static const real32_T Linv[64] = {
      0.873217F,     -0.0958564207F, -0.19216229F,   -0.145486474F,
      -0.0925369F,   -0.0526134633F, -0.0250893366F, -0.00893775653F,
      0.0F,          0.898876131F,   -0.039572861F,  -0.152487546F,
      -0.117993094F, -0.0710828826F, -0.034883339F,  -0.013035194F,
      0.0F,          0.0F,           0.938640296F,   0.0241231918F,
      -0.105061777F, -0.0830979422F, -0.0446673743F, -0.0180997644F,
      0.0F,          0.0F,           0.0F,           0.96553582F,
      0.0682512522F, -0.0684088543F, -0.0537332259F, -0.0253732558F,
      0.0F,          0.0F,           0.0F,           0.0F,
      0.984173536F,  0.102816306F,   -0.0360708125F, -0.0302853677F,
      0.0F,          0.0F,           0.0F,           0.0F,
      0.0F,          1.00013947F,    0.135931969F,   -0.00324654672F,
      0.0F,          0.0F,           0.0F,           0.0F,
      0.0F,          0.0F,           1.0166434F,     0.195628822F,
      0.0F,          0.0F,           0.0F,           0.0F,
      0.0F,          0.0F,           0.0F,           1.17098844F};
  int32_T iC[32];
  int32_T b_i;
  int32_T exitflag;
  int32_T exitg1;
  int32_T exitg3;
  int32_T i;
  int32_T iSave;
  int32_T k;
  int32_T kDrop;
  int32_T kNext;
  int32_T nA;
  int32_T tmp;
  real32_T D[64];
  real32_T H[64];
  real32_T RLinv[64];
  real32_T U[64];
  real32_T b_b[32];
  real32_T cTol[32];
  real32_T lam[32];
  real32_T Opt[16];
  real32_T Rhs[16];
  real32_T r[8];
  real32_T z[8];
  real32_T Xnorm0;
  real32_T cMin;
  real32_T cVal;
  real32_T rMin;
  real32_T t1;
  int8_T AcRow[8];
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  for (i = 0; i < 32; i++) {
    b_b[i] = -b[i];
  }
  for (i = 0; i < 8; i++) {
    x[i] = 0.0F;
  }
  memset(&lam[0], 0, 32U * sizeof(real32_T));
  exitflag = 1;
  for (i = 0; i < 8; i++) {
    r[i] = 0.0F;
  }
  rMin = 0.0F;
  cTolComputed = false;
  for (i = 0; i < 32; i++) {
    cTol[i] = 1.0F;
    iC[i] = 0;
  }
  nA = 0;
  for (i = 0; i < 32; i++) {
    if (iA0[i]) {
      nA++;
      iC[nA - 1] = i + 1;
    }
  }
  guard1 = false;
  if (nA > 0) {
    memset(&Opt[0], 0, 16U * sizeof(real32_T));
    for (i = 0; i < 8; i++) {
      Rhs[i] = f[i];
      Rhs[i + 8] = 0.0F;
    }
    DualFeasible = false;
    tmp = (int32_T)rt_roundf_snf(0.3F * (real32_T)nA);
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (exitflag <= 50)) {
        cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
        if (cVal < 0.0F) {
          if (ColdReset) {
            exitflag = -2;
            exitg3 = 2;
          } else {
            nA = 0;
            memset(&iC[0], 0, 32U * sizeof(int32_T));
            for (i = 0; i < 32; i++) {
              iA0[i] = false;
            }
            ColdReset = true;
          }
        } else {
          b_i = (uint8_T)nA;
          for (kDrop = 0; kDrop < b_i; kDrop++) {
            Rhs[kDrop + 8] = b_b[iC[kDrop] - 1];
            for (i = kDrop + 1; i <= nA; i++) {
              iSave = (i + (kDrop << 3)) - 1;
              U[iSave] = 0.0F;
              for (k = 0; k < b_i; k++) {
                kNext = k << 3;
                U[iSave] += RLinv[(i + kNext) - 1] * RLinv[kDrop + kNext];
              }
              U[kDrop + ((i - 1) << 3)] = U[iSave];
            }
          }
          for (i = 0; i < 8; i++) {
            cVal = 0.0F;
            for (iSave = 0; iSave < 8; iSave++) {
              cVal += H[i + (iSave << 3)] * Rhs[iSave];
            }
            Opt[i] = cVal;
            for (k = 0; k < b_i; k++) {
              Opt[i] += D[i + (k << 3)] * Rhs[k + 8];
            }
          }
          cMin = -1.0E-12F;
          kDrop = 0;
          for (i = 0; i < b_i; i++) {
            cVal = 0.0F;
            for (iSave = 0; iSave < 8; iSave++) {
              cVal += D[iSave + (i << 3)] * Rhs[iSave];
            }
            Opt[i + 8] = cVal;
            for (k = 0; k < b_i; k++) {
              Opt[i + 8] += U[i + (k << 3)] * Rhs[k + 8];
            }
            cVal = Opt[i + 8];
            lam[iC[i] - 1] = cVal;
            if ((cVal < cMin) && (i + 1 <= nA)) {
              kDrop = i + 1;
              cMin = cVal;
            }
          }
          if (kDrop <= 0) {
            DualFeasible = true;
            for (i = 0; i < 8; i++) {
              x[i] = Opt[i];
            }
          } else {
            exitflag++;
            if (tmp <= 5) {
              i = 5;
            } else {
              i = tmp;
            }
            if (exitflag > i) {
              nA = 0;
              memset(&iC[0], 0, 32U * sizeof(int32_T));
              for (i = 0; i < 32; i++) {
                iA0[i] = false;
              }
              ColdReset = true;
            } else {
              lam[iC[kDrop - 1] - 1] = 0.0F;
              DropConstraint(kDrop, iA0, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lam[0], 0, 32U * sizeof(real32_T));
          for (i = 0; i < 8; i++) {
            cVal = 0.0F;
            for (b_i = 0; b_i < 8; b_i++) {
              cVal += -Hinv[i + (b_i << 3)] * f[b_i];
            }
            x[i] = cVal;
          }
        }
        exitg3 = 1;
      }
    } while (exitg3 == 0);
    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (i = 0; i < 8; i++) {
      cVal = 0.0F;
      for (b_i = 0; b_i < 8; b_i++) {
        cVal += -Hinv[i + (b_i << 3)] * f[b_i];
      }
      x[i] = cVal;
    }
    guard1 = true;
  }
  if (guard1) {
    Xnorm0 = b_norm(x);
    exitg2 = false;
    while ((!exitg2) && (exitflag <= 50)) {
      cMin = -1.0E-6F;
      kNext = -1;
      for (i = 0; i < 32; i++) {
        if (!cTolComputed) {
          for (k = 0; k < 8; k++) {
            z[k] = (real32_T)fabsf(Ac[i + (k << 5)] * x[k]);
          }
          cVal = qp_maximum(z);
          if ((!(cTol[i] >= cVal)) && (!rtIsNaNF(cVal))) {
            cTol[i] = cVal;
          }
        }
        if (!iA0[i]) {
          cVal = 0.0F;
          for (b_i = 0; b_i < 8; b_i++) {
            cVal += Ac[i + (b_i << 5)] * x[b_i];
          }
          cVal = (cVal - b_b[i]) / cTol[i];
          if (cVal < cMin) {
            cMin = cVal;
            kNext = i;
          }
        }
      }
      cTolComputed = true;
      if (kNext + 1 <= 0) {
        exitg2 = true;
      } else if (exitflag == 50) {
        exitflag = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((kNext + 1 > 0) && (exitflag <= 50)) {
            for (b_i = 0; b_i < 8; b_i++) {
              AcRow[b_i] = (int8_T)Ac[kNext + (b_i << 5)];
            }
            guard2 = false;
            if (nA == 0) {
              for (b_i = 0; b_i < 8; b_i++) {
                cVal = 0.0F;
                for (iSave = 0; iSave < 8; iSave++) {
                  cVal += Hinv[b_i + (iSave << 3)] * (real32_T)AcRow[iSave];
                }
                z[b_i] = cVal;
              }
              guard2 = true;
            } else {
              cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
              if (cVal <= 0.0F) {
                exitflag = -2;
                exitg1 = 1;
              } else {
                for (b_i = 0; b_i < 64; b_i++) {
                  U[b_i] = -H[b_i];
                }
                for (b_i = 0; b_i < 8; b_i++) {
                  cVal = 0.0F;
                  for (iSave = 0; iSave < 8; iSave++) {
                    cVal += U[b_i + (iSave << 3)] * (real32_T)AcRow[iSave];
                  }
                  z[b_i] = cVal;
                }
                b_i = (uint8_T)nA;
                for (i = 0; i < b_i; i++) {
                  cVal = 0.0F;
                  for (iSave = 0; iSave < 8; iSave++) {
                    cVal += (real32_T)AcRow[iSave] * D[iSave + (i << 3)];
                  }
                  r[i] = cVal;
                }
                guard2 = true;
              }
            }
            if (guard2) {
              kDrop = 0;
              t1 = 0.0F;
              ColdReset = true;
              DualFeasible = true;
              if (nA > 0) {
                iSave = 0;
                exitg4 = false;
                while ((!exitg4) && (iSave <= (uint8_T)nA - 1)) {
                  if (r[iSave] >= 1.0E-12F) {
                    DualFeasible = false;
                    exitg4 = true;
                  } else {
                    iSave++;
                  }
                }
              }
              if ((nA != 0) && (!DualFeasible)) {
                b_i = (uint8_T)nA;
                for (i = 0; i < b_i; i++) {
                  cVal = r[i];
                  if (cVal > 1.0E-12F) {
                    cVal = lam[iC[i] - 1] / cVal;
                    if ((kDrop == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      kDrop = i + 1;
                    }
                  }
                }
                if (kDrop > 0) {
                  t1 = rMin;
                  ColdReset = false;
                }
              }
              cMin = 0.0F;
              for (k = 0; k < 8; k++) {
                cMin += z[k] * (real32_T)AcRow[k];
              }
              if (cMin <= 0.0F) {
                cVal = 0.0F;
                DualFeasible = true;
              } else {
                cVal = 0.0F;
                for (b_i = 0; b_i < 8; b_i++) {
                  cVal += (real32_T)AcRow[b_i] * x[b_i];
                }
                cVal = (b_b[kNext] - cVal) / cMin;
                DualFeasible = false;
              }
              if (ColdReset && DualFeasible) {
                exitflag = -1;
                exitg1 = 1;
              } else {
                if (DualFeasible) {
                  cMin = t1;
                } else if (ColdReset) {
                  cMin = cVal;
                } else if (t1 < cVal) {
                  cMin = t1;
                } else {
                  cMin = cVal;
                }
                b_i = (uint8_T)nA;
                for (i = 0; i < b_i; i++) {
                  iSave = iC[i];
                  lam[iSave - 1] -= cMin * r[i];
                  if (lam[iSave - 1] < 0.0F) {
                    lam[iSave - 1] = 0.0F;
                  }
                }
                lam[kNext] += cMin;
                if ((real32_T)fabsf(cMin - t1) < 1.1920929E-7F) {
                  DropConstraint(kDrop, iA0, &nA, iC);
                }
                if (!DualFeasible) {
                  for (b_i = 0; b_i < 8; b_i++) {
                    x[b_i] += cMin * z[b_i];
                  }
                  if ((real32_T)fabsf(cMin - cVal) < 1.1920929E-7F) {
                    if (nA == 8) {
                      exitflag = -1;
                      exitg1 = 1;
                    } else {
                      if (nA > 2147483646) {
                        nA = MAX_int32_T;
                      } else {
                        nA++;
                      }
                      iC[nA - 1] = kNext + 1;
                      i = nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (i + 1 > 1)) {
                        b_i = iC[i - 1];
                        if (iC[i] > b_i) {
                          exitg4 = true;
                        } else {
                          iSave = iC[i];
                          iC[i] = b_i;
                          iC[i - 1] = iSave;
                          i--;
                        }
                      }
                      iA0[kNext] = true;
                      kNext = -1;
                      exitflag++;
                    }
                  } else {
                    exitflag++;
                  }
                } else {
                  exitflag++;
                }
              }
            }
          } else {
            cVal = b_norm(x);
            if ((real32_T)fabsf(cVal - Xnorm0) > 0.001F) {
              Xnorm0 = cVal;
              for (k = 0; k < 32; k++) {
                cVal = (real32_T)fabsf(b_b[k]);
                if (cVal >= 1.0F) {
                  cTol[k] = cVal;
                } else {
                  cTol[k] = 1.0F;
                }
              }
              cTolComputed = false;
            }
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
  return exitflag;
}

/*
 * File trailer for mpcActiveSetSolver.c
 *
 * [EOF]
 */
