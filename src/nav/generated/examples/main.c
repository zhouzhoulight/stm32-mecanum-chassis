/*
 * File: main.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-10-31 21:26:50
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "nav_fusion_step.h"
#include "nav_fusion_step_terminate.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_2x1_real32_T(real32_T result[2]);

static void argInit_7x1_real32_T(real32_T result[7]);

static void argInit_7x7_real32_T(real32_T result[49]);

static void argInit_9x1_real32_T(real32_T result[9]);

static real32_T argInit_real32_T(void);

/* Function Definitions */
/*
 * Arguments    : real32_T result[2]
 * Return Type  : void
 */
static void argInit_2x1_real32_T(real32_T result[2])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real32_T();
  }
}

/*
 * Arguments    : real32_T result[7]
 * Return Type  : void
 */
static void argInit_7x1_real32_T(real32_T result[7])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 7; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real32_T();
  }
}

/*
 * Arguments    : real32_T result[49]
 * Return Type  : void
 */
static void argInit_7x7_real32_T(real32_T result[49])
{
  int32_T i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 49; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : real32_T result[9]
 * Return Type  : void
 */
static void argInit_9x1_real32_T(real32_T result[9])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 9; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real32_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : real32_T
 */
static real32_T argInit_real32_T(void)
{
  return 0.0F;
}

/*
 * Arguments    : int32_T argc
 *                char **argv
 * Return Type  : int32_T
 */
int32_T main(int32_T argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_nav_fusion_step();
  /* Terminate the application.
You do not need to do this more than one time. */
  nav_fusion_step_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_nav_fusion_step(void)
{
  real32_T P[49];
  real32_T fv[9];
  real32_T x[7];
  real32_T d[4];
  real32_T fv1[2];
  /* Initialize function 'nav_fusion_step' input arguments. */
  /* Initialize function input argument 'x'. */
  /* Initialize function input argument 'P'. */
  /* Initialize function input argument 'u'. */
  /* Initialize function input argument 'geometry'. */
  /* Call the entry-point 'nav_fusion_step'. */
  argInit_7x1_real32_T(x);
  argInit_7x7_real32_T(P);
  argInit_9x1_real32_T(fv);
  argInit_2x1_real32_T(fv1);
  nav_fusion_step(x, P, fv, fv1, d);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
