/* Produced by CVXGEN, 2020-07-15 07:47:46 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"delta", "dq"};
  const int num_var_names = 2;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_1");
  if (xm == NULL) {
    /* Attempt to pull J_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.J_1 or params.J{1}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_1 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_1;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_2");
  if (xm == NULL) {
    /* Attempt to pull J_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.J_2 or params.J{2}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_2 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_2;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_3");
  if (xm == NULL) {
    /* Attempt to pull J_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.J_3 or params.J{3}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_3 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_3;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_4");
  if (xm == NULL) {
    /* Attempt to pull J_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.J_4 or params.J{4}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_4 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_4;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_5");
  if (xm == NULL) {
    /* Attempt to pull J_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.J_5 or params.J{5}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_5 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_5;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_6");
  if (xm == NULL) {
    /* Attempt to pull J_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.J_6 or params.J{6}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_6 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_6;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_7");
  if (xm == NULL) {
    /* Attempt to pull J_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.J_7 or params.J{7}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_7 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_7;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_8");
  if (xm == NULL) {
    /* Attempt to pull J_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.J_8 or params.J{8}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_8 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_8;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_9");
  if (xm == NULL) {
    /* Attempt to pull J_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.J_9 or params.J{9}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_9 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_9;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_10");
  if (xm == NULL) {
    /* Attempt to pull J_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.J_10 or params.J{10}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_10 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_10;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_11");
  if (xm == NULL) {
    /* Attempt to pull J_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.J_11 or params.J{11}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_11 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_11;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_12");
  if (xm == NULL) {
    /* Attempt to pull J_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }
  if (xm == NULL) {
    printf("could not find params.J_12 or params.J{12}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_12 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_12 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_12 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_12;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_13");
  if (xm == NULL) {
    /* Attempt to pull J_13 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 12);
  }
  if (xm == NULL) {
    printf("could not find params.J_13 or params.J{13}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_13 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_13 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_13 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_13 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_13;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_14");
  if (xm == NULL) {
    /* Attempt to pull J_14 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 13);
  }
  if (xm == NULL) {
    printf("could not find params.J_14 or params.J{14}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_14 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_14 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_14 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_14 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_14;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_15");
  if (xm == NULL) {
    /* Attempt to pull J_15 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 14);
  }
  if (xm == NULL) {
    printf("could not find params.J_15 or params.J{15}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_15 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_15 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_15 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_15 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_15;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_16");
  if (xm == NULL) {
    /* Attempt to pull J_16 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 15);
  }
  if (xm == NULL) {
    printf("could not find params.J_16 or params.J{16}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_16 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_16 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_16 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_16 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_16;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_17");
  if (xm == NULL) {
    /* Attempt to pull J_17 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 16);
  }
  if (xm == NULL) {
    printf("could not find params.J_17 or params.J{17}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_17 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_17 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_17 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_17 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_17;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_18");
  if (xm == NULL) {
    /* Attempt to pull J_18 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 17);
  }
  if (xm == NULL) {
    printf("could not find params.J_18 or params.J{18}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_18 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_18 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_18 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_18 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_18;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_19");
  if (xm == NULL) {
    /* Attempt to pull J_19 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 18);
  }
  if (xm == NULL) {
    printf("could not find params.J_19 or params.J{19}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_19 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_19 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_19 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_19 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_19;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_20");
  if (xm == NULL) {
    /* Attempt to pull J_20 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 19);
  }
  if (xm == NULL) {
    printf("could not find params.J_20 or params.J{20}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_20 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_20 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_20 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_20 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_20;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_21");
  if (xm == NULL) {
    /* Attempt to pull J_21 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 20);
  }
  if (xm == NULL) {
    printf("could not find params.J_21 or params.J{21}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_21 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_21 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_21 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_21 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_21;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_22");
  if (xm == NULL) {
    /* Attempt to pull J_22 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 21);
  }
  if (xm == NULL) {
    printf("could not find params.J_22 or params.J{22}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_22 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_22 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_22 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_22 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_22;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_23");
  if (xm == NULL) {
    /* Attempt to pull J_23 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 22);
  }
  if (xm == NULL) {
    printf("could not find params.J_23 or params.J{23}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_23 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_23 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_23 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_23 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_23;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_24");
  if (xm == NULL) {
    /* Attempt to pull J_24 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 23);
  }
  if (xm == NULL) {
    printf("could not find params.J_24 or params.J{24}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_24 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_24 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_24 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_24 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_24;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_25");
  if (xm == NULL) {
    /* Attempt to pull J_25 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 24);
  }
  if (xm == NULL) {
    printf("could not find params.J_25 or params.J{25}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_25 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_25 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_25 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_25 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_25;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_26");
  if (xm == NULL) {
    /* Attempt to pull J_26 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 25);
  }
  if (xm == NULL) {
    printf("could not find params.J_26 or params.J{26}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_26 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_26 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_26 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_26 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_26;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_27");
  if (xm == NULL) {
    /* Attempt to pull J_27 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 26);
  }
  if (xm == NULL) {
    printf("could not find params.J_27 or params.J{27}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_27 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_27 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_27 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_27 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_27;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_28");
  if (xm == NULL) {
    /* Attempt to pull J_28 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 27);
  }
  if (xm == NULL) {
    printf("could not find params.J_28 or params.J{28}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_28 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_28 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_28 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_28 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_28;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_29");
  if (xm == NULL) {
    /* Attempt to pull J_29 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 28);
  }
  if (xm == NULL) {
    printf("could not find params.J_29 or params.J{29}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_29 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_29 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_29 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_29 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_29;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_30");
  if (xm == NULL) {
    /* Attempt to pull J_30 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 29);
  }
  if (xm == NULL) {
    printf("could not find params.J_30 or params.J{30}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_30 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_30 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_30 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_30 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_30;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_31");
  if (xm == NULL) {
    /* Attempt to pull J_31 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 30);
  }
  if (xm == NULL) {
    printf("could not find params.J_31 or params.J{31}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_31 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_31 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_31 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_31 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_31;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_32");
  if (xm == NULL) {
    /* Attempt to pull J_32 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 31);
  }
  if (xm == NULL) {
    printf("could not find params.J_32 or params.J{32}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_32 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_32 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_32 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_32 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_32;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_33");
  if (xm == NULL) {
    /* Attempt to pull J_33 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 32);
  }
  if (xm == NULL) {
    printf("could not find params.J_33 or params.J{33}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_33 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_33 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_33 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_33 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_33;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_34");
  if (xm == NULL) {
    /* Attempt to pull J_34 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 33);
  }
  if (xm == NULL) {
    printf("could not find params.J_34 or params.J{34}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_34 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_34 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_34 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_34 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_34;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_35");
  if (xm == NULL) {
    /* Attempt to pull J_35 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 34);
  }
  if (xm == NULL) {
    printf("could not find params.J_35 or params.J{35}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_35 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_35 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_35 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_35 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_35;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_36");
  if (xm == NULL) {
    /* Attempt to pull J_36 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 35);
  }
  if (xm == NULL) {
    printf("could not find params.J_36 or params.J{36}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_36 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_36 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_36 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_36 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_36;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_37");
  if (xm == NULL) {
    /* Attempt to pull J_37 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 36);
  }
  if (xm == NULL) {
    printf("could not find params.J_37 or params.J{37}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_37 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_37 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_37 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_37 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_37;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "J_38");
  if (xm == NULL) {
    /* Attempt to pull J_38 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "J");
    if (cell != NULL)
      xm = mxGetCell(cell, 37);
  }
  if (xm == NULL) {
    printf("could not find params.J_38 or params.J{38}.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("J_38 must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter J_38 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter J_38 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter J_38 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.J_38;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "damping");
  if (xm == NULL) {
    printf("could not find params.damping.\n");
  } else {
    if (!((mxGetM(xm) == 38) && (mxGetN(xm) == 1))) {
      printf("damping must be size (38,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter damping must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter damping must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter damping must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.damping;
      src = mxGetPr(xm);
      for (i = 0; i < 38; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "dx");
  if (xm == NULL) {
    printf("could not find params.dx.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("dx must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter dx must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter dx must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter dx must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.dx;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "hard");
  if (xm == NULL) {
    printf("could not find params.hard.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("hard must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter hard must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter hard must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter hard must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.hard;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qlow");
  if (xm == NULL) {
    printf("could not find params.qlow.\n");
  } else {
    if (!((mxGetM(xm) == 32) && (mxGetN(xm) == 1))) {
      printf("qlow must be size (32,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qlow must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qlow must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qlow must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qlow;
      src = mxGetPr(xm);
      for (i = 0; i < 32; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qmax");
  if (xm == NULL) {
    printf("could not find params.qmax.\n");
  } else {
    if (!((mxGetM(xm) == 32) && (mxGetN(xm) == 1))) {
      printf("qmax must be size (32,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qmax must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qmax must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qmax must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qmax;
      src = mxGetPr(xm);
      for (i = 0; i < 32; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qmin");
  if (xm == NULL) {
    printf("could not find params.qmin.\n");
  } else {
    if (!((mxGetM(xm) == 32) && (mxGetN(xm) == 1))) {
      printf("qmin must be size (32,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qmin must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qmin must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qmin must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qmin;
      src = mxGetPr(xm);
      for (i = 0; i < 32; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qref");
  if (xm == NULL) {
    printf("could not find params.qref.\n");
  } else {
    if (!((mxGetM(xm) == 38) && (mxGetN(xm) == 1))) {
      printf("qref must be size (38,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qref must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qref must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qref must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qref;
      src = mxGetPr(xm);
      for (i = 0; i < 38; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "qup");
  if (xm == NULL) {
    printf("could not find params.qup.\n");
  } else {
    if (!((mxGetM(xm) == 32) && (mxGetN(xm) == 1))) {
      printf("qup must be size (32,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter qup must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter qup must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter qup must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.qup;
      src = mxGetPr(xm);
      for (i = 0; i < 32; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "slack");
  if (xm == NULL) {
    printf("could not find params.slack.\n");
  } else {
    if (!((mxGetM(xm) == 36) && (mxGetN(xm) == 1))) {
      printf("slack must be size (36,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter slack must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter slack must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter slack must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.slack;
      src = mxGetPr(xm);
      for (i = 0; i < 36; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_1");
  if (xm == NULL) {
    printf("could not find params.svm_val_1.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_1 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_1;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_2");
  if (xm == NULL) {
    printf("could not find params.svm_val_2.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_2 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_2;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_3");
  if (xm == NULL) {
    printf("could not find params.svm_val_3.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_3 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_3;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_4");
  if (xm == NULL) {
    printf("could not find params.svm_val_4.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_4 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_4;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_5");
  if (xm == NULL) {
    printf("could not find params.svm_val_5.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_5 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_5;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_6");
  if (xm == NULL) {
    printf("could not find params.svm_val_6.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_6 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_6;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_7");
  if (xm == NULL) {
    printf("could not find params.svm_val_7.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_7 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_7;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_8");
  if (xm == NULL) {
    printf("could not find params.svm_val_8.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_8 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_8;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_9");
  if (xm == NULL) {
    printf("could not find params.svm_val_9.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_9 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_9;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_val_10");
  if (xm == NULL) {
    printf("could not find params.svm_val_10.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("svm_val_10 must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_val_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_val_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_val_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_val_10;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_1");
  if (xm == NULL) {
    printf("could not find params.svm_vec_1.\n");
  } else {
    if (!((mxGetM(xm) == 14) && (mxGetN(xm) == 1))) {
      printf("svm_vec_1 must be size (14,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_1;
      src = mxGetPr(xm);
      for (i = 0; i < 14; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_2");
  if (xm == NULL) {
    printf("could not find params.svm_vec_2.\n");
  } else {
    if (!((mxGetM(xm) == 16) && (mxGetN(xm) == 1))) {
      printf("svm_vec_2 must be size (16,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_2;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_3");
  if (xm == NULL) {
    printf("could not find params.svm_vec_3.\n");
  } else {
    if (!((mxGetM(xm) == 16) && (mxGetN(xm) == 1))) {
      printf("svm_vec_3 must be size (16,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_3;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_4");
  if (xm == NULL) {
    printf("could not find params.svm_vec_4.\n");
  } else {
    if (!((mxGetM(xm) == 10) && (mxGetN(xm) == 1))) {
      printf("svm_vec_4 must be size (10,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_4;
      src = mxGetPr(xm);
      for (i = 0; i < 10; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_5");
  if (xm == NULL) {
    printf("could not find params.svm_vec_5.\n");
  } else {
    if (!((mxGetM(xm) == 16) && (mxGetN(xm) == 1))) {
      printf("svm_vec_5 must be size (16,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_5;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_6");
  if (xm == NULL) {
    printf("could not find params.svm_vec_6.\n");
  } else {
    if (!((mxGetM(xm) == 16) && (mxGetN(xm) == 1))) {
      printf("svm_vec_6 must be size (16,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_6;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_7");
  if (xm == NULL) {
    printf("could not find params.svm_vec_7.\n");
  } else {
    if (!((mxGetM(xm) == 10) && (mxGetN(xm) == 1))) {
      printf("svm_vec_7 must be size (10,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_7;
      src = mxGetPr(xm);
      for (i = 0; i < 10; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_8");
  if (xm == NULL) {
    printf("could not find params.svm_vec_8.\n");
  } else {
    if (!((mxGetM(xm) == 12) && (mxGetN(xm) == 1))) {
      printf("svm_vec_8 must be size (12,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_8;
      src = mxGetPr(xm);
      for (i = 0; i < 12; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_9");
  if (xm == NULL) {
    printf("could not find params.svm_vec_9.\n");
  } else {
    if (!((mxGetM(xm) == 9) && (mxGetN(xm) == 1))) {
      printf("svm_vec_9 must be size (9,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_9;
      src = mxGetPr(xm);
      for (i = 0; i < 9; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "svm_vec_10");
  if (xm == NULL) {
    printf("could not find params.svm_vec_10.\n");
  } else {
    if (!((mxGetM(xm) == 9) && (mxGetN(xm) == 1))) {
      printf("svm_vec_10 must be size (9,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter svm_vec_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter svm_vec_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter svm_vec_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.svm_vec_10;
      src = mxGetPr(xm);
      for (i = 0; i < 9; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 67) {
    printf("Error: %d parameters are invalid.\n", 67 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 38; i++)
      printf("  params.damping[%d] = %.6g;\n", i, params.damping[i]);
    for (i = 0; i < 38; i++)
      printf("  params.qref[%d] = %.6g;\n", i, params.qref[i]);
    for (i = 0; i < 36; i++)
      printf("  params.slack[%d] = %.6g;\n", i, params.slack[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_1[%d] = %.6g;\n", i, params.J_1[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_2[%d] = %.6g;\n", i, params.J_2[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_3[%d] = %.6g;\n", i, params.J_3[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_4[%d] = %.6g;\n", i, params.J_4[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_5[%d] = %.6g;\n", i, params.J_5[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_6[%d] = %.6g;\n", i, params.J_6[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_7[%d] = %.6g;\n", i, params.J_7[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_8[%d] = %.6g;\n", i, params.J_8[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_9[%d] = %.6g;\n", i, params.J_9[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_10[%d] = %.6g;\n", i, params.J_10[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_11[%d] = %.6g;\n", i, params.J_11[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_12[%d] = %.6g;\n", i, params.J_12[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_13[%d] = %.6g;\n", i, params.J_13[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_14[%d] = %.6g;\n", i, params.J_14[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_15[%d] = %.6g;\n", i, params.J_15[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_16[%d] = %.6g;\n", i, params.J_16[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_17[%d] = %.6g;\n", i, params.J_17[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_18[%d] = %.6g;\n", i, params.J_18[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_19[%d] = %.6g;\n", i, params.J_19[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_20[%d] = %.6g;\n", i, params.J_20[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_21[%d] = %.6g;\n", i, params.J_21[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_22[%d] = %.6g;\n", i, params.J_22[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_23[%d] = %.6g;\n", i, params.J_23[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_24[%d] = %.6g;\n", i, params.J_24[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_25[%d] = %.6g;\n", i, params.J_25[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_26[%d] = %.6g;\n", i, params.J_26[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_27[%d] = %.6g;\n", i, params.J_27[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_28[%d] = %.6g;\n", i, params.J_28[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_29[%d] = %.6g;\n", i, params.J_29[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_30[%d] = %.6g;\n", i, params.J_30[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_31[%d] = %.6g;\n", i, params.J_31[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_32[%d] = %.6g;\n", i, params.J_32[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_33[%d] = %.6g;\n", i, params.J_33[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_34[%d] = %.6g;\n", i, params.J_34[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_35[%d] = %.6g;\n", i, params.J_35[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_36[%d] = %.6g;\n", i, params.J_36[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_37[%d] = %.6g;\n", i, params.J_37[i]);
    for (i = 0; i < 36; i++)
      printf("  params.J_38[%d] = %.6g;\n", i, params.J_38[i]);
    for (i = 0; i < 36; i++)
      printf("  params.dx[%d] = %.6g;\n", i, params.dx[i]);
    for (i = 0; i < 36; i++)
      printf("  params.hard[%d] = %.6g;\n", i, params.hard[i]);
    for (i = 0; i < 32; i++)
      printf("  params.qlow[%d] = %.6g;\n", i, params.qlow[i]);
    for (i = 0; i < 32; i++)
      printf("  params.qup[%d] = %.6g;\n", i, params.qup[i]);
    for (i = 0; i < 14; i++)
      printf("  params.svm_vec_1[%d] = %.6g;\n", i, params.svm_vec_1[i]);
    for (i = 0; i < 32; i++)
      printf("  params.qmax[%d] = %.6g;\n", i, params.qmax[i]);
    for (i = 0; i < 32; i++)
      printf("  params.qmin[%d] = %.6g;\n", i, params.qmin[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_1[%d] = %.6g;\n", i, params.svm_val_1[i]);
    for (i = 0; i < 16; i++)
      printf("  params.svm_vec_2[%d] = %.6g;\n", i, params.svm_vec_2[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_2[%d] = %.6g;\n", i, params.svm_val_2[i]);
    for (i = 0; i < 16; i++)
      printf("  params.svm_vec_3[%d] = %.6g;\n", i, params.svm_vec_3[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_3[%d] = %.6g;\n", i, params.svm_val_3[i]);
    for (i = 0; i < 10; i++)
      printf("  params.svm_vec_4[%d] = %.6g;\n", i, params.svm_vec_4[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_4[%d] = %.6g;\n", i, params.svm_val_4[i]);
    for (i = 0; i < 16; i++)
      printf("  params.svm_vec_5[%d] = %.6g;\n", i, params.svm_vec_5[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_5[%d] = %.6g;\n", i, params.svm_val_5[i]);
    for (i = 0; i < 16; i++)
      printf("  params.svm_vec_6[%d] = %.6g;\n", i, params.svm_vec_6[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_6[%d] = %.6g;\n", i, params.svm_val_6[i]);
    for (i = 0; i < 10; i++)
      printf("  params.svm_vec_7[%d] = %.6g;\n", i, params.svm_vec_7[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_7[%d] = %.6g;\n", i, params.svm_val_7[i]);
    for (i = 0; i < 12; i++)
      printf("  params.svm_vec_8[%d] = %.6g;\n", i, params.svm_vec_8[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_8[%d] = %.6g;\n", i, params.svm_val_8[i]);
    for (i = 0; i < 9; i++)
      printf("  params.svm_vec_9[%d] = %.6g;\n", i, params.svm_vec_9[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_9[%d] = %.6g;\n", i, params.svm_val_9[i]);
    for (i = 0; i < 9; i++)
      printf("  params.svm_vec_10[%d] = %.6g;\n", i, params.svm_vec_10[i]);
    for (i = 0; i < 1; i++)
      printf("  params.svm_val_10[%d] = %.6g;\n", i, params.svm_val_10[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  xm = mxCreateDoubleMatrix(36, 1, mxREAL);
  mxSetField(plhs[0], 0, "delta", xm);
  dest = mxGetPr(xm);
  src = vars.delta;
  for (i = 0; i < 36; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(38, 1, mxREAL);
  mxSetField(plhs[0], 0, "dq", xm);
  dest = mxGetPr(xm);
  src = vars.dq;
  for (i = 0; i < 38; i++) {
    *dest++ = *src++;
  }
}
