/* Produced by CVXGEN, 2020-07-15 07:49:15 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: IKCG_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef IKCG_SOLVER_H
#define IKCG_SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (IKCG_testsolver.c, csolve.c or your own */
/* program) for the global variables IKCG_vars, IKCG_params, IKCG_work and IKCG_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define IKCG_pm(A, m, n) IKCG_printmatrix(#A, A, m, n, 1)
#endif
typedef struct IKCG_Params_t {
  double damping[38];
  double qref[38];
  double slack[36];
  double J_1[36];
  double J_2[36];
  double J_3[36];
  double J_4[36];
  double J_5[36];
  double J_6[36];
  double J_7[36];
  double J_8[36];
  double J_9[36];
  double J_10[36];
  double J_11[36];
  double J_12[36];
  double J_13[36];
  double J_14[36];
  double J_15[36];
  double J_16[36];
  double J_17[36];
  double J_18[36];
  double J_19[36];
  double J_20[36];
  double J_21[36];
  double J_22[36];
  double J_23[36];
  double J_24[36];
  double J_25[36];
  double J_26[36];
  double J_27[36];
  double J_28[36];
  double J_29[36];
  double J_30[36];
  double J_31[36];
  double J_32[36];
  double J_33[36];
  double J_34[36];
  double J_35[36];
  double J_36[36];
  double J_37[36];
  double J_38[36];
  double dx[36];
  double hard[36];
  double qlow[32];
  double qup[32];
  double svm_vec_1[14];
  double qmax[32];
  double qmin[32];
  double svm_val_1[1];
  double svm_vec_2[16];
  double svm_val_2[1];
  double svm_vec_3[16];
  double svm_val_3[1];
  double svm_vec_4[10];
  double svm_val_4[1];
  double svm_vec_5[16];
  double svm_val_5[1];
  double svm_vec_6[16];
  double svm_val_6[1];
  double svm_vec_7[10];
  double svm_val_7[1];
  double svm_vec_8[12];
  double svm_val_8[1];
  double svm_vec_9[9];
  double svm_val_9[1];
  double svm_vec_10[9];
  double svm_val_10[1];
  double *J[39];
} IKCG_Params;
typedef struct IKCG_Vars_t {
  double *dq; /* 38 rows. */
  double *delta; /* 36 rows. */
} IKCG_Vars;
typedef struct IKCG_Workspace_t {
  double h[74];
  double s_inv[74];
  double s_inv_z[74];
  double b[36];
  double q[74];
  double rhs[258];
  double x[258];
  double *s;
  double *z;
  double *y;
  double lhs_aff[258];
  double lhs_cc[258];
  double buffer[258];
  double buffer2[258];
  double KKT[992];
  double L[1212];
  double d[258];
  double v[258];
  double d_inv[258];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_568441778176[1];
  double frac_515683778560;
  double frac_408380502016;
  double frac_28898807808;
  double frac_223435558912;
  double frac_64927649792;
  double frac_531556737024;
  double frac_453527642112;
  double frac_591585001472;
  double frac_409219706880;
  double frac_477442158592;
  double frac_864492830720;
  double frac_114324643840;
  double frac_271917142016;
  double frac_3932532736;
  double frac_932442243072;
  double frac_382803472384;
  double frac_405000867840;
  double frac_516286558208;
  double frac_480281485312;
  double frac_515203338240;
  double frac_24693317632;
  double frac_997301739520;
  double frac_426193444864;
  double frac_880426098688;
  double frac_231466696704;
  double frac_368617824256;
  double frac_563540553728;
  double frac_250229530624;
  double frac_593707646976;
  int converged;
} IKCG_Workspace;
typedef struct IKCG_Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} IKCG_Settings;
extern IKCG_Vars IKCG_vars;
extern IKCG_Params IKCG_params;
extern IKCG_Workspace IKCG_work;
extern IKCG_Settings IKCG_settings;
/* Function definitions in ldl.c: */
void IKCG_ldl_IKCG_solve(double *target, double *var);
void IKCG_ldl_factor(void);
double IKCG_check_factorization(void);
void IKCG_matrix_multiply(double *result, double *source);
double IKCG_check_residual(double *target, double *multiplicand);
void IKCG_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void IKCG_multbymA(double *lhs, double *rhs);
void IKCG_multbymAT(double *lhs, double *rhs);
void IKCG_multbymG(double *lhs, double *rhs);
void IKCG_multbymGT(double *lhs, double *rhs);
void IKCG_multbyP(double *lhs, double *rhs);
void IKCG_fillq(void);
void IKCG_fillh(void);
void IKCG_fillb(void);
void IKCG_pre_ops(void);

/* Function definitions in solver.c: */
double IKCG_eval_gap(void);
void IKCG_set_defaults(void);
void IKCG_setup_pointers(void);
void setup_indexed_IKCG_params(void);
void IKCG_setup_indexing(void);
void IKCG_set_start(void);
double IKCG_eval_objv(void);
void IKCG_fillrhs_aff(void);
void IKCG_fillrhs_cc(void);
void IKCG_refine(double *target, double *var);
double IKCG_calc_ineq_resid_squared(void);
double IKCG_calc_eq_resid_squared(void);
void IKCG_better_start(void);
void IKCG_fillrhs_start(void);
long IKCG_solve(void);

/* Function definitions in IKCG_testsolver.c: */
int IKCG_main(int argc, char **argv);
void IKCG_load_default_data(void);

/* Function definitions in util.c: */
void IKCG_tic(void);
float IKCG_toc(void);
float IKCG_tocq(void);
void IKCG_printmatrix(char *name, double *A, int m, int n, int sparse);
double IKCG_unif(double lower, double upper);
float IKCG_ran1(long*idum, int reset);
float IKCG_randn_internal(long *idum, int reset);
double IKCG_randn(void);
void IKCG_reset_rand(void);

#endif
