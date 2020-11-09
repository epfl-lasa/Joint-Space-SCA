/* Produced by CVXGEN, 2020-07-15 07:48:49 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "IKCG_solver.h"
double IKCG_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 74; i++)
    gap += IKCG_work.z[i]*IKCG_work.s[i];
  return gap;
}
void IKCG_set_defaults(void) {
  IKCG_settings.resid_tol = 1e-6;
  IKCG_settings.eps = 1e-4;
  IKCG_settings.max_iters = 25;
  IKCG_settings.refine_steps = 1;
  IKCG_settings.s_init = 1;
  IKCG_settings.z_init = 1;
  IKCG_settings.debug = 0;
  IKCG_settings.verbose = 1;
  IKCG_settings.verbose_refinement = 0;
  IKCG_settings.better_start = 1;
  IKCG_settings.kkt_reg = 1e-7;
}
void IKCG_setup_pointers(void) {
  IKCG_work.y = IKCG_work.x + 74;
  IKCG_work.s = IKCG_work.x + 110;
  IKCG_work.z = IKCG_work.x + 184;
  IKCG_vars.delta = IKCG_work.x + 0;
  IKCG_vars.dq = IKCG_work.x + 36;
}
void setup_indexed_IKCG_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  IKCG_params.J[1] = IKCG_params.J_1;
  IKCG_params.J[2] = IKCG_params.J_2;
  IKCG_params.J[3] = IKCG_params.J_3;
  IKCG_params.J[4] = IKCG_params.J_4;
  IKCG_params.J[5] = IKCG_params.J_5;
  IKCG_params.J[6] = IKCG_params.J_6;
  IKCG_params.J[7] = IKCG_params.J_7;
  IKCG_params.J[8] = IKCG_params.J_8;
  IKCG_params.J[9] = IKCG_params.J_9;
  IKCG_params.J[10] = IKCG_params.J_10;
  IKCG_params.J[11] = IKCG_params.J_11;
  IKCG_params.J[12] = IKCG_params.J_12;
  IKCG_params.J[13] = IKCG_params.J_13;
  IKCG_params.J[14] = IKCG_params.J_14;
  IKCG_params.J[15] = IKCG_params.J_15;
  IKCG_params.J[16] = IKCG_params.J_16;
  IKCG_params.J[17] = IKCG_params.J_17;
  IKCG_params.J[18] = IKCG_params.J_18;
  IKCG_params.J[19] = IKCG_params.J_19;
  IKCG_params.J[20] = IKCG_params.J_20;
  IKCG_params.J[21] = IKCG_params.J_21;
  IKCG_params.J[22] = IKCG_params.J_22;
  IKCG_params.J[23] = IKCG_params.J_23;
  IKCG_params.J[24] = IKCG_params.J_24;
  IKCG_params.J[25] = IKCG_params.J_25;
  IKCG_params.J[26] = IKCG_params.J_26;
  IKCG_params.J[27] = IKCG_params.J_27;
  IKCG_params.J[28] = IKCG_params.J_28;
  IKCG_params.J[29] = IKCG_params.J_29;
  IKCG_params.J[30] = IKCG_params.J_30;
  IKCG_params.J[31] = IKCG_params.J_31;
  IKCG_params.J[32] = IKCG_params.J_32;
  IKCG_params.J[33] = IKCG_params.J_33;
  IKCG_params.J[34] = IKCG_params.J_34;
  IKCG_params.J[35] = IKCG_params.J_35;
  IKCG_params.J[36] = IKCG_params.J_36;
  IKCG_params.J[37] = IKCG_params.J_37;
  IKCG_params.J[38] = IKCG_params.J_38;
}
void IKCG_setup_indexing(void) {
  IKCG_setup_pointers();
  setup_indexed_IKCG_params();
}
void IKCG_set_start(void) {
  int i;
  for (i = 0; i < 74; i++)
    IKCG_work.x[i] = 0;
  for (i = 0; i < 36; i++)
    IKCG_work.y[i] = 0;
  for (i = 0; i < 74; i++)
    IKCG_work.s[i] = (IKCG_work.h[i] > 0) ? IKCG_work.h[i] : IKCG_settings.s_init;
  for (i = 0; i < 74; i++)
    IKCG_work.z[i] = IKCG_settings.z_init;
}
double IKCG_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in IKCG_work.rhs. */
  IKCG_multbyP(IKCG_work.rhs, IKCG_work.x);
  objv = 0;
  for (i = 0; i < 74; i++)
    objv += IKCG_work.x[i]*IKCG_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 74; i++)
    objv += IKCG_work.q[i]*IKCG_work.x[i];
  objv += IKCG_work.quad_568441778176[0];
  return objv;
}
void IKCG_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = IKCG_work.rhs;
  r2 = IKCG_work.rhs + 74;
  r3 = IKCG_work.rhs + 148;
  r4 = IKCG_work.rhs + 222;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  IKCG_multbymAT(r1, IKCG_work.y);
  IKCG_multbymGT(IKCG_work.buffer, IKCG_work.z);
  for (i = 0; i < 74; i++)
    r1[i] += IKCG_work.buffer[i];
  IKCG_multbyP(IKCG_work.buffer, IKCG_work.x);
  for (i = 0; i < 74; i++)
    r1[i] -= IKCG_work.buffer[i] + IKCG_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 74; i++)
    r2[i] = -IKCG_work.z[i];
  /* r3 = -Gx - s + h. */
  IKCG_multbymG(r3, IKCG_work.x);
  for (i = 0; i < 74; i++)
    r3[i] += -IKCG_work.s[i] + IKCG_work.h[i];
  /* r4 = -Ax + b. */
  IKCG_multbymA(r4, IKCG_work.x);
  for (i = 0; i < 36; i++)
    r4[i] += IKCG_work.b[i];
}
void IKCG_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = IKCG_work.rhs + 74;
  ds_aff = IKCG_work.lhs_aff + 74;
  dz_aff = IKCG_work.lhs_aff + 148;
  mu = 0;
  for (i = 0; i < 74; i++)
    mu += IKCG_work.s[i]*IKCG_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 74; i++)
    if (ds_aff[i] < minval*IKCG_work.s[i])
      minval = ds_aff[i]/IKCG_work.s[i];
  for (i = 0; i < 74; i++)
    if (dz_aff[i] < minval*IKCG_work.z[i])
      minval = dz_aff[i]/IKCG_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 74; i++)
    sigma += (IKCG_work.s[i] + alpha*ds_aff[i])*
      (IKCG_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.013513513513513514;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 74; i++)
    IKCG_work.rhs[i] = 0;
  for (i = 148; i < 258; i++)
    IKCG_work.rhs[i] = 0;
  for (i = 0; i < 74; i++)
    r2[i] = IKCG_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void IKCG_refine(double *target, double *var) {
  int i, j;
  double *residual = IKCG_work.buffer;
  double norm2;
  double *new_var = IKCG_work.buffer2;
  for (j = 0; j < IKCG_settings.refine_steps; j++) {
    norm2 = 0;
    IKCG_matrix_multiply(residual, var);
    for (i = 0; i < 258; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (IKCG_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    IKCG_ldl_IKCG_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 258; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (IKCG_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    IKCG_matrix_multiply(residual, var);
    for (i = 0; i < 258; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double IKCG_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  IKCG_multbymG(IKCG_work.buffer, IKCG_work.x);
  /* Add -s + h. */
  for (i = 0; i < 74; i++)
    IKCG_work.buffer[i] += -IKCG_work.s[i] + IKCG_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 74; i++)
    norm2_squared += IKCG_work.buffer[i]*IKCG_work.buffer[i];
  return norm2_squared;
}
double IKCG_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  IKCG_multbymA(IKCG_work.buffer, IKCG_work.x);
  /* Add +b. */
  for (i = 0; i < 36; i++)
    IKCG_work.buffer[i] += IKCG_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 36; i++)
    norm2_squared += IKCG_work.buffer[i]*IKCG_work.buffer[i];
  return norm2_squared;
}
void IKCG_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  IKCG_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 74; i++)
    IKCG_work.s_inv_z[i] = 1;
  IKCG_fill_KKT();
  IKCG_ldl_factor();
  IKCG_fillrhs_start();
  /* Borrow IKCG_work.lhs_aff for the solution. */
  IKCG_ldl_IKCG_solve(IKCG_work.rhs, IKCG_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = IKCG_work.lhs_aff;
  s = IKCG_work.lhs_aff + 74;
  z = IKCG_work.lhs_aff + 148;
  y = IKCG_work.lhs_aff + 222;
  /* Just set x and y as is. */
  for (i = 0; i < 74; i++)
    IKCG_work.x[i] = x[i];
  for (i = 0; i < 36; i++)
    IKCG_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 74; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 74; i++)
      IKCG_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 74; i++)
      IKCG_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 74; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 74; i++)
      IKCG_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 74; i++)
      IKCG_work.z[i] = z[i] + alpha;
  }
}
void IKCG_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = IKCG_work.rhs;
  r2 = IKCG_work.rhs + 74;
  r3 = IKCG_work.rhs + 148;
  r4 = IKCG_work.rhs + 222;
  for (i = 0; i < 74; i++)
    r1[i] = -IKCG_work.q[i];
  for (i = 0; i < 74; i++)
    r2[i] = 0;
  for (i = 0; i < 74; i++)
    r3[i] = IKCG_work.h[i];
  for (i = 0; i < 36; i++)
    r4[i] = IKCG_work.b[i];
}
long IKCG_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  IKCG_work.converged = 0;
  IKCG_setup_pointers();
  IKCG_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (IKCG_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  IKCG_fillq();
  IKCG_fillh();
  IKCG_fillb();
  if (IKCG_settings.better_start)
    IKCG_better_start();
  else
    IKCG_set_start();
  for (iter = 0; iter < IKCG_settings.max_iters; iter++) {
    for (i = 0; i < 74; i++) {
      IKCG_work.s_inv[i] = 1.0 / IKCG_work.s[i];
      IKCG_work.s_inv_z[i] = IKCG_work.s_inv[i]*IKCG_work.z[i];
    }
    IKCG_work.block_33[0] = 0;
    IKCG_fill_KKT();
    IKCG_ldl_factor();
    /* Affine scaling directions. */
    IKCG_fillrhs_aff();
    IKCG_ldl_IKCG_solve(IKCG_work.rhs, IKCG_work.lhs_aff);
    IKCG_refine(IKCG_work.rhs, IKCG_work.lhs_aff);
    /* Centering plus corrector directions. */
    IKCG_fillrhs_cc();
    IKCG_ldl_IKCG_solve(IKCG_work.rhs, IKCG_work.lhs_cc);
    IKCG_refine(IKCG_work.rhs, IKCG_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 258; i++)
      IKCG_work.lhs_aff[i] += IKCG_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = IKCG_work.lhs_aff;
    ds = IKCG_work.lhs_aff + 74;
    dz = IKCG_work.lhs_aff + 148;
    dy = IKCG_work.lhs_aff + 222;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 74; i++)
      if (ds[i] < minval*IKCG_work.s[i])
        minval = ds[i]/IKCG_work.s[i];
    for (i = 0; i < 74; i++)
      if (dz[i] < minval*IKCG_work.z[i])
        minval = dz[i]/IKCG_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 74; i++)
      IKCG_work.x[i] += alpha*dx[i];
    for (i = 0; i < 74; i++)
      IKCG_work.s[i] += alpha*ds[i];
    for (i = 0; i < 74; i++)
      IKCG_work.z[i] += alpha*dz[i];
    for (i = 0; i < 36; i++)
      IKCG_work.y[i] += alpha*dy[i];
    IKCG_work.gap = IKCG_eval_gap();
    IKCG_work.eq_resid_squared = IKCG_calc_eq_resid_squared();
    IKCG_work.ineq_resid_squared = IKCG_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (IKCG_settings.verbose) {
      IKCG_work.optval = IKCG_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, IKCG_work.optval, IKCG_work.gap, sqrt(IKCG_work.eq_resid_squared),
          sqrt(IKCG_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (IKCG_work.gap < IKCG_settings.eps)
        && (IKCG_work.eq_resid_squared <= IKCG_settings.resid_tol*IKCG_settings.resid_tol)
        && (IKCG_work.ineq_resid_squared <= IKCG_settings.resid_tol*IKCG_settings.resid_tol)
       ) {
      IKCG_work.converged = 1;
      IKCG_work.optval = IKCG_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
