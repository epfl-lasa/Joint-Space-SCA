/*
Generated 22-Nov-2018 13:14:19 by SD/FAST, Kane's formulation
(sdfast B.2.8 #30123) on machine ID unknown
Copyright (c) 1990-1997 Symbolic Dynamics, Inc.
Copyright (c) 1990-1997 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041
*/
#include <math.h>

/* These routines are passed to icub_air_root. */

void icub_air_posfunc(double vars[38],
    double param[1],
    double resid[38])
{
    int i;
    double pos[39],vel[38];

    for (i = 0; i < 38; i++) {
        vel[i] = 0.;
    }
    icub_air_ang2st(vars,pos);
    icub_air_state(param[0],pos,vel);
    icub_air_umotion(param[0],pos,vel);
    icub_air_perr(resid);
}

void icub_air_velfunc(double vars[38],
    double param[40],
    double resid[38])
{

    icub_air_state(param[39],param,vars);
    icub_air_umotion(param[39],param,vars);
    icub_air_verr(resid);
}

void icub_air_statfunc(double vars[38],
    double param[39],
    double resid[76])
{
    double pos[39],qdotdum[39];

    icub_air_ang2st(vars,pos);
    icub_air_state(param[38],pos,param);
    icub_air_umotion(param[38],pos,param);
    icub_air_uforce(param[38],pos,param);
    icub_air_perr(resid);
    icub_air_deriv(qdotdum,&resid[38]);
}

void icub_air_stdyfunc(double vars[76],
    double param[1],
    double resid[114])
{
    double pos[39],qdotdum[39];

    icub_air_ang2st(vars,pos);
    icub_air_state(param[0],pos,&vars[38]);
    icub_air_umotion(param[0],pos,&vars[38]);
    icub_air_uforce(param[0],pos,&vars[38]);
    icub_air_perr(resid);
    icub_air_verr(&resid[38]);
    icub_air_deriv(qdotdum,&resid[76]);
}

/* This routine is passed to the integrator. */

void icub_air_motfunc(double time,
    double state[77],
    double dstate[77],
    double param[1],
    int *status)
{
    double err[38];
    int i;

    icub_air_state(time,state,&state[39]);
    icub_air_umotion(time,state,&state[39]);
    icub_air_uforce(time,state,&state[39]);
    icub_air_deriv(dstate,&dstate[39]);
    *status = 1;
    icub_air_verr(err);
    for (i = 0; i < 38; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    icub_air_perr(err);
    for (i = 0; i < 38; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

void icub_air_assemble(double time,
    double state[77],
    int lock[38],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[38],param[1];
    int i;
    double jw[1444],dw[11552],rw[608];
    int iw[304],rooterr;

    icub_air_gentime(&i);
    if (i != 131418) {
        icub_air_seterr(50,42);
    }
    param[0] = time;
    icub_air_st2ang(state,state);
    icub_air_root(icub_air_posfunc,state,param,38,38,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,perrs,fcnt,&rooterr);
    icub_air_posfunc(state,param,perrs);
    *fcnt = *fcnt+1;
    icub_air_ang2st(state,state);
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs initial velocity analysis. */

void icub_air_initvel(double time,
    double state[77],
    int lock[38],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[38],param[40];
    int i;
    double jw[1444],dw[11552],rw[608];
    int iw[304],rooterr;

    icub_air_gentime(&i);
    if (i != 131418) {
        icub_air_seterr(51,42);
    }
    for (i = 0; i < 39; i++) {
        param[i] = state[i];
    }
    param[39] = time;
    icub_air_root(icub_air_velfunc,&
      state[39],param,38,38,0,lock,tol,tol,maxevals,jw,dw,rw,iw,verrs,fcnt,&
      rooterr);
    icub_air_velfunc(&state[39],param,verrs);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs static analysis. */

void icub_air_static(double time,
    double state[77],
    int lock[38],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[76],param[39],jw[2888],dw[25992],rw[874];
    int iw[456],rooterr,i;

    icub_air_gentime(&i);
    if (i != 131418) {
        icub_air_seterr(52,42);
    }
    for (i = 0; i < 38; i++) {
        param[i] = state[39+i];
    }
    param[38] = time;
    icub_air_st2ang(state,state);
    icub_air_root(icub_air_statfunc,state,param,76,38,38,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    icub_air_statfunc(state,param,resid);
    *fcnt = *fcnt+1;
    icub_air_ang2st(state,state);
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs steady motion analysis. */

void icub_air_steady(double time,
    double state[77],
    int lock[76],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[114],param[1],vars[76];
    double jw[8664],dw[72200],rw[1482];
    int iw[760],rooterr,i;

    icub_air_gentime(&i);
    if (i != 131418) {
        icub_air_seterr(53,42);
    }
    param[0] = time;
    icub_air_st2ang(state,vars);
    for (i = 0; i < 38; i++) {
        vars[38+i] = state[39+i];
    }
    icub_air_root(icub_air_stdyfunc,vars,param,114,76,38,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    icub_air_stdyfunc(vars,param,resid);
    *fcnt = *fcnt+1;
    icub_air_ang2st(vars,state);
    for (i = 0; i < 38; i++) {
        state[39+i] = vars[38+i];
    }
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs state integration. */

void icub_air_motion(double *time,
    double state[77],
    double dstate[77],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[462],ttime,param[1];
    int vintgerr,which,ferr,i;

    icub_air_gentime(&i);
    if (i != 131418) {
        icub_air_seterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        icub_air_motfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    icub_air_vinteg(icub_air_motfunc,&ttime,state,dstate,param,dt,&step,77,tol,
      work,&vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void icub_air_fmotion(double *time,
    double state[77],
    double dstate[77],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[308],ttime,param[1];
    int ferr,i;

    icub_air_gentime(&i);
    if (i != 131418) {
        icub_air_seterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        icub_air_motfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    icub_air_finteg(icub_air_motfunc,&ttime,state,dstate,param,dt,77,
      work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
