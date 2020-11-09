#pragma once

#include "Contacts.h"
#include "Joints.h"
#include "MedianFilter.h"
#include "InverseKinematics.h"
#include "Wrapper.h"
#include "nn_eval.h"

enum walking_state {IDLE=0, WALK_START, WALK, WALK_STIFF, WALK_STOP};

class Walking
{
public:

	MedianFilter state_filt;
	VectorXd lp3_state, lp3_dstate;
	double shift;
	double fblx, fbrx, fbly, fbry;
	double Tstep;
	double tphase;
	double left_support;
	double ref_vx, ref_vy, ref_w;
	double force_lf, force_rf;
	double al, ar;
	double hip_gain_K;
	double start_time;
	bool hardware;
	walking_state state;
	Vector3d com_adjustment;
	double torso_cost;
	double vx_avg;
	double vy_avg;

	Walking();
	~Walking() {};
	void update(double time, double dt, bool demand, Contact_Manager &points, Joints &joints, Wrapper &wrapper, vector<nn_state> &nn_models);
	void calculate_footstep_adjustments(double time, double dt, Contact_Manager &points, Joints &joints);
	void apply_speed_limits();
	void demand_speeds(Vector3d speeds, double dt);
	void cartesian_tasks(double time, Contact_Manager &points, Joints &joints, vector<nn_state> &nn_models);
	void joint_tasks(double time, double dt, Contact_Manager &points, Joints &joints);
	bool early_phase(double time, double dt);
};
