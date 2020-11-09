#pragma once

#include "Contacts.h"
#include "Object.h"
#include "Walking.h"

class Manipulation
{
public:

	VectorXd bias_lh, bias_rh;
	VectorXd avg_lh, avg_rh;
	VectorXd force_lh, force_rh;
	VectorXd reading_lh, reading_rh;
	VectorXd grav_compns;

	double gamma;
	double dgamma;
	double tau;
	double dtau;
	double orientation_factor;
	MatrixXd A_V;
	MatrixXd A;
	VectorXd xd[2];
	VectorXd xO[2];
	VectorXd xR[2];
	VectorXd xV[2];
	VectorXd qd[2];
	VectorXd qO[2];
	VectorXd qR[2];
	VectorXd qV[2];

	Manipulation();
	~Manipulation() {};
	void set_target_point(VectorXd hands);
	void set_start_point();
	void set_ideal_point(VectorXd hands);
	void read_forces(Contact_Manager &points);
	void initial_filtering(Contact_Manager &points, double dt);
	void secondary_filtering(Contact_Manager &points, double dt);
	void compliance_control(Contact_Manager &points, double force);
	void jacobian_transpose(Contact_Manager &points, Joints &joints, double force);
	void update(bool reachable, double dt, Object * box, bool grasp);
	VectorXd get_hands();
	bool is_at_target(double eps_obj);
	bool is_at_rest(double eps_obj);
	Vector3d modulate(Vector3d dx, Vector3d xR, Object * box);
};