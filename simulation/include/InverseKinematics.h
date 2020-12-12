#pragma once
#include "Contacts.h"
#include "Joints.h"
#include "nn_eval.h"
#define N_SM 10
class InverseKinematics
{
public:

	double damp;
	int num_iter;
	VectorXd Xr;
	VectorXd X0;
	VectorXd slack;
	MatrixXd J;
	VectorXd damping;

	MatrixXd sca_antigrad;
	VectorXd sca_lnval;
	VectorXd sca_sz;
	MatrixXd idx_sca;
	VectorXd sca_act;
    VectorXd vec_gamma;
    vector<VectorXd> vec_gamma_grad;		
	VectorXd qmin_l, qmax_l;
	bool sca_verb;

	InverseKinematics();
	~InverseKinematics() {};
	void update_model(Contact_Manager &points, VectorXd q0);
	VectorXd solve_QP(VectorXd &qref, VectorXd &qlow, VectorXd &qup, MatrixXd &svm_antigrad, VectorXd &svm_lnval);
	double return_hand_error();
	double solve(Contact_Manager &points, VectorXd& ref_pos, VectorXd freeze, vector<nn_state> &nn_models, bool &sca_toggle, bool &symmetry);
};
