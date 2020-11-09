#include "InverseKinematics.h"
using namespace std;
using namespace Eigen;

#ifdef __cplusplus
	extern "C" {
	#include "IKCG_solver.h"
	}
#endif

IKCG_Vars IKCG_vars;
IKCG_Params IKCG_params;
IKCG_Workspace IKCG_work;
IKCG_Settings IKCG_settings;

InverseKinematics::InverseKinematics()
{
	damp = 0.05;
	num_iter = 15;
	Xr = VectorXd::Zero(N_TASK);
	X0 = VectorXd::Zero(N_TASK);
	slack = VectorXd::Zero(N_TASK);
	J = MatrixXd::Zero(N_TASK, AIR_N_U);
	damping = damp * VectorXd::Ones(AIR_N_U);

	//SCA constants
	sca_antigrad = MatrixXd::Zero(N_SM, AIR_N_U);
	sca_lnval = VectorXd::Zero(N_SM);
	sca_sz = VectorXd::Zero(N_SM);
	sca_sz << 14, 16, 16, 10, 16, 16, 10, 12, 9, 9; //submodels dimensionalities
	//active joints for each submodel
	idx_sca = MatrixXd::Zero(N_SM, 16);
	idx_sca <<  24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 			0, 0,
				 6,  7,  8, 12, 13, 14, 15, 16, 17, 24, 25, 26, 27, 28, 29, 30,
				 6,  7,  8, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
				 6,  7,  8, 24, 25, 26, 27, 28, 29, 30,							0,0,0,0,0,0,
				 6,  7,  8, 12, 13, 14, 15, 16, 17, 31, 32, 33, 34, 35, 36, 37,
				 6,  7,  8, 18, 19, 20, 21, 22, 23, 31, 32, 33, 34, 35, 36, 37,
				 6,  7,  8, 31, 32, 33, 34, 35, 36, 37,							0,0,0,0,0,0,
				12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,					0,0,0,0,
				 6,  7,  8, 12, 13, 14, 15, 16, 17,								0,0,0,0,0,0,0,
				 6,  7,  8, 18, 19, 20, 21, 22, 23,								0,0,0,0,0,0,0;
	vec_gamma = VectorXd::Zero(N_SM);
	for(int i = 0;i<N_SM;i++)
    {
        vec_gamma_grad.push_back(VectorXd::Zero(sca_sz[i]));
    }
	qmin_l = VectorXd::Zero(AIR_N_U-6);
	qmax_l = VectorXd::Zero(AIR_N_U-6);
	//qmin and qmax for learned SCA
	qmin_l <<   -22, -39, -59, -30, -20, -44, 
				-44, -17, -79, -125, -42, -24,
				-44, -17, -79, -125, -42, -24,
				-95,   0, -37,    5, -50, -65, -25, 
				-95,   0, -37,    5, -50, -65, -25;
	qmax_l <<   84,  39,  59,  22,  20,  44,
				132, 119,  79, -5,  21, 24,
				132, 119,  79,  -5,  21,  24,
				5, 161, 100, 106,  50,  10,  25,
				5, 161, 100, 106,  50,  10,  25;
	qmin_l = qmin_l/180.0*M_PI;
	qmax_l = qmax_l/180.0*M_PI;

	sca_act = VectorXd::Zero(N_SM);
	//sca_act << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
	//sca_act << 0, 1, 1, 0, 1, 1, 0, 0, 1, 1;

	sca_verb = true;
}

void InverseKinematics::update_model(Contact_Manager &points, VectorXd q0)
{
	VectorXd dq0 = VectorXd::Zero(AIR_N_U);
	points.model.set_state(q0, dq0);
	int beg_index = 0;

	for(int index=0; index<points.size(); index++)
	{
		int body = points.EE[index].body;
		Vector3d offset = points.EE[index].offset;
		MatrixXd JJ = points.model.get_jacob(body, offset, CT_FULL);

		if(points.EE[index].contact_type==CT_FULL || points.EE[index].contact_type==CT_TRANSLATION)
		{
			X0.segment(beg_index,3)			= points.model.get_pos(body, offset);
			Xr.segment(beg_index,3)			= points.EE[index].ref_p.pos.segment(0,3);
			slack.segment(beg_index,3)		= points.EE[index].T.slack_cost;
			J.block(beg_index,0,3,AIR_N_U) 	= JJ.block(0,0,3,AIR_N_U);
			if(index==0 && points.ifCoM)
			{
				X0.segment(beg_index,3)		= points.model.get_cm();
				J.block(beg_index,0,3,AIR_N_U) = points.model.get_cm_J();
			}
			beg_index += 3;
		}
		if(points.EE[index].contact_type==CT_FULL || points.EE[index].contact_type==CT_ROTATION)
		{
			VectorXd qact 					= quat_sub(points.model.get_orient_quat(body), points.EE[index].ref_p.pos.segment(3,4));
			VectorXd qref 					= ang2quat(Vector3d(0,0,0));
			X0.segment(beg_index,3)			= qact.segment(0,3);
			Xr.segment(beg_index,3)			= qref.segment(0,3);
			slack.segment(beg_index,3)		= points.EE[index].R.slack_cost;
			J.block(beg_index,0,3,AIR_N_U) 	= JJ.block(3,0,3,AIR_N_U);
			beg_index += 3;
		}
	}
}

VectorXd InverseKinematics::solve_QP(VectorXd &qref, VectorXd &qlow, VectorXd &qup, MatrixXd &svm_antigrad, VectorXd &svm_lnval)
{
	//cout << svm_antigrad << endl << svm_lnval << endl;
	VectorXd dX = Xr - X0;

	IKCG_set_defaults();
	IKCG_setup_indexing();

	for(int i=0;i<AIR_N_U;i++)
		memcpy(IKCG_params.J[i+1], &J(0,i),     sizeof(double)*(N_TASK));
		
	memcpy(IKCG_params.qlow,       &qlow[0],    sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.qup,        &qup[0],     sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.dx,         &dX[0],      sizeof(double)*(N_TASK));
	memcpy(IKCG_params.damping,    &damping[0], sizeof(double)*(AIR_N_U));
	memcpy(IKCG_params.qref,       &qref[0],    sizeof(double)*(AIR_N_U));
	memcpy(IKCG_params.slack,      &slack[0],   sizeof(double)*(N_TASK));
	memcpy(IKCG_params.qmin,       &qmin_l[0],  	sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.qmax,       &qmax_l[0],  	sizeof(double)*(AIR_N_U-6));
	memcpy(IKCG_params.svm_vec_1,  &svm_antigrad(1-1,0),	sizeof(double)*(14));
	memcpy(IKCG_params.svm_val_1,  &svm_lnval[0], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_2,  &svm_antigrad(2-1,0),	sizeof(double)*(16));
	memcpy(IKCG_params.svm_val_2,  &svm_lnval[2-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_3,  &svm_antigrad(3-1,0),	sizeof(double)*(16));
	memcpy(IKCG_params.svm_val_3,  &svm_lnval[3-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_4,  &svm_antigrad(4-1,0),	sizeof(double)*(10));
	memcpy(IKCG_params.svm_val_4,  &svm_lnval[4-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_5,  &svm_antigrad(5-1,0),	sizeof(double)*(16));
	memcpy(IKCG_params.svm_val_5,  &svm_lnval[5-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_6,  &svm_antigrad(6-1,0),	sizeof(double)*(16));
	memcpy(IKCG_params.svm_val_6,  &svm_lnval[6-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_7,  &svm_antigrad(7-1,0),	sizeof(double)*(10));
	memcpy(IKCG_params.svm_val_7,  &svm_lnval[7-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_8,  &svm_antigrad(8-1,0),	sizeof(double)*(12));
	memcpy(IKCG_params.svm_val_8,  &svm_lnval[8-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_9,  &svm_antigrad(9-1,0),	sizeof(double)*(9));
	memcpy(IKCG_params.svm_val_9,  &svm_lnval[9-1], 	sizeof(double));
	memcpy(IKCG_params.svm_vec_10,  &svm_antigrad(10-1,0),	sizeof(double)*(9));
	memcpy(IKCG_params.svm_val_10,  &svm_lnval[10-1], 	sizeof(double));
	VectorXd hard = slack;
	for(int i=0; i<hard.size(); i++)
		hard[i] = hard[i]==-1 ? 1 : 0;
	memcpy(IKCG_params.hard, &hard[0], sizeof(double)*(N_TASK));

	IKCG_settings.verbose = false;
	int iter = IKCG_solve();
	VectorXd dq = VectorXd::Zero(AIR_N_U);
	memcpy(&dq[0], IKCG_vars.dq, sizeof(double)*(AIR_N_U));
	return dq;
}

double InverseKinematics::return_hand_error()
{
	// call exactly after solving IK
	return vectorbig((Xr-X0).segment(18,3), (Xr-X0).segment(24,3)).norm();
}

double InverseKinematics::solve(Contact_Manager &points, VectorXd& ref_pos, VectorXd freeze, vector<nn_state> &nn_models, bool &sca_toggle)
{
	timeval start, end;
	gettimeofday(&start, NULL);
	damping = damp * VectorXd::Ones(AIR_N_U);
	for(int m=0;m<num_iter;m++)
	{
		// recalculate gradients
		update_model(points, ref_pos);

		for(int i=0;i<AIR_N_U;i++)
			if(freeze[i]==1)
				J.block(0,i,N_TASK,1) *= 0;

		// avoid singular knee positions
		points.model.qmax[15-6] = -10;
		points.model.qmax[21-6] = -10;
		points.model.qmin[25-6] = 10;
		points.model.qmin[32-6] = 10;

		VectorXd qlow = points.model.qmin/180.0*M_PI - ref_pos.segment(6,AIR_N_U-6);
		VectorXd qup  = points.model.qmax/180.0*M_PI - ref_pos.segment(6,AIR_N_U-6);

		VectorXd qref = VectorXd::Zero(AIR_N_U);
		qref.segment(6,AIR_N_U-6) = - (qup+qlow) * 0.5 * 0;
		qref = qref.cwiseProduct(VectorXd::Ones(AIR_N_U)-freeze);

		//SCA constraints
		if(sca_toggle)
		{
			sca_act << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
			//sca_act << 0, 1, 1, 0, 1, 1, 0, 1, 1, 1;

		}
		else
		{
			sca_act = VectorXd::Zero(N_SM);
		}
		vector<VectorXd> scaled_jpos(N_SM); 
		for(int i=0;i<N_SM;i++)
		{
			scaled_jpos[i] = VectorXd::Zero(sca_sz[i]);
			for(int j=0;j<sca_sz[i];j++)
			{
				int j_idx = idx_sca(i,j);
				scaled_jpos[i][j] = (ref_pos[j_idx]-qmin_l[j_idx-6]) / (qmax_l[j_idx-6] - qmin_l[j_idx-6]);
			}
		}
		#pragma omp parallel for num_threads(4)
		for(int i=0;i<N_SM;i++)
		{
			if(sca_act[i])
			{
				calcGamma_nn_manual(scaled_jpos[i], nn_models[i], vec_gamma[i], vec_gamma_grad[i]);
			}
		} 
		for(int i=0;i<N_SM;i++)
		{
			VectorXd sca_grad = VectorXd::Zero(sca_sz[i]);
			double sca_val = 0;
			if(sca_act[i])
			{
				sca_val = vec_gamma[i];
				sca_grad = vec_gamma_grad[i]; 
				sca_val > 0 ? sca_lnval[i] = std::max(log(sca_val),-10.0) : sca_lnval[i] = -10.0;
				//sca_lnval[i] = std::max(sca_val,-50.0);
				sca_antigrad.block(i,0,1,sca_sz[i]) = -1*sca_grad.transpose();
			}
			else
			{
				sca_lnval[i] = 100000;
				sca_antigrad.block(i,0,1,sca_sz[i]) = -1*sca_grad.transpose();
			}
		}

		if(sca_verb && m==num_iter-1){
			for(int i=0;i<N_SM;i++){
				//cout << std::fixed << std::setprecision(2);
				//cout << i+1 <<": G = " << vec_gamma[i] << endl;
				//cout << "Grad = " << vec_gamma_grad[i].transpose() << endl;
				//cout<< "Jpos = " << scaled_jpos[i].transpose() << endl;
				//cout << std::fixed << std::setprecision(2) << i+1 <<": " << vec_gamma[i] << ", " << sca_lnval[i] << " ";
				cout << std::fixed << std::setprecision(2) << i+1 <<": "<< sca_lnval[i] << "; ";

			}
		cout << endl;
		}

		//cout << sca_lnval.transpose() << endl;
		// solve linearized CVXGEN problem
		VectorXd dq = solve_QP(qref, qlow, qup, sca_antigrad, sca_lnval);
		dq = dq.cwiseProduct(VectorXd::Ones(AIR_N_U)-freeze);

		// apply dq
		VectorXd q0 = getQ(ref_pos);
		ref_pos.segment(0,AIR_N_U) += dq;
		Vector4d w(0,0,0,0);
		w.segment(0,3) = quat2dc(q0) * dq.segment(3,3);
		VectorXd rotBase = quat_mul(quat_exp(0.5 * w), q0);
		ref_pos = setQ(ref_pos,rotBase);
	}


	gettimeofday(&end, NULL);
	return (end.tv_sec-start.tv_sec) + (end.tv_usec-start.tv_usec)/1e6;
}
