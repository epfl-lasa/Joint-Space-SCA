#include "Navigation.h"

Navigation::Navigation()
{

	/* For LPV-DS */
	string path_model  = "../../models/nav-target1-right/";	
	string path_dim    = path_model +  "dimensions";
	string path_Priors = path_model +  "Priors";
	string path_Mu     = path_model +  "Mu";
	string path_Sigma  = path_model +  "Sigma";
	string path_A      = path_model +  "A_k";
	string path_att    = path_model +  "attractor";


    /* Instantiate lpv-DS Model with parameters read from Yaml file via ROS Parameter Server*/
    LPV_DS_.reset(new lpvDS(path_dim.c_str(), path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str(), path_A.c_str()));
	
	/* Read Attractor */
	fileUtils fileUtils_;
	MatrixXd attractor = fileUtils_.readMatrix(path_att.c_str());
	learned_target = attractor.col(0); 


   /* Load orientation map */
   string path_orient_map  = "../../models/nav-target1-right/";


}

Vector3d Navigation::linear_DS(VectorXd target)
{
	// this function takes target position described in robot's frame
	double max_v = 0.3;
	Vector3d q_dot_obj = quat_log(target.segment(3,4)).segment(0,3) * 4;
	Vector3d q_dot_path = Vector3d(0,0,atan2(target[1],target[0]));
	double sigmoid = 1.0/(1.0+exp(-target.segment(0,2).norm()));
	sigmoid = pow(2.0*sigmoid-1.0, 2.0);		
	Vector3d horizontal = Vector3d(target[0], target[1], 0);
	Vector3d crss = horizontal.cross((1.0-sigmoid) * q_dot_obj);
	Vector2d vel = target.segment(0,2);
	vel -= vel.normalized() * 0.3; // to keep a safe distance
	Vector3d x_dot = Vector3d(vel[0]+crss[0], vel[1]+crss[1], q_dot_path[2]);
	return x_dot;
}

Vector3d Navigation::nonlinear_DS(VectorXd target, VectorXd Root)
{
	/* Shift the target of the DS -- which might not be necessary anymore */
	Vector3d root_front = quat2dc(Root.segment(3,4)) * Vector3d(1,0,0);
	Vector3d obj_front = quat2dc(target.segment(3,4)) * Vector3d(1,0,0);
	VectorXd shifted_target = target.segment(0,2) - obj_front.segment(0,2)*0.10;

    /* This function takes current target position described in world frame */
	VectorXd vel = LPV_DS_->compute_f(Root.segment(0,2) - (target.segment(0,2) - learned_target), learned_target);
	cout << "[Norm DS]: " << vel.norm() << endl;
	Vector2d error = target.segment(0,2) - Root.segment(0,2);
    cout << "[Dist to Target]: " << error.norm() << endl;

	/* Compute Desired Orientation! */
	double theta_des = 0.0;
	double theta_act = atan2(root_front[1],root_front[0]);
	if (true)
		// Compute desired theta by aligning to the direction of motion of the DS
		theta_des = atan2(vel[1],vel[0]);								
	else{		
		// Compute desired theta from mapping function learned from demonstrations
		VectorXd gpr_theta(1);
		// gpr_theta = GPR_ORIENT_->regress_GPR(Root.segment(0,2)); 
		theta_des = gpr_theta[0];
		cout << "[GPR-Regressed Orientation]  theta:" << theta_des << endl;
	}			
	double omega = theta_des - theta_act;
	omega += omega<-M_PI ? 2*M_PI : 0;
	omega += omega>M_PI ? -2*M_PI : 0;
	cout << " omega:" <<  omega << endl;

    /* Transform to Robot's Reference frame */ 
	Vector3d x_dot = Vector3d(vel[0], vel[1], 0) * 3.0;
	x_dot = quat2dc(quat_inverse(Root.segment(3,4))) * x_dot;
	x_dot[2] = omega;

	/* If near attractor stop sending velocities */
	if (error.norm() < 0.1){
		cout << "REACH TARGET ----- Sending 0 velocities" << endl;
		x_dot.setZero();
	}

    return x_dot;
}
