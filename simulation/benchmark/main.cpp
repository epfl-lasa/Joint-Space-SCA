#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <random>
#include "InverseKinematics.h"
#include "Contacts.h"
#include "Joints.h"
#include <MeshUtils.h>
#include "Collision.h"
#include "jac_sca.h"

using namespace std;
using namespace Eigen;

# define des_com vectorbig(Vector3d(0.0, 0.0, 0.53), zero_quat)
# define des_obj vectorbig(Vector3d(0.25, 0.0, 0.65), zero_quat)
# define des_lf vectorbig(Vector3d(0.0, 0.075, 0.0), zero_quat)
# define des_rf vectorbig(Vector3d(0.0,-0.075, 0.0), zero_quat)


int main(int argc, char *argv[])
{	
	// random generator setup
	std::mt19937_64 rng;
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng.seed(ss);
    // initialize a uniform distribution between 0 and 1
    std::uniform_real_distribution<double> unif(0, 1);

    std::cout << "Hello World!" << endl;
	// inverse kinematics
	InverseKinematics IK;
    IK.num_iter = 15;
    IK.damp = 0.05;
	Collision collision;
    jac_sca jac_sca;
    bool symmetry = true;
    bool sca_toggle = true;


	// contact definition
	Contact_Manager points;
	// joint variables
	Joints joints;

	// set entire body to position-direct mode
	for(int i=6;i<AIR_N_U;i++) 
		joints.mode[i] = 1;

    // disable the neck joints
	for(int i=9;i<12;i++)
		joints.freeze[i] = 1;


	vector<nn_state> nn_models;
	for(int i =0;i<10;i++){
		nn_models.push_back(nn_model_state_load("../../dependencies/NN_models/0cm/nn_py"+ std::to_string(i+1) +".dat"));
	}
	//use symmetric submodels instead of separately trained
	if(symmetry)
	{
		nn_models[5] = nn_models[1];
		nn_models[4] = nn_models[2];
		nn_models[6] = nn_models[3];
		nn_models[9] = nn_models[8];
	}


	// // task flexibility in IK, -1 means hard constraint
	// // positions
	// points[CN_CM].T.slack_cost = Vector3d(-1,-1,0.1);
	// points[CN_LF].T.slack_cost = Vector3d(-1,-1,-1);
	// points[CN_RF].T.slack_cost = Vector3d(-1,-1,-1);
	// points[CN_LH].T.slack_cost = 1*Vector3d(1,1,1);
	// points[CN_RH].T.slack_cost = 1*Vector3d(1,1,1);
	// // orientations
	// points[CN_CM].R.slack_cost = Vector3d(0.01,0.01,0.01);
	// points[CN_TO].R.slack_cost = Vector3d(0.01,0.01,0.01);
	// points[CN_HD].R.slack_cost = Vector3d(0.01,0.01,0.01);
	// points[CN_LF].R.slack_cost = Vector3d(-1,-1,-1);
	// points[CN_RF].R.slack_cost = Vector3d(-1,-1,-1);
	// hands orientation doesnt matter for collision avoidance
	 points[CN_LH].R.slack_cost = 1e-5*Vector3d(1,1,1);
	 points[CN_RH].R.slack_cost = 1e-5*Vector3d(1,1,1);


	chrono::steady_clock::time_point begin = chrono::steady_clock::now();

	bool collision_flag = false;
	// main loop
	int n_col = 0;
	int n_total = 10;
	float mean_err = 0;
    VectorXd t_eval_arr(n_total);
	for(int i=0; i<n_total; i++)
	{	
		Vector3d boxpos = Vector3d(unif(rng)*0.2+0.05, -0.25+unif(rng)*0.25, 0+unif(rng)*0.2);
		//Vector3d boxpos = Vector3d(0.35, 0, 0.15);
		double obj_width = 0.2;
		//cout << boxpos.transpose() << endl;
		//cout << "\n LH desired pos:" << points[CN_LH].ref_p.pos.transpose() << "\n RH desired pos: " << points[CN_RH].ref_p.pos.transpose() << endl;
        
        Vector3d lh_pos = boxpos + Vector3d(0,obj_width/2.0,0);
        VectorXd lh_ang = ang2quat(Vector3d(0,-M_PI/2,0));

        Vector3d rh_pos = boxpos + Vector3d(0,-obj_width/2.0,0);
        VectorXd rh_ang = ang2quat(Vector3d(0,-M_PI/2,0));

        VectorXd lh = vectorbig(lh_pos,lh_ang);
        VectorXd rh = vectorbig(rh_pos,rh_ang);

        points.load_tasks(des_com, des_lf, des_rf, vectorbig(lh, rh));

		points.update_kinematics();	
		// whole-body IK ////////////////////////////////////////////////////////////////////////////////
		joints.ref_pos  = VectorXd::Zero(AIR_N_U+1); joints.ref_pos[AIR_N_U] = 1;
        double IK_time = IK.solve(points, joints.ref_pos, joints.freeze, nn_models, sca_toggle, symmetry);
        t_eval_arr[i] = IK_time;
		points.model.set_state(joints.ref_pos, 0*joints.sens_vel);
		points.update_kinematics();

		double LH_error = (points[CN_LH].ref_p.pos.segment(0,3) - points[CN_LH].p.pos.segment(0,3)).norm();
		double RH_error = (points[CN_RH].ref_p.pos.segment(0,3) - points[CN_RH].p.pos.segment(0,3)).norm();
		double LF_error = (points[CN_LF].ref_p.pos.segment(0,3) - points[CN_LF].p.pos.segment(0,3)).norm();
		double RF_error = (points[CN_RF].ref_p.pos.segment(0,3) - points[CN_RF].p.pos.segment(0,3)).norm();
		//cout << "LH desired pos:" << points[CN_LH].ref_p.pos.transpose() << "\nRH desired pos: " << points[CN_RH].ref_p.pos.transpose() << endl;
        //cout << "LH act pos:" << points[CN_LH].p.pos.transpose() << "\nRH act pos: " << points[CN_RH].p.pos.transpose() << endl;

        //cout << "\n" << "Errors: " <<LH_error << ", " << RH_error << ", " << LF_error << ", " << RF_error <<"; " << endl;
		bool exit = false;
		mean_err += LH_error + RH_error;
        collision_flag = false;
        //model.set_state(0,ref_pos,ref_vel);
        double mindist = jac_sca.calc_mindist(points);
        vector<string> collided_obj = collision.check(points);
        if(collided_obj.size() > 0)
        {
            n_col +=1;
            collision_flag = true;
            //for(int i = 0;i<collided_obj.size();i++) cout << collided_obj[i] <<" ";
            //cout << endl;
        }
		//cout << "\n" << ref_pos.segment(0,38).transpose() << endl;
		cout << "i = " << i <<", Collision: " << collision_flag <<", Mindist: " << mindist <<endl;
	};	
	cout << "Collision rate = " << 1.0*n_col/n_total << "; Mean error = " << mean_err/n_total <<endl;
	chrono::steady_clock::time_point end= chrono::steady_clock::now();
	cout << "Time difference (sec) = " << (chrono::duration_cast<chrono::microseconds>(end - begin).count()) /1000000.0 << endl;
    cout << "Mean IK time = " << std::setprecision(5) << 1000*t_eval_arr.mean() << "ms" << endl;

    Body meshes_zero = read_txt_meshes("../../dependencies/collision/meshes/input/full/0");
	Body meshes_new;
    string filepath_out = "../../dependencies/collision/meshes/output/";
	meshes_new = update_mesh(meshes_zero, points.model);
    int tmp = write_txt_meshes(meshes_new, filepath_out);
    cout << "Meshes written..." << endl;


    return 0;
}