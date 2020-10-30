#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <random>
#include <ctime>
#include "Model.h"
#include "mesh_utils.h"

using namespace std;
using namespace Eigen;


/* Index reference and a natural posture of icub           */
/* global positions                                        */
/* pelvis pos:     pos[0] pos[1] pos[2]                    */
/* pelvis rot:     pos[3] pos[4] pos[5] pos[38]            */
/*                 // right      // left                   */
/* head                   pos[11]                          */
/* neck roll              pos[10]                          */
/* neck pitch             pos[9]                           */
/* shoulder pitch  pos[31]       pos[24]                   */
/* shoulder roll   pos[32]       pos[25]                   */
/* shoulder yaw    pos[33]       pos[26]                   */
/* elbow           pos[34]       pos[27]                   */
/* forearm yaw     pos[35]       pos[28]                   */
/* forearm roll    pos[36]       pos[29]                   */
/* forearm pitch   pos[37]       pos[30]                   */
/* torso pitch            pos[6]                           */
/* torso roll             pos[7]                           */
/* torso yaw              pos[8]                           */
/* hip pitch       pos[18]       pos[12]                   */
/* hip roll        pos[19]       pos[13]                   */
/* hip yaw         pos[20]       pos[14]                   */
/* knee            pos[21]       pos[15]                   */
/* ankle pitch     pos[22]       pos[16]                   */
/* ankle roll      pos[23]       pos[17]                   */
/*                    R             L                      */
int main(int argc, char *argv[])
{
	// random generator setup
	std::mt19937_64 rng;
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng.seed(ss);
    // initialize a uniform distribution between 0 and 1
    std::uniform_real_distribution<double> unif(0, 1);

    // definitions of icub model
    Model model;
    model.init();

	// joint variables
	Cvector ref_pos  = Cvector::Zero(AIR_N_U+1); ref_pos[AIR_N_U] = 1;
	Cvector ref_pos_scaled  = Cvector::Zero(AIR_N_U+1); ref_pos[AIR_N_U] = 1;
	Cvector ref_vel  = Cvector::Zero(AIR_N_U+1);
    Cvector freezeIK = Cvector::Zero(AIR_N_U);

	model.set_state(0,ref_pos,ref_vel);

	// freeze neck joints
	for(int i=9;i<=11;i++)
    	freezeIK[i] = 1;
	string fp_base = "../meshes/input/full/";
	string fp_idx = argv[1];
	string filepath_in = fp_base+fp_idx;
	//cout << filepath_in << endl;
	Body meshes_zero = read_txt_meshes(filepath_in);
	Body meshes_new;
	string filepath_map = "../meshes/idx_map_c.txt";

	auto collision_map = read_noncol_map(filepath_map);	

	VectorXi idx_map(meshes_zero.size());
	vector<string> idx_bnames(meshes_zero.size());

	for(int i=0;i<meshes_zero.size();i++)
	{
		idx_map[i] = get<0>(meshes_zero[i]);
		idx_bnames[i] = get<3>(meshes_zero[i]);
		//cout << idx_bnames[i] << endl;
	}

	//cout << idx_map.transpose() << endl;
	auto collisions_grid = CollisionsBodyGrid(meshes_zero, collision_map, idx_map);
	vector<fcl::CollisionObjectd*> colObjs(meshes_zero.size());
	colObjs = createCollisionObj(meshes_zero);


	chrono::steady_clock::time_point begin = chrono::steady_clock::now();
	time_t t_start = time(0);

	int n_trials = stoi(argv[2]);
	cout << n_trials <<endl;
	int n_col = 0;
	int n_noncol = 0;

	float d_threshold_close = 0.15; //in meters, if >100, then no distance queries at all.
	float d_threshold_collide = 0.01; //in meters, was 0.03 for candidacy proposal
	// dataset is 50:50 free/collided. 50 free is subdivided, dist_fract - within treshold, rest - far from collisions
	float dist_fract = 0.7;
	int n_distanced = n_trials/2*dist_fract;
	int n_undistanced = n_trials/2  - n_distanced;

	bool ignore_stable;
	if(argc==4)ignore_stable = stoi(argv[3]);
	else ignore_stable = 1;

	bool status;
	if(argc==5)status = stoi(argv[4]);
	else status = 0;
	//cout << status << endl;

	vector<ArrayXd> data;
	vector<string> p_status, c_status;
	stringstream ss_status_p, ss_status_c;
	ArrayXd line(ref_pos.size()+1-7); // +1 for collision state, -7 for pelvis (3 pos & 4 orient)
	
	//ArrayXi rand_joints =  ArrayXi::LinSpaced(14, 24,37);
	ArrayXi rand_joints(32);

	ArrayXi torso_head = ArrayXi::LinSpaced(3,6,8);
	ArrayXi l_leg = ArrayXi::LinSpaced(6,12,17);
	ArrayXi r_leg = ArrayXi::LinSpaced(6,18,23);
	ArrayXi l_arm = ArrayXi::LinSpaced(7,24,30);
	ArrayXi r_arm = ArrayXi::LinSpaced(7,31,37);

	//cout << r_arm.transpose() <<endl;
	switch(stoi(argv[1])) 
	{
    	case 1 : 	rand_joints.resize(14); 	
					rand_joints << l_arm, r_arm; 		
					break;
    	case 2 : 	rand_joints.resize(16); 	
					rand_joints << torso_head, l_leg, l_arm; 		
					break;
    	case 3 : 	rand_joints.resize(16); 	
					rand_joints << torso_head, r_leg, l_arm; 		
					break;
    	case 4 : 	rand_joints.resize(10); 	
					rand_joints << torso_head, l_arm; 
					break;
    	case 5 : 	rand_joints.resize(16); 	
					rand_joints << torso_head, l_leg, r_arm; 		
					break;
    	case 6 : 	rand_joints.resize(16); 	
					rand_joints << torso_head, r_leg, r_arm; 		
					break;
    	case 7 : 	rand_joints.resize(10); 	
					rand_joints << torso_head, r_arm; 	
					break;
    	case 8 : 	rand_joints.resize(12); 	
					rand_joints << r_leg, l_leg; 		
					break;
    	case 9 : 	rand_joints.resize(9); 	 	
					rand_joints << torso_head, l_leg; 	
					break;
    	case 10: 	rand_joints.resize(9); 		
					rand_joints << torso_head, r_leg; 	
					break;
		case 0: 	rand_joints.resize(32); 		
					rand_joints << ArrayXi::LinSpaced(32,6,37); 	
					break;
		//toy example for 2d representation				
		case 11: 	rand_joints.resize(7); 		
					rand_joints << 31,32,33,34,35,36,37; //31,34

					//ref_pos_scaled[32] = 0.1;
					//ref_pos_scaled[33] = 1;
					
					ref_pos_scaled[24] = 0.3; //0.3
					ref_pos_scaled[25] = 0.1; //0.3
					ref_pos_scaled[26] = 1; //1
					ref_pos_scaled[27] = 0.7;

					break;	
		//plot given posture (first line in cycle "k+=1")
		case 12: 	rand_joints.resize(0); 							
					// ref_pos_scaled[24] = 0.5; 
					// ref_pos_scaled[25] = 0.1; 
					// ref_pos_scaled[26] = 0.8; 
					// ref_pos_scaled[27] = 0.7;
					// ref_pos_scaled[28] = 0.5;
					// ref_pos_scaled[29] = 0.8;
					// ref_pos_scaled[30] = 0.4;
					// ref_pos_scaled[31] = 0.2;
					// ref_pos_scaled[32] = 0.1;
					// ref_pos_scaled[33] = 0.8;
					// ref_pos_scaled[34] = 0.99;
					// ref_pos_scaled[35] = 0.5;
					// ref_pos_scaled[36] = 0.8;
					// ref_pos_scaled[37] = 0.01;

					//ref_pos_scaled[25] = 0.1;
					//ref_pos_scaled[32] = 0.1;
					//ref_pos[1] = 1;
					//ref_pos[27] = 1;
					ref_pos <<  -0.0361887 , 0.00191279 ,   0.614406 , -0.0211822, -0.00750377,   0.0205988  ,  0.100705,  -0.0581313,  -0.0547453  , 0.0796597  ,-0.0136359 , -0.0872524  ,  0.293185  , 0.0285984,  -0.0519935 ,  -0.571409 ,  -0.263959 ,-0.00032168  ,  0.230729  ,-0.0454897  , 0.0535228   ,-0.469318  , -0.225087 ,  0.0144255,   -0.700999  ,  0.400857  ,  0.988554   ,  1.84674  , -0.864245  , -0.768919,   0.0300755 ,   -1.58841  ,   0.83356   ,  1.73659    ,  1.7478   , 0.868461 ,   0.161648,   -0.433266,    0.999535;
					cout << ref_pos.transpose() << endl;
					break;				
	}

	bool collision_flag = true;
	bool stable_flag;
	bool dist_flag;
	fcl::CollisionRequestd c_req;
	fcl::CollisionResultd c_res;
	fcl::DistanceRequestd d_req;
	fcl::DistanceResultd d_res;
	d_req.enable_signed_distance = true;
	d_req.enable_nearest_points = true;
	c_req.enable_contact = true;
	float mindist;
	Vector3d CoM_pos;
	// main loop
	int k = 0;
	int g = 0;
	//cout << model.qmin.transpose() << endl << model.qmax.transpose() << endl;
	while(k<n_trials)
	{
		g+=1;
		if(stoi(argv[1]) == 12) k+=1;
		line   = ArrayXd::Zero(line.size()); 

		//create random ref_pos vector
		for(int i=6;i<AIR_N_U;i++)
			if((rand_joints.size()>0) && (rand_joints == i).any()) //randomize only rand_joints	
				//double p = unif(rng);
				//ref_pos[i] = (model.qmin[i-6] + p * (model.qmax[i-6] - model.qmin[i-6]))/180.0*M_PI;
				ref_pos_scaled[i] = unif(rng);	
		//cout << "RefPosScaled:" << ref_pos_scaled.transpose() << endl;

		for(int i=9;i<=11;i++)
    	{
			ref_pos[i] = 0;
			ref_pos_scaled[i] = 0;
		}

		for(int i=6;i<AIR_N_U;i++)	
			if(ref_pos_scaled[i] != 0)
				ref_pos[i] = (model.qmin[i-6] + ref_pos_scaled[i] * (model.qmax[i-6] - model.qmin[i-6]))/180.0*M_PI;
		//cout << "RefPos:" << ref_pos.transpose() << endl;


		model.set_state(0,ref_pos,ref_vel);
		CoM_pos = model.get_cm();
		if(abs(CoM_pos[0])<0.05 && abs(CoM_pos[1])<0.05){
			stable_flag = true;
		}
		else {
			stable_flag = false;
		}
		
		//if (ignore_stable) stable_flag = true;
		if (ignore_stable || stable_flag){
			//cout << CoM_pos.transpose() << " " << stable_flag << endl;
			updateCollisionObj(colObjs, model, idx_map);
			bool c_prev = collision_flag;
			collision_flag = false;
			ss_status_p.str(string());
			ss_status_c.str(string());
			ss_status_p << k <<"\n";
			ss_status_c << "Collided: ";

			for(int i=0;i<idx_map.size();i++)
			{	
				Vector3d T_vec = model.get_pos(idx_map[i],Vector3d(0,0,0));
				Matrix3d R_mat = model.get_orient(idx_map[i]);
				//cout << idx_map[i] << T_vec.transpose() << endl;	


				ss_status_p << idx_map[i] << ", " << T_vec.transpose() << " " << R_mat.row(0) << " " << R_mat.row(1) << " " << R_mat.row(2) << "\n" ;
				//cout << status_ss.str() << endl;
				for(int j=0;j<collisions_grid[i].size();j++)
				{	
					
					int i1 = i;
					int i2 = collisions_grid[i][j];

					c_res.clear();

					fcl::collide(colObjs[i1], colObjs[i2], c_req, c_res);

					if(c_res.isCollision()){
						//cout << " Collision!" << endl;
						collision_flag = true;
						ss_status_c << idx_map[i1] << " " <<  idx_map[i2] << ", ";
						if(!status)break;
					}  
				}
				if(collision_flag && !status)break;
			}
			//cout << endl;
			line.segment(1,32) = ref_pos_scaled.segment(6,32);

			dist_flag = true;
			//check if there exists pair of bodies closer than treshold value
			if(!collision_flag && d_threshold_close < 100)
			{
				dist_flag = false;
				for(int i=0;i<idx_map.size();i++)		
				{
					for(int j=0;j<collisions_grid[i].size();j++)
					{	
						int i1 = i;
						int i2 = collisions_grid[i][j];
						d_res.clear();
						fcl::distance(colObjs[i1], colObjs[i2], d_req, d_res);
						if(d_res.min_distance < d_threshold_close && d_res.min_distance > d_threshold_collide)
						{
							dist_flag = true;
							//cout << d_res.min_distance << endl;
							//break;
						}
						if(d_res.min_distance < d_threshold_collide)
						{
							collision_flag = true;
							ss_status_c << idx_map[i1] << " " <<  idx_map[i2] << ", ";
							//cout << d_res.min_distance << endl;
							break;
						}
					}
					if(collision_flag)break;
				}
			}

			if(collision_flag && n_col < n_trials/2)
			{	
				line[0] = 1;
				data.push_back(line);
				p_status.push_back(ss_status_p.str());
				if(status){
					string buf = ss_status_c.str();
					buf.erase(buf.size() - 2);
					cout << buf <<endl;
					c_status.push_back(buf);
				}
				n_col +=1;
				k = k+1;

				if(k % 100 == 0) 
					cout << "Total points: " << k <<", Collided: " << 1.0*n_col/k << endl;
			}

			if(!collision_flag && n_noncol < n_trials/2)
			{
				if(n_distanced == 0 && n_undistanced > 0.5*(n_trials/2 - n_trials/2*dist_fract)) //if we have collected all needed "close-to-collisions" and have no "far from collision"
				{
					dist_flag = true;
					n_distanced = n_distanced + 1;
				}
				if(dist_flag && n_distanced > 0)
				{
					n_distanced = n_distanced - 1;
					data.push_back(line);
					p_status.push_back(ss_status_p.str());
					c_status.push_back(string("Collided: None"));
					n_noncol +=1;
					k = k+1;
					if(k % 100 == 0) 
						cout << "Total points: " << k <<", Collided: " << 1.0*n_col/k << endl << n_distanced << " " << n_undistanced << endl;

				}
				if(!dist_flag && n_undistanced > 0)
				{
					n_undistanced = n_undistanced - 1;
					data.push_back(line);
					p_status.push_back(ss_status_p.str());
					c_status.push_back(string("Collided: None"));
					n_noncol +=1;
					k = k+1;
					if(k % 100 == 0) 
						cout << "Total points: " << k <<", Collided: " << 1.0*n_col/k << endl << n_distanced << " " << n_undistanced << endl;
				}
				
			}
			//Measure time for k collisions
			//if(k==1000){
			//	chrono::steady_clock::time_point tmp= chrono::steady_clock::now();
			//	cout << "Time collisions (sec) = " << (chrono::duration_cast<chrono::microseconds>(tmp - begin).count()) /1000000.0 <<endl;
			//}
		}
	};
	//cout << data.size() << endl;
	int res_size = data.size();
	//cout << "Collided cases = " << 1.0 * col_n / res_size <<endl;
	chrono::steady_clock::time_point end= chrono::steady_clock::now();
	cout << "Time difference (sec) = " << (chrono::duration_cast<chrono::microseconds>(end - begin).count()) /1000000.0 <<endl;
	cout << "Total samples number = " << g <<endl;


	string filepath_out = "../meshes/output/";
	meshes_new = update_mesh(meshes_zero, model);
    int tmp = write_txt_meshes(meshes_new, filepath_out);
	
    cout << "Meshes written..." << endl;

	string bin_fname = "../data/data_"+fp_idx+"_"+to_string(n_trials/1000)+"k_"+to_string(t_start)+".bin";

	auto file = fstream(bin_fname, ios::out | ios::binary);

	file.write((char*) &res_size, sizeof(int));	

	int lsize = data[0].size();

	file.write((char*) &lsize, sizeof(int));	

	for(int i=0;i<res_size;i++)
	{	
		for(int j=0;j<lsize;j++)
		{				
			float d_f = (float) data[i][j];
			file.write((char*) &d_f, sizeof(float));
		}
	}
    file.close();

	if(status){
		string txt_fname = "../data/data_"+fp_idx+"_"+to_string(n_trials/1000)+"k_"+to_string(t_start)+".txt";
		auto txtfile = fstream(txt_fname, ios::out);
		for(int i=0;i<res_size;i++)
			txtfile << p_status[i] << c_status[i] << "\n" << endl;
		txtfile.close();
	}
    return 0;
}