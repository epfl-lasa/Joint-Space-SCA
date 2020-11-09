#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>


// lpvDS stuff
#include <stdio.h>
#include <fstream>
#include <time.h>
#include "eigen3/Eigen/Dense"

#include "Wrapper.h"
#include "InverseKinematics.h"
#include "Walking.h"
#include "Collision.h"
#include "Navigation.h"
#include "Manipulation.h"
#include "nn_eval.h"

#if defined(ICUBSIM)
	//gazebo libs to highlight collisions in simulation
	#include <gazebo-9/gazebo/gazebo_config.h>
	#include <gazebo-9/gazebo/transport/transport.hh>
	#include <gazebo-9/gazebo/msgs/msgs.hh>
	#include <gazebo-9/gazebo/gazebo_client.hh>
#endif

using namespace std;

enum object_state {EMPTY=0, PICK_APPROACH, PICK, PICK_STAND, HOLD, DROP_APPROACH, DROP, DROP_STAND};
enum trigger {NONE=0, GO, HALT, TAKE, RELEASE};

// robot's default posture
# define des_com vectorbig(Vector3d(0.0, 0.0, 0.53), zero_quat)
# define des_obj vectorbig(Vector3d(0.25, 0.0, 0.65), zero_quat)
# define des_lf vectorbig(Vector3d(0.0, 0.095, 0.0), zero_quat)
# define des_rf vectorbig(Vector3d(0.0,-0.095, 0.0), zero_quat)

bool exit_sim = false;
void my_handler(int s)
{
	cout << "Exit Program Now ... " << endl;
	exit_sim = true;
}

void loadData(Wrapper &wrapper, Contact_Manager &points, Joints &joints)
{
	// read the wrapper and bring everything to the mid-foot frame
	wrapper.readSensors(joints.sens_pos, joints.sens_vel, joints.sens_tau);
	for(int i=0; i<4; i++)
	{
		points[i+1].T.F_sens 	= wrapper.FTsensor[i][0];
		points[i+1].R.F_sens 	= wrapper.FTsensor[i][1];
	}

	// read IMU and remove yaw angle
	double roll = atan2(wrapper.R(2,1), wrapper.R(2,2));
	double pitch = -asin(wrapper.R(2,0));
	MatrixXd rot = ang2dc(Vector3d(0,pitch,0)) * ang2dc(Vector3d(roll,0,0));
	joints.sens_pos					= setQ(joints.sens_pos, dc2quat(rot));
	joints.sens_vel.segment(3,3)	= wrapper.angvel;

	// update the model
	points.model.set_state(joints.sens_pos, joints.sens_vel);

	// bring the mid-foot points to zero and make feet point to +x direction
	Vector3d base 	= points.model.get_pos(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.model.get_pos(points[CN_RF].body, points[CN_RF].offset) * 0.5;
	Vector3d dbase 	= points.model.get_vel(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.model.get_vel(points[CN_RF].body, points[CN_RF].offset) * 0.5;

	// apply transformations
	joints.sens_pos.segment(0,3) 	-= base;
	joints.sens_vel.segment(0,3) 	-= dbase;

	// update the model
	points.model.set_state(joints.sens_pos, joints.sens_vel);
	VectorXd zero_acc = VectorXd::Zero(AIR_N_U);
	points.model.set_acc(zero_acc);
	points.update_kinematics();
}

int main(int argc, char *argv[])
{
	// setup existing condition
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// list of objects sensed from the environment
	Object Box(des_obj);
	Box.name = "BoxWood";
	Box.max_expansion = 0.08;
	Box.max_force = 15;
	//2 commands for picking up the box if it's on the side
	//Box.ideal_grasp_axis = Vector3d(1,0,0);
	//Box.ideal_grasp_rot = ang2quat(Vector3d(0,-M_PI/4,0));
	#if defined(ICUB5) || defined(ICUB33)
		Box.dim = Vector3d(0.16, 0.13, 0.22);
		Box.top_marker = true;
		Box.proximity = 0.2;
	#elif defined(ICUBSIM)
		Box.dim = Vector3d(0.16, 0.13, 0.30); //act height 0.22
		//Box.dim = Vector3d(0.195, 0.10, 0.30); //act height 0.225
		//Box.dim = Vector3d(0.1, 0.05, 0.15); //act height 0.22

		Box.proximity = 0.2;
	#endif

	vector<nn_state> nn_models;
	for(int i =0;i<10;i++){
		nn_models.push_back(nn_model_state_load("../../dependencies/NN_models/0cm/nn_py"+ std::to_string(i+1) +".dat"));
	}

		// YARP Wrapper
	Wrapper wrapper;
	if(wrapper.checkRobot(argc, argv))
		{cout << "Robot name problem" << endl; return 1;}
	wrapper.initialize();
	wrapper.rePID(false);
	// Initialize reading robot/object pos from gazebo/mocap
	#if defined(ICUB5) || defined(ICUB33)
		wrapper.initObject(0, wrapper.robotName + "/get_root_link_WorldPose:o");
		wrapper.initObject(1, "/BoxWood/GraspedObject_WorldPose:o");

		wrapper.Object[0] = des_com;
		wrapper.Object[1] = Box.ideal_pos;
	#elif defined(ICUBSIM)
		wrapper.initObject(0, wrapper.robotName + "/get_root_link_WorldPose:o");
		wrapper.initObject(1, "/BoxWood/GraspedObject_WorldPose:o");
	#endif

	yarp::os::Time::delay(1);

	for(int i=0; i<=1; i++)
		if(wrapper.readObject(i).isZero())
			{
				cout << "Error: Object " << i << " mocap problem!" << endl; 
				return 1;
			}

	// logger
	std::ofstream OutRecord;
	string path_log = "log_icub.txt";
	OutRecord.open(path_log.c_str());

	// inverse kinematics
	InverseKinematics IK;

	Collision collision;
	// gazebo messaging node to highlight collisions in simulation
	#if defined(ICUBSIM)
		gazebo::client::setup();
		gazebo::transport::NodePtr node(new gazebo::transport::Node());
		node->Init();
		gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::GzString_V>("~/collisions/");
		pub->WaitForConnection();
		gazebo::msgs::GzString_V msg;
	#endif
	cout << "init done!" << endl;

	// walking controller
	Walking walking;

	// navigation DS
	Navigation navigation;

	// contact definition
	Contact_Manager points;

	// joint variables
	Joints joints;

	// set entire body to position-direct mode
	for(int i=6;i<AIR_N_U;i++) 
		joints.mode[i] = 1;

	// set arms to force control in simulation
	#if defined(ICUBSIM)
		for(int i=24;i<24+14;i++) 
			joints.mode[i] = 2;
	#endif

	// disable the neck joints
	for(int i=9;i<12;i++)
		joints.freeze[i] = 1;

	// update joint limits
	wrapper.initializeJoint(joints.mode.segment(6,AIR_N_U-6), joints.freeze.segment(6,AIR_N_U-6));
	wrapper.getJointLimits(&points.model.qmin[0], &points.model.qmax[0]);
	points.model.qmax[36-6] -= 10;
	points.model.qmax[29-6] -= 10;
	points.model.qmin[32-6] += 10;
	points.model.qmin[25-6] += 10;

	// read data once
	yarp::os::Time::delay(1);
	loadData(wrapper, points, joints);
	joints.init_pos = joints.sens_pos;

	// Object properties, by default facing forward
	Object Front(des_obj);
	Front.sens_pos = des_obj;
	Front.dim = Vector3d(0.1, 0.1, 0.1);
	Front.max_expansion = 0.1;
	Front.max_force = 0;
	Front.name = "Front";
	//Front.ideal_grasp_axis = Vector3d(1,0,0);
	//Front.ideal_grasp_rot = ang2quat(Vector3d(0,0,0));
	//Front.ideal_pos = vectorbig(Vector3d(0.0, -0.1, 0.65), zero_quat);
	Object Joystick = Front;
	Joystick.name = "Joystick";

	// grasping controller
	Manipulation manip;
	VectorXd Pdrop = des_obj;
	manip.set_ideal_point(Front.get_hand_ideal());
	manip.set_start_point();

	// prepare for reading the keyboard
	nonblock(1);

	// start the loop
	Object *Ptarget = &Joystick;
	double calib_time = 10;
	object_state task = EMPTY;
	trigger signal = NONE;
	bool navigate = false;
	bool follow = false;
	bool user_grasp = false;
	bool user_walk = false;
	bool picking_walk = false;
	bool sca_toggle = false;
	bool verbose = false;
	double t_sca_switch = 0;
	VectorXd pre_sca_switch_pose = joints.com_pos;
	double start_time = wrapper.time;
	while(!exit_sim)
	{
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// sensing //////////////////////////////////////////////////////////////////////////////////////////////////
		double time = wrapper.time - start_time;
		cout << time << "  |   ";
		
		// read sensors
		loadData(wrapper, points, joints);

		// hand force sensor filtering
		if(time<calib_time) 
			manip.initial_filtering(points, wrapper.dt);
		else
			manip.secondary_filtering(points, wrapper.dt);

		// robot position
		VectorXd Root = wrapper.readObject(0);
		// do whatever transform necessary to find root's pos/rot
		#ifdef ICUB5
			Root.segment(0,3) += quat2dc(Root.segment(3,4)) * Vector3d(0.05, -0.05, 0.05);
		#elif defined(ICUB33)
//			Root.segment(0,3) += quat2dc(Root.segment(3,4)) * Vector3d(0.05, -0.05, 0.05);
			//Root.segment(0,3) += quat2dc(Root.segment(3,4)) * Vector3d(-0.08, 0.1, 0.05);
			//Root.segment(3,4) = quat_mul(Root.segment(3,4), ang2quat(Vector3d(0,0,M_PI/2))); 
			Root.segment(0,3) += quat2dc(Root.segment(3,4)) * Vector3d(0.065, 0.053, 0.03);
		#elif defined(ICUBSIM)
			Root.segment(3,4) = quat_mul(Root.segment(3,4), ang2quat(Vector3d(0,0,M_PI))); 
		#endif

		// OutRecord << time << "   " << Root.segment(0,3).transpose() << "  " << 
		// 			quat2ang(Root.segment(3,4)).transpose() << endl;

		// object world positions
		VectorXd Base = points.model.get_trans7(0,zero_v3);

		Box.update_position(Base, points[CN_LF].p.pos, points[CN_RF].p.pos, Root, wrapper.readObject(1));

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// picking state machine ////////////////////////////////////////////////////////////////////////////////////

		// read keyboard signals
		double delta = 0.01;
		if(khbit() != 0)
		{
			char c = fgetc(stdin);
			fflush(stdin);
			switch(c)
			{
				// general signals
				case 't': signal = TAKE; break;
				case 'r': signal = RELEASE; break;
				case 'x': signal = HALT; break;
				case 'n': navigate = !navigate; break;
				case 'f': follow = !follow; break;
				case 'm': user_grasp = !user_grasp; break;
				case 'g': user_walk = !user_walk; break;
				case 'b': sca_toggle =!sca_toggle; t_sca_switch = time;  pre_sca_switch_pose = joints.com_pos; break;
				case 'z': verbose =!verbose; break;

				case '0': Ptarget = &Joystick; break;
				case '1': Ptarget = &Box; break;

				// desired velocities
				case 'w': walking.ref_vx += 0.1; break;
				case 's': walking.ref_vx -= 0.1; break;
				case 'a': walking.ref_vy += 0.1; break;
				case 'd': walking.ref_vy -= 0.1; break;
				case 'q': walking.ref_w += 0.1; break;
				case 'e': walking.ref_w -= 0.1; break;

				case 'c': wrapper.graspLeft(true); wrapper.graspRight(true); break;
				case 'v': wrapper.graspLeft(false); wrapper.graspRight(false); break;

				// desired object position
				case 'i': Joystick.ideal_pos[0] += delta; break;
				case 'k': Joystick.ideal_pos[0] -= delta; break;
				case 'j': Joystick.ideal_pos[1] += delta; break;
				case 'l': Joystick.ideal_pos[1] -= delta; break;
				case 'p': Joystick.ideal_pos[2] += delta; break;
				case ';': Joystick.ideal_pos[2] -= delta; break;
				case 'o': Joystick.ideal_pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0,-delta*5)), Joystick.ideal_pos.segment(3,4)); break;
				case 'u': Joystick.ideal_pos.segment(3,4) = quat_mul(ang2quat(Vector3d(0,0, delta*5)), Joystick.ideal_pos.segment(3,4)); break;
			}
		}

		// state machine
		double eps1	= 0.01 * Ptarget->max_expansion; // if hands reached the point close to the object
		double eps2	= 0.5 * Ptarget->proximity; // if hands reached the point close to the object
		double eps_rest	= 0.2; // if hands reached the default position
		switch(task)
		{
			case EMPTY: 
				if(signal==TAKE)				{signal=NONE; task=PICK_APPROACH;}
				if(signal==HALT)				{signal=NONE; task=EMPTY;} break;
			case PICK_APPROACH:
				if(manip.is_at_target(eps1))	{signal=NONE; task=PICK;}
				if(signal==HALT)				{signal=NONE; task=DROP_STAND;} break;
			case PICK:
				if(manip.is_at_target(eps2) && Ptarget->grow < 0.5)		{signal=NONE; task=PICK_STAND; } 
				if(signal==HALT)				{signal=NONE; task=DROP_STAND;} break;
			case PICK_STAND:
				if(manip.is_at_rest(eps_rest))	{signal=NONE; task=HOLD;} break;
			case HOLD: 
				if(signal==RELEASE)				{signal=NONE; task=DROP_APPROACH;}
				if(signal==HALT)				{signal=NONE; task=HOLD;} break;
			case DROP_APPROACH:
				if(manip.is_at_target(eps1))	{signal=NONE; task=DROP;} 
				if(signal==HALT)				{signal=NONE; task=PICK_STAND;} break;
			case DROP:
				if(Ptarget->grow > 0.9)			{signal=NONE; task=DROP_STAND;}
				if(signal==HALT)				{signal=NONE; task=DROP_STAND;} break;
			case DROP_STAND:
				if(manip.is_at_rest(eps_rest))	{signal=NONE; task=EMPTY;} break;
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// tasks ////////////////////////////////////////////////////////////////////////////////////////////////////

		// define grasping behavior
		bool grasp = task==PICK || task==PICK_STAND || task==HOLD || task==DROP_APPROACH || user_grasp;
		Ptarget->grow += ((double(grasp) ? 0 : 1) - Ptarget->grow) * .5 * wrapper.dt;

		// define hand ideal positions
		VectorXd ideal = Front.get_hand_ideal();
		if(grasp && !Ptarget->sens_pos.segment(0,3).isZero())
			ideal = Ptarget->get_hand_ideal();
		manip.set_ideal_point(ideal);

		// define next hand target positions
		VectorXd target = ideal;
		if(!Ptarget->sens_pos.segment(0,3).isZero())
			if(task==PICK_APPROACH || task==PICK)
				target = Ptarget->get_hand();
			else if(task==DROP_APPROACH || task==DROP)
				target = Ptarget->get_hand_moved(Pdrop);
		manip.set_target_point(target);
		// check feasibility of the object, call collision right after the IK
		// after this point, points.model will be updated with ref_pos in the IK function, not the actual sens_pos
		points.load_tasks(des_com, des_lf, des_rf, target);
		VectorXd copy = joints.Ik_pos0;
		//IK.solve(points, copy, joints.freeze, nn_models);
		bool collision_flag = false;
		// vector<string> collided_obj = collision.check(points);
		// if(collided_obj.size() > 0)
		// {
		// 	collision_flag = true;
		// 	for(int i = 0;i<collided_obj.size();i++) cout << collided_obj[i] <<" ";
		// 	cout << endl;
		// }
		//experimentsb
		// 1:23cm, high, nosca-sca
		// 2:17cm, high, nosca-sca
		// 3: 19cm, low, nosca-sca
		//bool feasibility = walking.state==IDLE && (IK.return_hand_error()< Ptarget->proximity*2.0) && !collision_flag;
		bool feasibility = walking.state==IDLE && !collision_flag;
		double k_dt = 0.25;
		#if defined(ICUB5) || defined(ICUB33)
			k_dt = 0.5;
		#elif defined(ICUBSIM)
			k_dt = 0.5;
		#endif

		manip.update(feasibility, k_dt*wrapper.dt, Ptarget, grasp);
		points.load_tasks(des_com, des_lf, des_rf, manip.get_hands());
		std::cout << std::fixed << std::setprecision(4);
		cout << Ptarget->name << "   |   " << sca_toggle << "   " <<  wrapper.dt << "   " << task << "   " << signal << "   |   " << 
				IK.return_hand_error() << "   " << collision_flag << "    |     ";

		// communication between picking and walking state machines
		if(task==PICK_APPROACH || task==DROP_APPROACH)
			if((IK.return_hand_error()>Ptarget->proximity*2.0) || collision_flag)
				picking_walk = true;
		if((IK.return_hand_error()<Ptarget->proximity) && !collision_flag)
				picking_walk = false;

		//compliant arm control
		double force = (1.0 - Ptarget->grow) * Ptarget->max_force;
		#if defined(ICUB5) || defined(ICUB33)
			if(time>calib_time)
				manip.compliance_control(points, force);
		#elif defined(ICUBSIM)
			manip.jacobian_transpose(points, joints, force);
		#endif
					
		// whole-body IK 
		joints.ref_pos = joints.Ik_pos0;
		
		double ik_time = IK.solve(points, joints.ref_pos, joints.freeze, nn_models, sca_toggle);

		//checking collisions to highlight, publishing them to gazebo node
		#if defined(ICUBSIM)
			vector<string> collided_obj = collision.check(points);
			if (collided_obj.size()==0)
				collided_obj.push_back("empty");
			msg.clear_data();
			for(int i=0;i<collided_obj.size();i++)
				msg.add_data(collided_obj[i]);
			pub->Publish(msg);
		#endif

		// walking task
		if(navigate && walking.state == WALK) 
		{
			// here navigation determines reference walking velocities
			// Vector3d x_dot = navigation.linear_DS(target.segment(0,7)/2.0+target.segment(7,7)/2.0);
			// navigation with learned lpvds
			Vector3d x_dot = navigation.nonlinear_DS(wrapper.readObject(1), Root);
			x_dot[0] *= x_dot[0]<0 ? 0.25 : 1;
			x_dot[1] *= 1.0;
			x_dot[2] *= 1.0;

			cout << endl <<  "---------xdot-ds: " << x_dot.transpose() << endl;
			walking.demand_speeds(x_dot, wrapper.dt);
		}
		bool demand = (task==EMPTY || task==PICK_APPROACH || task==HOLD || task==DROP_APPROACH)
					&& manip.is_at_rest(eps_rest) && (picking_walk || user_walk);
		walking.update(time, wrapper.dt, demand, points, joints, wrapper, nn_models);

		cout << walking.state << "   " << demand << "  " << picking_walk << "  " << navigate << "  |  " << "IK time: " << ik_time;
		cout << endl;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// send final commands //////////////////////////////////////////////////////////////////////////
		double e = exp(-pow(time/3.0,2.0));  // WARNING
		joints.com_pos = joints.init_pos * e + joints.ref_pos * (1.0-e); //smooth starting sequence
		double e2 = (1-e)*exp(-pow((t_sca_switch-time),2.0)); //(1-e) to avoid jump at t=0
		joints.com_pos = pre_sca_switch_pose * e2 + joints.com_pos * (1.0-e2); //smooth IK jump while switching SCA mode
		OutRecord << time << "   " << joints.com_pos.transpose() << endl;

		wrapper.controlJoint(	joints.mode.segment(6,AIR_N_U-6), 
								joints.freeze.segment(6,AIR_N_U-6), 
								joints.com_pos.segment(6,AIR_N_U-6),
								joints.ref_tau.segment(6,AIR_N_U-6));
		if(verbose)
			cout << "sens: " << Box.sens_pos.transpose() << endl;
			//cout << endl << e << " " << e2 << " " << t_sca_switch << " " << time <<  endl;

		//cout << "cmd: " << manip.get_hands().transpose() << endl;
		//cout << "target: " << Ptarget->get_hand().transpose() << endl;
		//cout << joints.com_pos.segment(6,AIR_N_U-6).transpose()  << endl;
		
	}

	wrapper.initializeJoint(VectorXd::Zero(AIR_N_U-6), joints.freeze.segment(6,AIR_N_U-6));
	wrapper.rePID(false);
	nonblock(0);
	wrapper.close();
	#if defined(ICUBSIM)
		gazebo::client::shutdown();
	#endif
	return 0;
}
