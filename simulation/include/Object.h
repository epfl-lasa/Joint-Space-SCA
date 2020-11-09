#pragma once

#include "Basics.h"

class Object
{
public:

	// name
	string name;

	// mocap positions, orientations
	VectorXd sens_pos;
	VectorXd sens_vel;
	VectorXd ideal_pos;

	// the xyz dimensions, assuming a box shape
	Vector3d dim;

	// How distant the hands should be with respect to the object
	double max_expansion;
	// Dynamic shrinkage of the hands on the object
	double grow;
	// Determines how close we can get to the object when walking towards it
	double proximity;
	
	// the direction (from object center) in which the left grasp should be made
	Vector3d ideal_grasp_axis;
	Vector3d opt_grasp_axis;
	// most comfortable rotation around the axis
	Vector4d ideal_grasp_rot;
	Vector4d opt_grasp_rot;
	// most comfortable offset with respect to object center
	Vector3d ideal_grasp_offset;
	Vector3d opt_grasp_offset;
	// any offset on the surface of the grasp
	Vector3d hand_offset;
	// any offset on the surface of the grasp
	bool grasp_opposite;

	// grasping behavior
	double max_force;

	// whether the markers are attached on top. Otherwise they give the geometric center by default.
	bool top_marker;

	// enable obstacle avoidance
	bool enable_avoidance;
	// smoothness of the ellipse: 1.0 means ellipse and inf means box
	double curvature;

	// quaternion between two different grasp axis
	Vector4d local_rot(Vector3d v1, Vector3d v2);

	Object(VectorXd des_obj);
	~Object() {};

	// get the raw mocap data and process, if not valid stick to the previous positions
	bool update_position(VectorXd P_B, VectorXd P_lf, VectorXd P_rf, VectorXd P_R, VectorXd P_obj);

	// get the desired grasping points expanded
	VectorXd get_hand();

	// get hand positions at a desired place
	VectorXd get_hand_moved(VectorXd center);

	// get hand positions in fron of the robot
	VectorXd get_hand_ideal();

	// optimal grasp
	bool allow_search;
	void optimize_grasp();
};

//! add two transformations together
VectorXd trans7_add(VectorXd a, VectorXd b);

//! invert a transformation
VectorXd trans7_invert(VectorXd a);

//! describe a transformation a in b
VectorXd trans7_sub(VectorXd a, VectorXd b);

//! scale a transformation
VectorXd trans7_scale(VectorXd a, double b);