#include "Object.h"

# define left_side Vector3d(0,1,0)

Object::Object(VectorXd des_obj)
{
	name = "not_assigned";
	sens_pos = zero_v7;
	sens_vel = vectorbig(zero_v3, zero_v3);
	ideal_pos = des_obj;
	dim = Vector3d(1,1,1) * 0.2;
	max_expansion = 0.1;
	grow = 1;
	proximity = 0.1;
	// default object pos is in front of the robot, default object is a box
	ideal_grasp_axis = left_side;
	opt_grasp_axis = ideal_grasp_axis;
	ideal_grasp_rot = ang2quat(Vector3d(0,-M_PI/2,0));
	opt_grasp_rot = ideal_grasp_rot;
	ideal_grasp_offset = zero_v3;
	opt_grasp_offset = zero_v3;
	hand_offset = zero_v3;
	grasp_opposite = true;
	top_marker = false;
	max_force = 20;
	allow_search = true;
	enable_avoidance = true;
	curvature = 4;
}

VectorXd Object::get_hand()
{
	VectorXd out = VectorXd::Zero(14);
	Vector3d pos = sens_pos.segment(0,3) + quat2dc(sens_pos.segment(3,4)) * opt_grasp_offset;
	Vector4d rot = sens_pos.segment(3,4);

	// the hand_offset vector is defined in the left hand frame: fingers: +x, back of the hand: +z
	// this transforms the hand frame to the default world frame when the robot is upright
	Vector4d hand_transform = quat_mul(ang2quat(Vector3d(-M_PI/2.0,0,0)), ang2quat(Vector3d(0,0,M_PI/2.0)));

	Vector3d direction = opt_grasp_axis;
	Vector4d palm_rot  = opt_grasp_rot;	
	direction *= abs(double(direction.adjoint() * dim))/2 + max_expansion * grow;
	
	Vector4d rot_lh = quat_mul(local_rot(left_side, direction), palm_rot);
	out.segment(3,4) = quat_mul(rot, rot_lh);
	out.segment(0,3) = pos + quat2dc(rot) * direction + quat2dc(out.segment(3,4)) * quat2dc(hand_transform) * hand_offset;
	
	direction *= grasp_opposite ? -1.0 : 1.0;
	
	Vector4d rot_rh = quat_mul(local_rot(-left_side, direction), palm_rot);
	out.segment(10,4) = quat_mul(rot, rot_rh);
	out.segment(7,3) = pos + quat2dc(rot) * direction - quat2dc(out.segment(3,4)) * quat2dc(hand_transform) * hand_offset;

	return out;
}

VectorXd Object::get_hand_moved(VectorXd center)
{
	// calculate actual hand positions on the object
	VectorXd out = get_hand();
	Vector3d pos_actual = sens_pos.segment(0,3) + quat2dc(sens_pos.segment(3,4)) * opt_grasp_offset;
	Vector4d rot_actual = out.segment(3,4);

	// calculate ideal hand positions at the target point
	Vector3d pos_ideal = center.segment(0,3);
	Vector4d rot_ideal = quat_mul(local_rot(left_side, ideal_grasp_axis), ideal_grasp_rot);
	rot_ideal = quat_mul(center.segment(3,4), rot_ideal);

	// now move hands to the ideal place
	Vector4d delta_rot = quat_mul(rot_ideal, quat_inverse(rot_actual));

	out.segment(0,3) = pos_ideal + quat2dc(delta_rot) * (out.segment(0,3) - pos_actual);
	out.segment(7,3) = pos_ideal + quat2dc(delta_rot) * (out.segment(7,3) - pos_actual);

	out.segment(3,4)  = quat_mul(delta_rot, out.segment(3,4));
	out.segment(10,4) = quat_mul(delta_rot, out.segment(10,4));

	return out;
}

VectorXd Object::get_hand_ideal()
{
	return get_hand_moved(ideal_pos);
}

VectorXd trans7_add(VectorXd a, VectorXd b)
{
	VectorXd c(7);
	c.segment(0,3) = a.segment(0,3) + b.segment(0,3);
	c.segment(3,4) = quat_mul(b.segment(3,4), a.segment(3,4));
	return c;
}

VectorXd trans7_invert(VectorXd a)
{
	VectorXd c = a;
	c.segment(0,3) *= -1.0;
	c.segment(3,4) = quat_inverse(c.segment(3,4));
	return c;
}

VectorXd trans7_sub(VectorXd a, VectorXd b)
{
	VectorXd c(7);
	b = trans7_invert(b);
	c.segment(0,3) = quat2dc(b.segment(3,4)) * (a.segment(0,3) + b.segment(0,3));
	c.segment(3,4) = quat_mul(b.segment(3,4), a.segment(3,4));
	return c;
}

VectorXd trans7_scale(VectorXd a, double b)
{
	VectorXd c = a;
	c.segment(0,3) *= b;
	c.segment(3,4) =  quat_scale(c.segment(3,4), b);
	return c;
}

bool Object::update_position(VectorXd P_B, VectorXd P_lf, VectorXd P_rf, VectorXd P_R, VectorXd P_obj)
{
	// This function brings the Object to a frame attached to the mid-foot point, heading forward
	// model's base frame: P_B
	// model's feet frame, P_lf, P_rf

	// model's mid-foot frame
	VectorXd P_F = trans7_add(trans7_scale(P_lf,0.5), trans7_scale(P_rf,0.5));

	// model's mid-foot frame in model's base frame
	VectorXd P_F_B = trans7_sub(P_F, P_B);

	// reality's mid-foot frame in reality's base frame
	VectorXd P_Fp_R = P_F_B;

	// reality's object frame in reality's base frame
	VectorXd P_obj_R = trans7_sub(P_obj, P_R);

	// reality's object frame in reality's mid-foot frame
	VectorXd P_obj_Fp = trans7_sub(P_obj_R, P_Fp_R);

	sens_pos = P_obj_Fp;

	if(top_marker)
		sens_pos.segment(0,3) -= quat2dc(sens_pos.segment(3,4)) * Vector3d(0,0,dim[2]/2.0);

	optimize_grasp();

	return true;
}

Vector4d Object::local_rot(Vector3d v1, Vector3d v2)
{
	Vector4d q;
	v1.normalize();
	v2.normalize();
	Vector3d a = v1.cross(v2);
	if(v1.adjoint()*v2 < (-1 + 1e-3))
	 	a = abs(v1[0]) > abs(v1[2]) ? Vector3d(-v1[1], v1[0], 0) : Vector3d(0, -v1[2], v1[1]);
	q.segment(0,3) = a;
	q[3] = 1 + v1.adjoint()*v2;
	return q.normalized();
}

void Object::optimize_grasp()
{
	// this function searches over different sides of the object to find the closest grasping point
	// for now, orientations and postiions are decoupled
	// first decide which side to grasp
	MatrixXd map = MatrixXd(6,3);
	map <<  1, 0, 0,
			-1, 0, 0,
			0, 1, 0,
			0, -1, 0,
			0, 0, 1,
			0, 0, -1;
	double min_diff = 1e6;
	Vector3d ax_diff = left_side;
	Vector4d rot_diff = zero_quat;
	// the desired left hand rotation when the object is in front of the robot
	Vector4d rot_desired = quat_mul(local_rot(left_side, ideal_grasp_axis), ideal_grasp_rot);
	for(int i=0;i<6;i++)
	{
		// search over grasping sides
		Vector3d ax = allow_search ? map.block(i,0,1,3).transpose() : ideal_grasp_axis;
		// ignore too big sides
		if(abs(ax.adjoint()*dim) > 0.25) 
			continue;
		for(int j=0;j<4;j++)
		{
			// search over different grasp orientations around the grasp axis
			Vector4d palm_rot = allow_search ? ang2quat(left_side * j * M_PI/2.0) : ideal_grasp_rot;
			Vector4d rot = quat_mul(local_rot(left_side, ax), palm_rot);
			rot = quat_mul(sens_pos.segment(3,4), rot);
			double error = quat_diff(rot, rot_desired).norm();
			if(error<min_diff)
			{
				ax_diff = ax;
				min_diff = error;
				rot_diff = palm_rot;
			}
		}
	}
	opt_grasp_axis = ax_diff;
	opt_grasp_rot = rot_diff;

	// now search whether we should grasp corners of the object
	int index1=0, index2=1;
	if(abs(opt_grasp_axis[0]) > 0.5)
	{
		index1 = 1; 
		index2 = 2;
	}
	else if(abs(opt_grasp_axis[1]) > 0.5)
	{
		index1 = 0; 
		index2 = 2;
	}
	else if(abs(opt_grasp_axis[2]) > 0.5)
	{
		index1 = 0; 
		index2 = 1;
	}

	min_diff = 1e6;
	Vector3d offset_diff = ideal_grasp_offset;
	Vector3d mid_shoulder(0,0,0.8);
	for(int i=-1;i<2;i++)
		for(int j=-1;j<2;j++)
		{
			Vector3d offset = zero_v3;
			offset[index1] = double(i) * max(dim[index1] / 2.0 - 0.1, 0.0);
			offset[index2] = double(j) * max(dim[index2] / 2.0 - 0.1, 0.0);
			Vector3d grasp_point = sens_pos.segment(0,3) + quat2dc(sens_pos.segment(3,4)) * offset;
			double error = (grasp_point - mid_shoulder).norm();
			if(error<min_diff)
			{
				min_diff = error;
				offset_diff = offset;
			}
		}
	opt_grasp_offset = offset_diff;
}