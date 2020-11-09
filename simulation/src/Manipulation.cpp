#include "Manipulation.h"

using namespace std;

Manipulation::Manipulation()
{
	bias_lh = VectorXd::Zero(6);
	bias_rh = VectorXd::Zero(6);
	avg_lh = VectorXd::Zero(6);
	avg_rh = VectorXd::Zero(6);	
	force_lh = VectorXd::Zero(6);
	force_rh = VectorXd::Zero(6);
	reading_lh = VectorXd::Zero(6);
	reading_rh = VectorXd::Zero(6);
	grav_compns = VectorXd::Zero(AIR_N_U);
	
	#ifdef ICUB5
		// pure bias values read from the hardware when stretching the arms and subtracting lower arm weights
		bias_lh << -11.8466,  -14.6179,  -36.7255,  0.177665, -0.748966,   0.18572;
		bias_rh <<  32.6632,  -5.57603,  -55.8315, -0.748966, -0.511756,  0.313862;
	#elif ICUB33
		// pure bias values read from the hardware when stretching the arms and subtracting lower arm weights
		bias_lh << 11.7387,   6.31751,   10.4043, 0.0467604,  0.417137,  0.537757;
		bias_rh << 16.2898,   16.8943,   4.73801, -0.233956,  0.600148,  0.097026;
	#endif

	// multiarm ds variables
	gamma = 1;
	dgamma = 0;
	tau = 0;
	dtau = 0;
	orientation_factor = 3.0;
	A_V = MatrixXd::Zero(3,3);
	A_V(0,0) = -5/2.0;
	A_V(1,1) = -5/2.0;
	A_V(2,2) = -5/2.0;
	A = MatrixXd::Zero(3,3);
	A(0,0) = -1/2.0;
	A(1,1) = -1/2.0;
	A(2,2) = -1/2.0;

	// ideal pos
	xd[0] = zero_v3; qd[0] = zero_quat;
	xd[1] = zero_v3; qd[1] = zero_quat;
	xO[0] = zero_v3; qO[0] = zero_quat;
	xO[1] = zero_v3; qO[1] = zero_quat;
	xR[0] = zero_v3; qR[0] = zero_quat;
	xR[1] = zero_v3; qR[1] = zero_quat;
	xV[0] = zero_v3; qV[0] = zero_quat;
	xV[1] = zero_v3; qV[1] = zero_quat;
}

void Manipulation::set_target_point(VectorXd hands)
{
	VectorXd lh = hands.segment(0,7);
	VectorXd rh = hands.segment(7,7);

	// multiarm position ds initiations
	xO[0] = lh.segment(0,3);
	xO[1] = rh.segment(0,3);
	
	// multiarm orientation ds initiations
	qO[0] = lh.segment(3,4);
	qO[1] = rh.segment(3,4);
}

void Manipulation::set_start_point()
{
	// multiarm position ds initiations
	xR[0] = xd[0];
	xV[0] = xd[0];
	xR[1] = xd[1];
	xV[1] = xd[1];
	
	// multiarm orientation ds initiations
	qR[0] = qd[0];
	qV[0] = qd[0];
	qR[1] = qd[1];
	qV[1] = qd[1];
}

void Manipulation::set_ideal_point(VectorXd hands)
{
	VectorXd lh = hands.segment(0,7);
	VectorXd rh = hands.segment(7,7);

	// multiarm position ds initiations
	xd[0] = lh.segment(0,3);
	xd[1] = rh.segment(0,3);
	
	// multiarm orientation ds initiations
	qd[0] = lh.segment(3,4);
	qd[1] = rh.segment(3,4);
}

void Manipulation::read_forces(Contact_Manager &points)
{
	MatrixXd Forces(AIR_N_BODY,3);
	MatrixXd Torques(AIR_N_BODY,3);
	points.model.get_reac(Forces, Torques);
	grav_compns = points.model.get_frcmat() * (-1);

	MatrixXd RR = points.model.get_orient(26);
	reading_lh = 	vectorbig(	RR*(-points[CN_LH].T.F_sens-bias_lh.segment(0,3)) + RR*Forces.block(26,0,1,3).transpose(), 
								RR*(-points[CN_LH].R.F_sens-bias_lh.segment(3,3)) + RR*Torques.block(26,0,1,3).transpose());
	
	RR = points.model.get_orient(34);
	reading_rh = 	vectorbig(	RR*(-points[CN_RH].T.F_sens-bias_rh.segment(0,3)) + RR*Forces.block(34,0,1,3).transpose(),
								RR*(-points[CN_RH].R.F_sens-bias_rh.segment(3,3)) + RR*Torques.block(34,0,1,3).transpose());
}

void Manipulation::initial_filtering(Contact_Manager &points, double dt)
{
	read_forces(points);
	
	// orientation dependent bias if any
	avg_lh += (reading_lh - avg_lh) * 1.0 * dt;
	avg_rh += (reading_rh - avg_rh) * 1.0 * dt;
}

void Manipulation::secondary_filtering(Contact_Manager &points, double dt)
{
	read_forces(points);
	
	// online differences
	force_lh += (reading_lh - avg_lh - force_lh) * 1.0 * dt;
	force_rh += (reading_rh - avg_rh - force_rh) * 1.0 * dt;
}

void Manipulation::compliance_control(Contact_Manager &points, double force)
{
	// calculate force directions
	Vector3d mid = points[CN_RH].ref_p.pos.segment(0,3)*0.5 + points[CN_LH].ref_p.pos.segment(0,3)*0.5;
	Vector3d hold_force_l = (mid - points[CN_LH].ref_p.pos.segment(0,3)).normalized() * force;
	Vector3d hold_force_r = (mid - points[CN_RH].ref_p.pos.segment(0,3)).normalized() * force;

	// This function should be called once and only after inverse kinematics	
	points[CN_LH].ref_p.pos.segment(0,3) += truncate_command(((force_lh).segment(0,3)+hold_force_l) * 0.1/20.0, 0.2, -0.2);
	points[CN_RH].ref_p.pos.segment(0,3) += truncate_command(((force_rh).segment(0,3)+hold_force_r) * 0.1/20.0, 0.2, -0.2);
}

void Manipulation::jacobian_transpose(Contact_Manager &points, Joints &joints, double force)
{
	points[CN_LH].T.K = -Vector3d(30, 1, 1);
	points[CN_LH].R.K = -Vector3d(10, 0.3, 1);
	points[CN_RH].T.K = -Vector3d(30, 1, 1);
	points[CN_RH].R.K = -Vector3d(10, 0.3, 1);
	points.control(CN_LH, points[CN_LH].ref_p);
	points.control(CN_RH, points[CN_RH].ref_p);
	points[CN_LH].T.F_ref += - (points[CN_RH].p.pos - points[CN_LH].p.pos).segment(0,3).normalized() * force;
	points[CN_RH].T.F_ref += - (points[CN_LH].p.pos - points[CN_RH].p.pos).segment(0,3).normalized() * force;
	VectorXd h = grav_compns - points.get_virtual_mult();
	joints.ref_tau.segment(24,14) = h.segment(24,14);
}

void Manipulation::update(bool reachable, double dt, Object * box, bool grasp)
{
	#if defined(ICUB5) || defined(ICUB33)
		dtau = 0.02;
	#elif defined(ICUBSIM)
		dtau = 0.1;
	#endif
	//tau = reachable;
	//tau = std::min(tau+dtau * dt,1.0);
	tau = 1;
	//cout << "tau:" << tau << endl;
	// orientation control gains
	MatrixXd A_V_Q = (A_V.array() * orientation_factor).matrix();
	MatrixXd A_Q = (A.array() * orientation_factor).matrix();
	
	// dynamical systems
	for(int i=0; i<2; i++)
	{
		// multiarm position DS
		Vector3d dxO = zero_v3;
		Vector3d X_I_C = xO[i];
		Vector3d dxV = gamma * dxO + dgamma * (xO[i] - X_I_C) + A_V * ((xV[i] - X_I_C) - gamma * (xO[i] - X_I_C));
		xV[i] += dxV * dt;
		Vector3d ATX = A * (xR[i] - xd[i] - tau * (xV[i] - xd[i])) + dtau * (xV[i] - xd[i]);
		Vector3d dxR  = tau * dxV * 0 + ATX;

		// multiarm orientation DS
		VectorXd dqO = zero_v3;
		VectorXd Q_I_C = qO[i];
		Vector3d dqV = gamma * dqO + dgamma * quat_diff(qO[i], Q_I_C) + A_V_Q * (quat_diff(qV[i], Q_I_C) - gamma * quat_diff(qO[i], Q_I_C));
		qV[i] = quat_mul(quat_exp(0.5 * quat_deriv(dqV * dt)), qV[i]);
		Vector3d ATQ = A_Q * (quat_diff(qR[i], qd[i]) - tau * quat_diff(qV[i], qd[i])) + dtau * quat_diff(qV[i], qd[i]);
		Vector3d dqR  = tau * dqV * 0 + ATQ;

		//obstacle avoidance
		// if(!grasp && box->enable_avoidance)
		// 	dxR = modulate(dxR, xR[i], box);

		// now integrate
		//dxR = tau < 1 ? dxR : dxR*0.1;
		xR[i] += dxR * dt;
		qR[i] = quat_mul(quat_exp(0.5 * quat_deriv(dqR * dt)), qR[i]);
	}
}

VectorXd Manipulation::get_hands()
{
	VectorXd lh = vectorbig(xR[0],qR[0]);
	VectorXd rh = vectorbig(xR[1],qR[1]);
	return vectorbig(lh, rh);
}

bool Manipulation::is_at_target(double eps_obj)
{
	//return (xR[0]-xO[0]).norm() <= eps_obj && (xR[1]-xO[1]).norm() <= eps_obj;
	//cout << "R:" <<(xR[0]-xO[0]).norm() << "?" << eps_obj << endl;
	return (xR[0]-xO[0]).norm() <= eps_obj;
}

bool Manipulation::is_at_rest(double eps_obj)
{
	//return (xR[0]-xd[0]).norm() <= eps_obj && (xR[1]-xd[1]).norm() <= eps_obj;
	return (xR[0]-xd[0]).norm() <= eps_obj;
}

Vector3d Manipulation::modulate(Vector3d dx, Vector3d xR, Object * box)
{
	// translate and rotate p in the frame of the object
	Vector3d p = quat2dc(box->sens_pos.segment(3,4)).inverse() * (xR - box->sens_pos.segment(0,3));
	Vector3d v = quat2dc(box->sens_pos.segment(3,4)).inverse() * dx;
	double margin = box->max_expansion;
	Vector3d dim = box->dim / 2.0 + Vector3d(1,1,1) * margin * 0;
	double u = box->curvature;
	

	// approximate the object with an ellipse, calculate the closest point q on the ellipse to p
	// q lies on the ellipse border defined in F[0], and the jacobian of F at q (called J) is parallel to p-q direction.
	// Therefore, F[1] to F[3] implement J cross (p-q) = 0. In the following, we solve for q iteratively:
	Vector3d q = p;
	for(int i=0; i<20; i++)
	{
		Vector4d F;
		MatrixXd J(4,3);
		F[0] = pow(q[0]/dim[0], 2*u) + pow(q[1]/dim[1], 2*u) + pow(q[2]/dim[2], 2*u) - 1; 
		F[1] = (2*u*pow(q[2]/dim[2], 2*u-1)*(p[1]-q[1]))/dim[2] - (2*u*pow(q[1]/dim[1], 2*u-1)*(p[2]-q[2]))/dim[1]; 
		F[2] = (2*u*pow(q[0]/dim[0], 2*u-1)*(p[2]-q[2]))/dim[0] - (2*u*pow(q[2]/dim[2], 2*u-1)*(p[0]-q[0]))/dim[2]; 
		F[3] = (2*u*pow(q[1]/dim[1], 2*u-1)*(p[0]-q[0]))/dim[1] - (2*u*pow(q[0]/dim[0], 2*u-1)*(p[1]-q[1]))/dim[0]; 
		J(0, 0) = (2*u*pow(q[0]/dim[0], 2*u-1))/dim[0]; 
		J(0, 1) = (2*u*pow(q[1]/dim[1], 2*u-1))/dim[1]; 
		J(0, 2) = (2*u*pow(q[2]/dim[2], 2*u-1))/dim[2]; 
		J(1, 0) = 0; 
		J(1, 1) = - (2*u*pow(q[2]/dim[2], 2*u-1))/dim[2] - (2*u*(2*u-1)*pow(q[1]/dim[1], 2*u-2)*(p[2]-q[2]))/pow(dim[1],2.0); 
		J(1, 2) = (2*u*pow(q[1]/dim[1], 2*u-1))/dim[1] + (2*u*(2*u-1)*pow(q[2]/dim[2], 2*u-2)*(p[1]-q[1]))/pow(dim[2],2.0); 
		J(2, 0) = (2*u*pow(q[2]/dim[2], 2*u-1))/dim[2] + (2*u*(2*u-1)*pow(q[0]/dim[0], 2*u-2)*(p[2]-q[2]))/pow(dim[0],2.0); 
		J(2, 1) = 0; 
		J(2, 2) = - (2*u*pow(q[0]/dim[0], 2*u-1))/dim[0] - (2*u*(2*u-1)*pow(q[2]/dim[2], 2*u-2)*(p[0]-q[0]))/pow(dim[2],2.0); 
		J(3, 0) = - (2*u*pow(q[1]/dim[1], 2*u-1))/dim[1] - (2*u*(2*u-1)*pow(q[0]/dim[0], 2*u-2)*(p[1]-q[1]))/pow(dim[0],2.0); 
		J(3, 1) = (2*u*pow(q[0]/dim[0], 2*u-1))/dim[0] + (2*u*(2*u-1)*pow(q[1]/dim[1], 2*u-2)*(p[0]-q[0]))/pow(dim[1],2.0); 
		J(3, 2) = 0; 
		q += J.householderQr().solve(-F);
	}
	
	// calculate the distance between p and q
	double distance = (p-q).norm();
	// if p was inside the ellipse
	if (pow(p[0]/dim[0], 2*u) + pow(p[1]/dim[1], 2*u) + pow(p[2]/dim[2], 2*u) - 1 < 0)
		distance = 0;

	// determine mixture coefficient
	double alpha = max(1.0 - distance/margin/2.0, 0.0);

	// determine repellant force
	Vector3d rep;
	if(distance == 0)
	{
		rep[0] = (2*u*pow(q[0]/dim[0], 2*u-1))/dim[0];
		rep[1] = (2*u*pow(q[1]/dim[1], 2*u-1))/dim[1];
		rep[2] = (2*u*pow(q[2]/dim[2], 2*u-1))/dim[2];
	}
	else
		rep = p-q;
	rep = rep.normalized() * v.norm();

	// calculate the final velocity with a repellant force
	v = (1-alpha) * v + rep * alpha;
	v = quat2dc(box->sens_pos.segment(3,4)) * v;

	if(isnan(v[0]) || isnan(v[1]) || isnan(v[2]))
	{
		cout << "Warning: obstacle avoidance nan error!" << endl;
		return dx;
	}
	else
		return v;	
}