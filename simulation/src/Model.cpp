#include "Model.h"
using namespace std;

void Model::init()
{
	icub_air_init();

	// icub joint limits, obtained from urdf automatically
	qmin = VectorXd::Ones(AIR_N_U-6) * (-200);
	qmax = VectorXd::Ones(AIR_N_U-6) * (200);
	taumin = VectorXd::Ones(AIR_N_U-6) * (-200);
	taumax = VectorXd::Ones(AIR_N_U-6) * (200);
	ddqmin = VectorXd::Ones(AIR_N_U-6) * (-500.0/180.0*M_PI);
	ddqmax = VectorXd::Ones(AIR_N_U-6) *  (500.0/180.0*M_PI);

	// didn't find the appropriate function in yarp to read them automatically.
	// the last three elements of the hands are updated to give more strength for object picking tasks (also in the sdf files).
	taumax << 36, 80, 80, 20, 20, 20,
			  84, 84, 40, 30, 24, 11,
			  84, 84, 40, 30, 24, 11,
			  84, 84, 34, 20, 0.45*0+3, 0.65*0+3, 0.65*0+3,
			  84, 84, 34, 20, 0.45*0+3, 0.65*0+3, 0.65*0+3;
	taumin = -taumax;
}

void Model::set_state(VectorXd in_state, VectorXd in_stated)
{
	icub_air_state( 0.0, &in_state[0], &in_stated[0] );
}

VectorXd Model::get_equivht()
{
	VectorXd tau = VectorXd::Zero(AIR_N_U);
	icub_air_equivht( &tau[0] );
	return tau;
}

VectorXd Model::get_comptrq(VectorXd& udot)
{
	VectorXd tau = VectorXd::Zero(AIR_N_U);
	icub_air_comptrq( &udot[0],  &tau[0] );
	return tau;
}

VectorXd Model::get_fulltrq(VectorXd& udot, VectorXd& mult)
{
	VectorXd tau = VectorXd::Zero(AIR_N_U);
	icub_air_fulltrq( &udot[0], &mult[0], &tau[0] );
	return tau;
}

MatrixXd Model::get_massmat()
{
	double mmat[AIR_N_U][AIR_N_U];
	icub_air_massmat( mmat );
	MatrixXd M = MatrixXd::Zero(AIR_N_U, AIR_N_U);
	for(int i=0;i<AIR_N_U;i++)
		for(int j=0;j<AIR_N_U;j++)
			M(i,j) = mmat[i][j];
	return M;
}

VectorXd Model::get_frcmat()
{
	VectorXd tau = VectorXd::Zero(AIR_N_U);
	icub_air_frcmat( &tau[0] );
	return tau;
}

VectorXd Model::get_multtrq(VectorXd & mult)
{
	VectorXd tau = VectorXd::Zero(AIR_N_U);
	icub_air_multtrq( &mult[0], &tau[0] );
	return tau;
}

VectorXd Model::get_trans7(int body, Vector3d pt)
{
	return vectorbig(get_pos(body, pt), get_orient_quat(body));
}

VectorXd Model::get_vel6(int body, Vector3d pt)
{
	return vectorbig(get_vel(body, pt), get_angvel(body));
}

Vector3d Model::get_pos(int body, Vector3d pt)
{
	Vector3d loc_cp = zero_v3;
	icub_air_pos(body, &pt[0], &loc_cp[0]);
	return loc_cp;
}

Vector3d Model::get_vel(int body, Vector3d pt)
{
	// automatically gives the velocity in the global ref frame
	Vector3d loc_cp = zero_v3;
	icub_air_vel(body, &pt[0], &loc_cp[0]);
	return loc_cp;
}

Vector3d Model::get_acc(int body, Vector3d pt)
{
	Vector3d loc_cp = zero_v3;
	icub_air_acc(body, &pt[0], &loc_cp[0]);
	return loc_cp;
}

MatrixXd Model::get_orient(int body)
{
	double dircos[3][3];
	icub_air_orient(body, dircos);
	return matrix3_convert(dircos);
}

VectorXd Model::get_orient_quat(int body)
{
	MatrixXd Orient = get_orient(body);
	return dc2quat(Orient);
}

Vector3d Model::get_orient_ang(int body)
{
	MatrixXd Orient = get_orient(body);
	return dc2ang(Orient);
}

Vector3d Model::get_angvel(int body)
{
	// gives the velocity in the local ref frame, we should multiply by body
	Vector3d loc_cp;
	icub_air_angvel(body, &loc_cp[0]);
	icub_air_trans(body, &loc_cp[0], -1, &loc_cp[0]);
	return loc_cp;
}

Vector3d Model::get_angacc(int body)
{
	// gives the velocity in the local ref frame, we should multiply by body
	Vector3d loc_cp;
	icub_air_angacc(body, &loc_cp[0]);
	icub_air_trans(body, &loc_cp[0], -1, &loc_cp[0]);
	return loc_cp;
}

Vector3d Model::get_trans(int frbod, Vector3d& ivec, int tobod)
{
	double loc_cp[3];
	icub_air_trans(frbod, &ivec[0], tobod, loc_cp);
	return Vector3d(loc_cp[0], loc_cp[1], loc_cp[2]);
}

MatrixXd Model::get_jacob(int body, Vector3d pt, constraint_type type)
{
	// gives the velocity in the local ref frame, we should multiply
	double lin[3];
	double rot[3];

	MatrixXd J((type==CT_TRANSLATION || type==CT_ROTATION) ? 3 : 6, AIR_N_U);
	for(int i=0;i<AIR_N_U;i++)
	{
		icub_air_rel2cart(i, body, &pt[0], lin, rot);
		if(type==CT_TRANSLATION || type==CT_FULL)
		{
			icub_air_trans(body, lin, -1, lin);
			J(0, i) = lin[0];
			J(1, i) = lin[1];
			J(2, i) = lin[2];
		}
		if(type==CT_ROTATION || type==CT_FULL)
		{
			int shift = type==CT_FULL ? 3 : 0;
			icub_air_trans(body, rot, -1, rot);
			J(shift+0, i) = rot[0];
			J(shift+1, i) = rot[1];
			J(shift+2, i) = rot[2];
		}
	}
	return J;
}

void Model::set_acc(VectorXd acc)
{
	icub_air_setudot(&acc[0]);
	double oqdot[AIR_N_U+1];
}

void  Model::get_mom(Vector3d &LM, Vector3d &AM, double * KE)
{
	double ke;
	icub_air_mom(&LM[0], &AM[0], &ke);
	if(KE!=NULL)
		*KE = ke;
}

void Model::get_sys(Vector3d &CM, MatrixXd &ICM, double * MTOT)
{
	double icm[3][3];
	double mtot;
	icub_air_sys(&mtot, &CM[0], icm);
	ICM = matrix3_convert(icm);
	*MTOT = mtot;
}

void Model::get_reac(MatrixXd &F, MatrixXd &T)
{
	double f[AIR_N_BODY][3];
	double t[AIR_N_BODY][3];
	icub_air_reac(f, t);
	F.resize(AIR_N_BODY,3);
	T.resize(AIR_N_BODY,3);
	for(int i=0;i<AIR_N_BODY;i++)
		for(int j=0;j<3;j++)
		{
			F(i,j)= f[i][j];
			T(i,j)= t[i][j];
		}
}

double Model::get_mass()
{
	double mass=0;
	double dm;
	for(int i=0;i<AIR_N_BODY;i++)
	{
		icub_air_getmass(i, &dm);
		mass += dm;
	}
	return mass;
}

double Model::get_mass(int body)
{
	double dm;
	icub_air_getmass(body, &dm);
	return dm;
}

MatrixXd Model::get_inertia(int body)
{
	double icm[3][3];
	icub_air_getiner(body, icm);
	return matrix3_convert(icm);
}

Vector3d Model::get_cm()
{
	double mass=0;
	Vector3d cm = zero_v3;
	double pt_cp[3] = {0,0,0};
	double loc_cp[3];
	for(int i=0;i<AIR_N_BODY;i++)
	{
		icub_air_getmass(i, &mass);
		icub_air_pos(i, pt_cp, loc_cp);
		cm[0] +=  loc_cp[0] * mass;
		cm[1] +=  loc_cp[1] * mass;
		cm[2] +=  loc_cp[2] * mass;
	}
	return cm/get_mass();
}

Vector3d Model::get_cm_v()
{
	double mass=0;
	Vector3d cm = zero_v3;
	double pt_cp[3] = {0,0,0};
	double loc_cp[3];
	for(int i=0;i<AIR_N_BODY;i++)
	{
		icub_air_getmass(i, &mass);
		icub_air_vel(i, pt_cp, loc_cp);
		cm[0] +=  loc_cp[0] * mass;
		cm[1] +=  loc_cp[1] * mass;
		cm[2] +=  loc_cp[2] * mass;
	}
	return cm/get_mass();
}

Vector3d Model::get_cm_a()
{
	double mass=0;
	Vector3d cm = zero_v3;
	double pt_cp[3] = {0,0,0};
	double loc_cp[3];
	for(int i=0;i<AIR_N_BODY;i++)
	{
		icub_air_getmass(i, &mass);
		icub_air_acc(i, pt_cp, loc_cp);
		cm[0] +=  loc_cp[0] * mass;
		cm[1] +=  loc_cp[1] * mass;
		cm[2] +=  loc_cp[2] * mass;
	}
	return cm/get_mass();
}

MatrixXd Model::get_cm_J()
{
	MatrixXd J = MatrixXd::Zero(3, AIR_N_U);
	double pt_cp[3]={0,0,0};
	double lin[3];
	double rot[3];

	double mass = 0;
	double M = get_mass();
	for(int j=0;j<AIR_N_BODY;j++)
	{
		icub_air_getmass(j, &mass);
		mass /= M;
		for(int i=0;i<AIR_N_U;i++)
		{
			icub_air_rel2cart(i, j, pt_cp, lin, rot);
			icub_air_trans(j, lin, -1, lin);
			J(0, i) += lin[0]*mass;
			J(1, i) += lin[1]*mass;
			J(2, i) += lin[2]*mass;
		}
	}
	return J;
}

void Model::check_consistency(VectorXd joint_pos, VectorXd joint_vel, VectorXd joint_acc)
{
	// check consistency of joint and global variables, jacobians and sdfast functions
	///////////////////////////// global vars
	// suppose the inertial fixed frame is called i
	// and the base frame is called b
	VectorXd OR = ang2quat(Vector3d(0.5,0.8,0.7));
	MatrixXd rot = quat2dc(OR); // base orientation in i
	Vector3d w(0.1, 0.5, -0.4); // base angular velocity expressed in b.
	Vector3d v(0.01,0.02,0.03); // base linear velocity in i
	Vector3d p(0.1,0.3,0.6); // base position in i
	Vector3d a(-0.3,0.5,0.7); // base acceleration in i
	Vector3d al(-0.6,0.4,0.9); // base angular acceleration in b.

	// now chose a link in the robot
	int body = 22;
	Vector3d offset_r = zero_v3;

	///////////////////////////// raw kinematics
	// setting base positions and orientations to zero in sdfast
	// in order to calculate end-effector's (chosen body) variables in base frame
	joint_pos[AIR_N_U] = 1;
	joint_pos.segment(0,6).setZero();
	joint_vel.segment(0,6).setZero();
	joint_acc.segment(0,6).setZero();

	for(int i=6;i<AIR_N_U;i++)
	{
		//joint_pos[i] += (double(rand()%1000)/1000.0*2.0-0.5);
		//joint_vel[i] += (double(rand()%1000)/1000.0*2.0-0.5);
		//joint_acc[i] += (double(rand()%1000)/1000.0*2.0-0.5);
	}

	set_state(joint_pos, joint_vel);
	set_acc(joint_acc);

	Vector3d pos = get_pos(body, offset_r);
	Vector3d vel = get_vel(body, offset_r);
	Vector3d acc = get_acc(body, offset_r);
	MatrixXd ang = get_orient(body);
	Vector3d angvel = get_angvel(body);
	Vector3d angacc = get_angacc(body);

	///////////////////////////// complex kinematics
	// setting base positions and orientations to desired values in sdfast
	// in order to calculate end-effector's (chosen body) variables in inertial frame
	joint_pos[AIR_N_U] = OR[3];
	joint_pos.segment(0,6) = vectorbig(p,OR.segment(0,3));
	joint_vel.segment(0,6) = vectorbig(v,w);
	joint_acc.segment(0,6) = vectorbig(a,al);
	set_state(joint_pos, joint_vel);
	set_acc(joint_acc);

	cout << endl << "/////////////check consistency//////////////" << endl;

	Vector3d pos1 = p + rot * pos;
	cout << "pos1 " << pos1.transpose() << endl;
	Vector3d pos2 = get_pos(body, offset_r);
	cout << "pos2 " << pos2.transpose() << endl;

	cout << "////////////////////////////////////////////" << endl;

	Vector3d rot1 = dc2ang(rot * ang);
	cout << "rot1 " << rot1.transpose() << endl;
	Vector3d rot2 = get_orient_ang(body);
	cout << "rot2 " << rot2.transpose() << endl;

	cout << "////////////////////////////////////////////" << endl;

	Vector3d vel1 = v + rot * vel + rot * skew_symmetric(w) * pos;
	cout << "vel1 " << vel1.transpose() << endl;
	Vector3d vel2 = get_vel(body, offset_r);
	cout << "vel2 " << vel2.transpose() << endl;
	Vector3d vel3 = get_jacob(body,offset_r,CT_TRANSLATION) * joint_vel;
	cout << "vel3 " << vel3.transpose() << endl;

	cout << "////////////////////////////////////////////" << endl;

	Vector3d angvel1 = rot * angvel + rot * w;
	cout << "angvel1 " << angvel1.transpose() << endl;
	Vector3d angvel2 = get_angvel(body);
	cout << "angvel2 " << angvel2.transpose() << endl;
	Vector3d angvel3 = get_jacob(body,offset_r,CT_ROTATION) * joint_vel;
	cout << "angvel3 " << angvel3.transpose() << endl;

	cout << "////////////////////////////////////////////" << endl;

	Vector3d acc1 = a + rot * acc +
					rot * skew_symmetric(w) * vel * 2 +
					rot * skew_symmetric(w) * skew_symmetric(w) * pos +
					rot * skew_symmetric(al) * pos;
	cout << "acc1 " << acc1.transpose() << endl;
	Vector3d acc2 = get_acc(body, offset_r);
	cout << "acc2 " << acc2.transpose() << endl;

	VectorXd copy = joint_acc * 0;
	set_acc(copy);
	Vector3d acc3 = get_acc(body, offset_r);
	acc3 += get_jacob(body,offset_r,CT_TRANSLATION) * joint_acc;
	set_acc(joint_acc);
	cout << "acc3 " << acc3.transpose() << endl;

	cout << "////////////////////////////////////////////" << endl;

	Vector3d angacc1 = rot * skew_symmetric(w) * angvel +
					  rot * angacc +
					  rot * al;
	cout << "angacc1 " << angacc1.transpose() << endl;
	Vector3d angacc2 = get_angacc(body);
	cout << "angacc2 " << angacc2.transpose() << endl;

	set_acc(copy);
	Vector3d angacc3 = get_angacc(body);
	angacc3 += get_jacob(body,offset_r,CT_ROTATION) * joint_acc;
	set_acc(joint_acc);
	cout << "angacc3 " << angacc3.transpose() << endl;

	cout << "/////////////check consistency//////////////" << endl << endl;

// conclusion:
// in the state vector, pos, vel and acc are in global frame
// in the state vector, OR is global, but w and al are local, in base frame
}
