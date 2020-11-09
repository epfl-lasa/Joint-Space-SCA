#include "Joints.h"

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
/* hip pitch       pos[18]      pos[12]                    */
/* hip roll        pos[19]       pos[13]                   */
/* hip yaw         pos[20]       pos[14]                   */
/* knee            pos[21]       pos[15]                   */
/* ankle pitch     pos[22]       pos[16]                   */
/* ankle roll      pos[23]       pos[17]                   */

Vector4d getQ(VectorXd Q)
{
	Vector4d a;
	a[3] = Q[AIR_N_U];
	a.segment(0,3) = Q.segment(3,3);
	return a;
}

VectorXd setQ(VectorXd Q, Vector4d a)
{
	Q[AIR_N_U] = a[3];
	Q.segment(3,3) = a.segment(0,3);
	return Q;
}

Joints::Joints()
{
	ref_pos  = VectorXd::Zero(AIR_N_U+1); ref_pos[AIR_N_U] = 1;
	sens_pos = VectorXd::Zero(AIR_N_U+1); sens_pos[AIR_N_U] = 1;
	init_pos = VectorXd::Zero(AIR_N_U+1); init_pos[AIR_N_U] = 1;
	com_pos  = VectorXd::Zero(AIR_N_U+1); com_pos[AIR_N_U] = 1;
	Ik_pos0  = VectorXd::Zero(AIR_N_U+1); Ik_pos0[AIR_N_U] = 1;
	//Ik_pos0[15] = -0.3;
	//Ik_pos0[21] = -0.3;
	ref_tau  = VectorXd::Zero(AIR_N_U);
	sens_vel = VectorXd::Zero(AIR_N_U);
	sens_acc = VectorXd::Zero(AIR_N_U);
	sens_tau = VectorXd::Zero(AIR_N_U);
	freeze = VectorXd::Zero(AIR_N_U);
	mode     = VectorXd::Zero(AIR_N_U);
}

void Joints::plot(Model &model)
{
	cout << fixed << endl;
	cout << "  joint     min_pos   init_pos    ref_pos   com_pos    sens_pos    max_pos" << endl;
	for(int i=6;i<AIR_N_U;i++)
	{
		cout << std::setw( 5 ) << i << ":"  <<
		setprecision( 0 ) <<std::setw( 11 ) << (i<6 ? -1000 : model.qmin[i-6]) <<
		setprecision( 2 ) <<std::setw( 11 ) << init_pos[i] * (i<6 ? 1 : 180/M_PI) <<
		setprecision( 2 ) <<std::setw( 11 ) << ref_pos[i] * (i<6 ? 1 : 180/M_PI) <<
		setprecision( 2 ) <<std::setw( 11 ) << com_pos[i] * (i<6 ? 1 : 180/M_PI) <<
		setprecision( 2 ) <<std::setw( 11 ) << sens_pos[i] * (i<6 ? 1 : 180/M_PI) <<
		setprecision( 0 ) <<std::setw( 11 ) << (i<6 ? 1000 : model.qmax[i-6]) <<
		endl;
		if(i==6-1 || i==12-1 || i==18-1 || i==24-1 || i==31-1)
			cout << endl;
	}
}
