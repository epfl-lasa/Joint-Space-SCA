#pragma once

#include "Model.h"

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

Vector4d getQ(VectorXd Q);
VectorXd setQ(VectorXd Q, Vector4d a);

class Joints
{
public:
	
	VectorXd ref_pos;
	VectorXd sens_pos;
	VectorXd init_pos;
	VectorXd Ik_pos0;
	VectorXd com_pos;
	VectorXd ref_tau;
	VectorXd sens_vel;
	VectorXd sens_acc;
	VectorXd sens_tau;
	VectorXd freeze;
	VectorXd mode;

	Joints();
	~Joints() {};
	void plot(Model &model);
};
