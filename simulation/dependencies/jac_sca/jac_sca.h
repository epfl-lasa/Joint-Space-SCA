#pragma once

#include "Contacts.h"
#include "MeshUtils.h"

class jac_sca
{
public:

	fcl::DistanceRequestd d_req;
	fcl::DistanceResultd d_res;
	VectorXi idx_map;
	vector<string> idx_bnames;

	vector<fcl::CollisionObjectd*> colObjs;
	vector<Eigen::VectorXi> collisions_grid;
	Body meshes_zero;

	jac_sca();
	~jac_sca() {};
	double calc_mindist(Contact_Manager &points);
};
