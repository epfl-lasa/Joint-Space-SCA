#pragma once

#include "Contacts.h"
#include "MeshUtils.h"

class Collision
{
public:

	fcl::CollisionRequestd c_req;
	fcl::CollisionResultd c_res;
	VectorXi idx_map;
	vector<fcl::CollisionObjectd*> colObjs;
	vector<Eigen::VectorXi> collisions_grid;
	Body meshes_zero;

	Collision();
	~Collision() {};
	vector<string> check(Contact_Manager &points);
};
