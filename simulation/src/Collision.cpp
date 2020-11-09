#include "Collision.h"

Collision::Collision()
{
	string filepath_in = "../../dependencies/collision/meshes/input/";
	meshes_zero = read_txt_meshes(filepath_in);
	Body meshes_new;
	string filepath_map = "../../dependencies/collision/meshes/idx_map.txt";
	auto collision_map = read_noncol_map(filepath_map);	

	idx_map.resize(meshes_zero.size());
	for(int i=0;i<meshes_zero.size();i++)
		idx_map[i] = get<0>(meshes_zero[i]);
		
	collisions_grid = CollisionsBodyGrid(meshes_zero, collision_map, idx_map);
	colObjs.resize(meshes_zero.size());
	colObjs = createCollisionObj(meshes_zero);
}

vector<string> Collision::check(Contact_Manager &points)
{
	updateCollisionObj(colObjs, points.model, idx_map);
	bool flag = false;
	vector<string> collided;
	for(int i=0;i<idx_map.size();i++)
	{		
		for(int j=0;j<collisions_grid[i].size();j++)
		{	
			int i1 = i;
			int i2 = collisions_grid[i][j];
			c_res.clear();
			fcl::collide(colObjs[i1], colObjs[i2], c_req, c_res);
			if(c_res.isCollision())
			{
				string name1 = get<3>(meshes_zero[i1]);
				string name2 = get<3>(meshes_zero[i2]);
				collided.push_back(name1);
				collided.push_back(name2);
				// cout<< "Collision between "<< name1 <<" and " << name2 << endl;
				flag = true;
			}
		}
	}
	// string filepath_out = "../../dependencies/meshes/output/";
	// meshes_new = update_mesh(meshes_zero, points.model);
	// int tmp = write_txt_meshes(meshes_new, filepath_out);
	return collided;
}
