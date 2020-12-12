#include "jac_sca.h"

jac_sca::jac_sca()
{
	string filepath_in = "../../dependencies/collision/meshes/input/simple/0";
	meshes_zero = read_txt_meshes(filepath_in);
	Body meshes_new;
	string filepath_map = "../../dependencies/collision/meshes/idx_map_c.txt";
	auto collision_map = read_noncol_map(filepath_map);	

	idx_map.resize(meshes_zero.size());
	idx_bnames.resize(meshes_zero.size());

	for(int i=0;i<meshes_zero.size();i++)
	{
		idx_map[i] = get<0>(meshes_zero[i]);
		idx_bnames[i] = get<3>(meshes_zero[i]);
	}
		
	collisions_grid = CollisionsBodyGrid(meshes_zero, collision_map, idx_map);
	colObjs.resize(meshes_zero.size());
	colObjs = createCollisionObj(meshes_zero);
    d_req.enable_nearest_points = true;
}

double jac_sca::calc_mindist(Contact_Manager &points)
{
    double mindist = 10;
	updateCollisionObj(colObjs, points.model, idx_map);
	bool flag = false;
	vector<string> collided;
    int i1m = 0;
    int i2m = 0;
	for(int i=0;i<idx_map.size();i++)
	{		
		for(int j=0;j<collisions_grid[i].size();j++)
		{	
			int i1 = i;
			int i2 = collisions_grid[i][j];
			d_res.clear();
			fcl::distance(colObjs[i1], colObjs[i2], d_req, d_res);
            if(d_res.min_distance < mindist)
            {
                mindist = d_res.min_distance;
                i1m = i1;
                i2m = i2;
            }
		}
	}
    //cout << d_res.nearest_points[0].transpose() << endl;
    //cout << d_res.nearest_points[1].transpose() << endl;
    //cout << idx_map.transpose() << endl;
    //cout << idx_bnames[0] << endl;
    //cout << idx_bnames[1] << endl;
    MatrixXd J1 = points.model.get_jacob(idx_map[i1m],Vector3d(0,0,0),CT_TRANSLATION);
    //MatrixXd J1pinv = J1.completeOrthogonalDecomposition().pseudoInverse();
    MatrixXd J2 = points.model.get_jacob(idx_map[i2m],Vector3d(0,0,0),CT_TRANSLATION);
    //MatrixXd J2pinv = J2.completeOrthogonalDecomposition().pseudoInverse();
    cout << J1.transpose() << endl;
    cout << J1.size() << endl;
	return mindist;
}
