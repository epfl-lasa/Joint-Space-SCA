#include <string>
#include <iosfwd>
#include <experimental/filesystem>
#include <eigen3/Eigen/Dense>
#include "Model.h"
#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model-inl.h>
#include <fcl/math/bv/OBBRSS.h>
#include <fcl/math/constants.h>
#include <fcl/narrowphase/collision_object.h>

typedef std::vector<std::tuple<int,std::vector<Eigen::Vector3d>,std::vector<Eigen::Vector3i>,std::string>> Body;
typedef fcl::BVHModel<fcl::OBBRSSd> fcl_body;
std::vector<fcl::CollisionObjectd*> createCollisionObj(const Body &meshes);
void updateCollisionObj(std::vector<fcl::CollisionObjectd*> colObjs, Model &model, Eigen::VectorXi &idx_map);
Body read_txt_meshes(const std::string & path);
int write_txt_meshes(Body & meshes, std::string & fname_out);
Body update_mesh(const Body &meshes_dflt, Model &model);
int find_idx(const Eigen::VectorXi & array, const int & idx);
std::vector<Eigen::VectorXi> CollisionsBodyGrid(const Body& meshes, std::vector<Eigen::VectorXi> collision_map, Eigen::VectorXi idx_map);
std::vector<Eigen::VectorXi> read_noncol_map(const std::string& fname);
std::vector<fcl::Triangle> createTriangle(const std::vector<Eigen::Vector3i> faces);
std::vector<Eigen::VectorXf> read_bin_data(const std::string& fname);


