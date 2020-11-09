#pragma once
#include <Eigen/Dense>

using namespace Eigen;

//! Converting quaternion to rotation matrix
MatrixXd quat2dc(Vector4d st);

//! Converting rotation matrix to euler angles
Vector3d dc2ang(MatrixXd dc);

//! Converting euler angles to rotation matrix
MatrixXd ang2dc(Vector3d stang);

//! Converting rotation matrix to quaternion
Vector4d dc2quat(MatrixXd dc);

//! Converting quaternion to euler angles
Vector3d quat2ang(Vector4d st);

//! Converting euler angles to quaternion
Vector4d ang2quat(Vector3d stang);

//! loading a double** to a matrix
void matrix3_convert(MatrixXd& in, double out[3][3]);

//! loading a matrix to double**
MatrixXd matrix3_convert(double in[3][3]);

//! Generating a skew symmetric matrix from angular velocities
MatrixXd skew_symmetric(Vector3d in);

//! Getting the vector of angular velocities from a skew symmetric matrix
Vector3d skew_symmetric_inv(MatrixXd &in);

//! Generate a matrix to replace cross product: would be P(a) * q = a x q
MatrixXd make_cross_lh(Vector4d a);

//! Quaternion cross product
Vector4d quat_cross(Vector4d a, Vector4d b);

//! Quaternion triple cross product
Vector4d quat_cross(Vector4d a, Vector4d b, Vector4d c);

//! Normalize a quaternion
Vector4d quat_normalize(Vector4d in);

//! Inverse a quaternion
Vector4d quat_inverse(Vector4d in);

//! Dot product two quaternions
double quat_dot(Vector4d a, Vector4d b);

//! Another implementation of quaternion cross product
Vector4d quat_mul(Vector4d a, Vector4d b);

//! adding two quaternions
Vector4d quat_add(Vector4d a, Vector4d b);

//! subtracting two quaternions
Vector4d quat_sub(Vector4d a, Vector4d b);

//! Finding the cosine between two quaternions
double quat_cos(Vector4d a, Vector4d b);

//! The norm of a quaternion
double quat_norm(Vector4d in);

//! Calculate the angle from a quaternion (assuming axis/angle representation)
double quat_get_theta(Vector4d in);

//! Scale the quaternion (angle) by a scalar
Vector4d quat_scale(Vector4d in, double d);

//! The exponential of a quaternion: e^q
Vector4d quat_exp(Vector4d in);

//! The log of a quaternion: ln(q)
Vector4d quat_log(Vector4d in);

//! The power of a quaternion
Vector4d quat_pow(Vector4d in, double t);

//! Slerp function using quaternion
Vector4d quat_slerp(Vector4d q0, Vector4d q1, double t);

//! Creating a derivative for quaternion
Vector4d quat_deriv(Vector3d in);

//! difference between two quaternions
Vector3d quat_diff(Vector4d a, Vector4d b);

