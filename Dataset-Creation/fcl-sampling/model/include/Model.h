/*!
 * \file Model.hpp
 *
 * \author Salman Faraji
 * \date 10.2.2015
 *
 * This file contains the implementatrion of the model class.\
 * The Model class interfaces the model library (sdfast) with the code.
 */

#pragma once
#include "Basics.h"

#define AIR_N_U 38
#define AIR_N_STATE (2*AIR_N_U+1)
#define AIR_N_BODY 39
#define act_lim 200.0


/**************/


#define foot_length (0.135+0.065)
#define foot_width  (0.09)
#define hand_length (0.03)
#define hand_width  (0.03)

//! The body index of the head
#define body_head 	6
//! The body index of the left foot point
#define body_l_foot 	14
//! The link-frame offset of the left foot point
#define offset_l_foot 	(Cvector3(0.025, 0, -0.073) - Cvector3(0.0241, 0.0006, -0.0086))
//! The body index of the right foot point
#define body_r_foot 	22
//! The link-frame offset of the right foot point
#define offset_r_foot 	(Cvector3(0.025, 0, -0.073) - Cvector3(0.0241, -0.0006, -0.0086))
//! The body index of the left hand point
#define body_l_hand 	30
//! The link-frame offset of the left hand point
#define offset_l_hand 	(Cvector3(0, -0.025, -0.04) - Cvector3(0.0081, -0.0070, -0.0702))
//! The body index of the right hand point
#define body_r_hand 	38
//! The link-frame offset of the right hand point
#define offset_r_hand 	(Cvector3(0, 0.025, -0.04) - Cvector3(0.0081, 0.0070, -0.0702))
//! The body index of the base point
#define body_base 	    0
//! The link-frame offset of the base point
#define offset_base 	(Cvector3(0, 0, 0)  - Cvector3(0, 0, 0))

enum constraint_type    {CT_TRANSLATION = 0, CT_ROTATION, CT_FULL};

/************************************************************************/

extern "C"
{
    void icub_air_init(void);

    void icub_air_stab(double velin,
            double posin);

    void icub_air_printerr(FILE *fnum);

    void icub_air_clearerr(void);

    void icub_air_state(double timein,
             double qin[AIR_N_U+1],
             double uin[AIR_N_U]);

    void icub_air_pos(int body,
               double pt[3],
               double loc[3]);

    void icub_air_vel(int body,
               double pt[3],
               double velo[3]);

    void icub_air_acc(int body,
                double pt[3],
                double accel[3]);

    void icub_air_orient(int body,
              double dircos[3][3]);

    void icub_air_angvel(int body,
              double avel[3]);

    void icub_air_angacc(int body,
                double aacc[3]);

    void icub_air_trans(int frbod,
             double ivec[3],
             int tobod,
             double ovec[3]);

    void icub_air_getbtj(int joint,
              double btjout[3]);

    void icub_air_fmotion(double *time,
               double state[AIR_N_STATE],
               double dstate[AIR_N_STATE],
               double dt,
               double ctol,
               int *flag,
               double *errest,
               int *err);

    void icub_air_pointf(int body,
              double point[3],
              double force[3]);

    void icub_air_hinget(int joint,
              int axis,
              double torque);

    void icub_air_bodyt(int body,
             double torque[3]);

    void icub_air_mom(double lm[3],
               double am[3],
               double *ke);

    void icub_air_sys(double *mtoto,
               double cm[3],
               double icm[3][3]);

    void icub_air_getitj(int joint,
                double itjout[3]);

    void icub_air_itj(int joint,
             double itjin[3]);

    void icub_air_assemble(double time,
                  double state[AIR_N_STATE],
                  int lock[AIR_N_U],
                  double tol,
                  int maxevals,
                  int *fcnt,
                  int *err);

    void icub_air_initvel(double time,
                 double state[AIR_N_STATE],
                 int lock[AIR_N_U],
                 double tol,
                 int maxevals,
                 int *fcnt,
                 int *err);

    void icub_air_reac(double force[AIR_N_U][3],
              double torque[AIR_N_U][3]);

    /************************************************************************/

    void icub_air_equivht(double tau[AIR_N_U]);

    void icub_air_comptrq(double udotin[AIR_N_U],
                         double trqout[AIR_N_U]);

    void icub_air_fulltrq(double udotin[AIR_N_U],
                         double multin[AIR_N_U],
                         double trqout[AIR_N_U]);

    void icub_air_massmat(double mmat[AIR_N_U][AIR_N_U]);

    void icub_air_frcmat(double fmat[AIR_N_U]);

    void icub_air_multtrq(double multin[AIR_N_U],
                         double trqout[AIR_N_U]);

    void icub_air_rel2cart(int coord,
                          int body,
                          double point[3],
                          double linchg[3],
                          double rotchg[3]);

    void icub_air_reac(double force[39][3],
                        double torque[39][3]);

    void icub_air_getiner(int body,
                    double inerout[3][3]);

    /************************************************************************/

    void icub_air_pres(int joint,
                    int axis,
                    int presin);

    void icub_air_prespos(int joint,
                        int axis,
                        double prval);

    void icub_air_presvel(int joint,
                        int axis,
                        double prval);

    void icub_air_presacc(int joint,
                        int axis,
                        double prval);

    void icub_air_getht(int joint,
                        int axis,
                        double *torque);

    void icub_air_deriv(double oqdot[AIR_N_U+1],
                        double oudot[AIR_N_U]);

    void icub_air_getmass(int body,
                        double *massout);

    void icub_air_mult(double omults[AIR_N_U],
                    int *owrank,
                    int omultmap[AIR_N_U]);

    void icub_air_setudot(double iudot[AIR_N_U]);

    void icub_air_qdot(double oqdot[AIR_N_U+1]);

    //////////////////////////////////////////////////////////////////////
    void icub_air_dc2quat(double dircos[3][3],
        double *e1,
        double *e2,
        double *e3,
        double *e4);

    void icub_air_dc2ang(double dircos[3][3],
        double *a1,
        double *a2,
        double *a3);

    void icub_air_ang2dc(double a1,
        double a2,
        double a3,
        double dircos[3][3]);

    void icub_air_quat2dc(double ie1,
        double ie2,
        double ie3,
        double ie4,
        double dircos[3][3]);
}

/************************************************************************/

//! Converting quaternion to rotation matrix
Cmatrix     quat2dc(Cvector st);
//! Converting rotation matrix to euler angles
Cvector3    dc2ang(Cmatrix dc);
//! Converting euler angles to rotation matrix
Cmatrix     ang2dc(Cvector3 stang);
//! Converting rotation matrix to quaternion
Cvector     dc2quat(Cmatrix dc);
//! Converting quaternion to euler angles
Cvector3    quat2ang(Cvector st);
//! Converting euler angles to quaternion
Cvector     ang2quat(Cvector3 stang);
//! Generating a skew symmetric matrix from angular velocities
Cmatrix     skew_symmetric(Cvector3 in);
//! Getting the vector of angular velocities from a skew symmetric matrix
Cvector3    skew_symmetric_inv(Cmatrix &in);

/************************************************************************/

//! Generate a matrix to replace cross product: would be P(a) * q = a x q
Cmatrix make_cross_lh(Cvector a);
//! Quaternion cross product
Cvector quat_cross(Cvector a, Cvector b);
//! Quaternion triple cross product
Cvector quat_cross(Cvector a, Cvector b, Cvector c);
//! Normalize a quaternion
Cvector quat_normalize(Cvector in);
//! Inverse a quaternion
Cvector quat_inverse(Cvector in);
//! Dot product two quaternions
double quat_dot(Cvector a, Cvector b);
//! Another implementation of quaternion cross product
Cvector quat_mul(Cvector a, Cvector b);
//! adding two quaternions
Cvector quat_add(Cvector a, Cvector b);
//! subtracting two quaternions
Cvector quat_sub(Cvector a, Cvector b);
//! Finding the cosine between two quaternions
double quat_cos(Cvector a, Cvector b);
//! The norm of a quaternion
double quat_norm(Cvector in);
//! Calculate the angle from a quaternion (assuming axis/angle representation)
double quat_get_theta(Cvector in);
//! The exponential of a quaternion: e^q
Cvector quat_exp(Cvector in);
//! The log of a quaternion: ln(q)
Cvector quat_log(Cvector in);
//! The power of a quaternion
Cvector quat_pow(Cvector in, double t);
//! Slerp function using quaternion
Cvector quat_slerp(Cvector q0, Cvector q1, double t);
//! Creating a derivative for quaternion
Cvector quat_deriv(Cvector3 in);

/************************************************************************/


/*!
 * \class Model
 *
 * \brief Interfaces the model library with the code
 *
 * The exchange of variables and vectors as well as some space conversions are done in this class.
 * The aim is to convert all methods to cpp.
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
class Model
{
public:
    double time;
    Cvector qmin,qmax;
	Cvector ddqmin, ddqmax;
    Cvector taumin, taumax;


public:
    Model() {};
    ~Model() {};

    /*!
    * \brief Initialize the model library
    */
    void        init();

    /*!
    * \brief Set the state of the robot including positions and velocities
    * \param[in] tt current time
    * \param[in] in_state the state vector including quaternion 4th element in the end
    * \param[in] in_stated the derivative of the state vector without quaternion 4th element
    * \note See sdfast documentation for more details
    * \warning Resets everything
    */
    void        set_state(double tt, Cvector& in_state, Cvector& in_stated);

    /*!
    * \brief Set accelerations for dynamic-ready stage
    * \note See sdfast documentation for more details
    */
    void        set_acc(Cvector &acc);

    /*!
    * \brief Gravity compensation torque
    * \return torque vector
    * \note See sdfast documentation for more details
    */
    Cvector     get_equivht();

    /*!
    * \brief Torques that produce a desired acceleration
    * \param[in] udot desired acceleration vector
    * \return torque vector
    * \note See sdfast documentation for more details
    */
    Cvector     get_comptrq(Cvector& udot);

    /*!
    * \brief Torques that produce a desired acceleration, but also taking into account constraint forces
    * \param[in] udot desired acceleration vector
    * \param[in] mult constraint forces
    * \return torque vector
    * \note See sdfast documentation for more details
    */
    Cvector     get_fulltrq(Cvector& udot, Cvector& mult);

    /*!
    * \brief Returns the mass matrix
    * \return mass matrix
    * \note See sdfast documentation for more details
    */
    Cmatrix     get_massmat();

    /*!
    * \brief Similar to Model::get_equivht, but also including inertial forces. This is equivalent to gravitational and corriolis forces
    * \return torque vector
    * \note See sdfast documentation for more details
    */
    Cvector     get_frcmat();

    /*!
    * \brief Convert multiplier forces to hinge torques
    *
    * This function is in fact usef for sdfast models which have constraints to environment
    *
    * \param[in] mult multiplier forces
    * \return torque vector
    * \note See sdfast documentation for more details
    */
    Cvector     get_multtrq(Cvector & mult);

    /*!
    * \brief Get global position of a point
    * \param[in] body body link index
    * \param[in] pt link-frame relative position of the point wrt the link's CoM
    * \return global 3D position
    */
    Cvector3    get_pos(int body, Cvector3 pt);

    /*!
    * \brief Get global velocity of a point
    * \param[in] body body link index
    * \param[in] pt link-frame relative position of the point wrt the link's CoM
    * \return global 3D velocity
    */
    Cvector3    get_vel(int body, Cvector3 pt);

    /*!
    * \brief Get global acceleration of a point
    * \param[in] body body link index
    * \param[in] pt link-frame relative position of the point wrt the link's CoM
    * \return global 3D acceleration
    */
    Cvector3    get_acc(int body, Cvector3 pt);

    /*!
    * \brief Get global orientation of a point (rotation matrix)
    * \param[in] body body link index
    * \return global orientation (rotation matrix)
    */
    Cmatrix     get_orient(int body);

    /*!
    * \brief Get global orientation of a point (quaternion)
    * \param[in] body body link index
    * \return global 4D orientation (quaternion)
    */
    Cvector     get_orient_quat(int body);

    /*!
    * \brief Get global orientation of a point (euler angles)
    * \param[in] body body link index
    * \return global 3D orientation (euler angles)
    */
    Cvector3    get_orient_ang(int body);

    /*!
    * \brief Get global angular velocity of a point
    * \param[in] body body link index
    * \return global 3D angular velocity
    */
    Cvector3    get_angvel(int body);

    /*!
    * \brief Get global angular acceleration of a point
    * \param[in] body body link index
    * \return global 3D angular acceleration
    */
    Cvector3    get_angacc(int body);

    /*!
    * \brief  Transforms a vector from one frame into another
    * \param[in] frbod initial frame
    * \param[in] ivec the input vector
    * \param[in] tobod final frame
    * \return new vector
    * \note See sdfast documentation for more details
    */
    Cvector3    get_trans(int frbod, Cvector3& ivec, int tobod);

    /*!
    * \brief Get the jacobian of a point in the global frame
    *
    * Returns the jacobian for global velocity / angular velocity
    *
    * \param[in] body body link index
    * \param[in] pt link-frame relative position of the point wrt the link's CoM
    * \return Jacobian matrix
    * \note See sdfast documentation for more details
    */
    Cmatrix     get_jacob(int body, Cvector3 pt, constraint_type type);

    /*!
    * \brief Absolute linear and angular momentum for the system
    * \param[out] LM linear momentum
    * \param[out] AM angular momentum
    * \param[out] KE kinetic energy
    * \note See sdfast documentation for more details
    */
    void        get_mom(Cvector3 &LM, Cvector3 &AM, double * KE=NULL);

    /*!
    * \brief Total mass, CoM and inertia
    * \param[out] CM centrer of mass in global frame
    * \param[out] ICM equivalent inertia matrix
    * \param[out] MTOT total mass
    */
    void        get_sys(Cvector3 &CM, Cmatrix &ICM, double * MTOT);

    /*!
    * \brief Get total mass
    * \return total mass
    */
    double      get_mass();

    /*!
    * \brief Get reaction forces
    * \return forces and moments
    */
    void        get_reac(Cmatrix &F, Cmatrix &T);

    /*!
    * \brief Get body mass
    * \param[in] body body link index
    * \return body mass
    */
    double      get_mass(int body);

    /*!
    * \brief Get body inertia
    * \param[in] body body link index
    * \return body inertia
    * \warning in local frame
    */
    Cmatrix     get_inertia(int body);

    /*!
    * \brief Get global position of a CoM
    * \return global 3D position of CoM
    */
    Cvector3    get_cm();

    /*!
    * \brief Get global velocity of a CoM
    * \return global 3D velocity of CoM
    */
    Cvector3    get_cm_v();

    /*!
    * \brief Get global acceleration of a CoM
    * \return global 3D acceleration of CoM
    */
    Cvector3    get_cm_a();

    /*!
    * \brief Get global Jacobian of a CoM
    * \return global 3D Jacobian of CoM
    */
    Cmatrix     get_cm_J();

    /*!
    * \brief Check consistency of all functions, variables and jacobians
    *
    * This function aims to plot values using different methods to check if all values in the geometric model is correct
    *
    * \param[in] joint_pos position states including quaternion 4th element
    * \param[in] joint_vel derivative of state vector
    * \param[in] joint_acc double derivative of state vector
    * \note For debugging purpose
    */
    void        check_consistency(Cvector joint_pos, Cvector joint_vel, Cvector joint_acc);
};

