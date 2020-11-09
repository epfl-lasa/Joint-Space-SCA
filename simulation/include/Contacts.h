/*!
 * \file Contact_Manager.hpp
 *
 * \author Salman Faraji
 * \date 10.2.2015
 *
 * In this file a list of all the interesting contacts is defined
 */

#pragma once
#include "Model.h"

#define NUM_Contact 7
#define N_TASK 36
enum Contact_Name       {CN_CM=0, CN_LF, CN_RF, CN_LH, CN_RH, CN_TO, CN_HD};
enum Point_Status       {PS_NO_CONTROL=0, PS_CONTACTED, PS_FLOATING};


/*!
 * \class Geom_Point
 *
 * \brief Contains geometric information about a Cartesian point
 *
 * Includes position, velocity and acceleration
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
struct Geom_Point{
    Geom_Point() {pos=zero_v7; vel=zero_v6;}
    VectorXd pos;
    VectorXd vel;
};

/*!
 * \class Dyn_Point
 *
 * \brief Contains dynamic information about a Cartesian point.
 *
 * Note that one needs to define two instances of this class for a point which has both translational and rotational motion
 *
 * Includes:
 * - Jacobian and its derivative
 * - Measured, desired and estimated forces
 * - Costs for force and acceleration
 * - PID gains (K)
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
struct Dyn_Point
{
    Dyn_Point() {F_ref=zero_v3;  F_sens=zero_v3;  F_est=zero_v3;  slack=zero_v3;  F_cost=zero_v3;  K=zero_v3;  slack_cost=zero_v3; }
    // variables
    //! The corresponding Jacobian
    MatrixXd J;
    //! Desired force at the contact
    Vector3d F_ref;
    //! Measured force at the contact (if there is a sensor)
    Vector3d F_sens;
    //! Estimated force, coming out of inverse dynamics (should ideally be equal to Dyn_Point::F_ref if Dyn_Point::F_cost is high)
    Vector3d F_est;
    //! Estimated slack, coming out of inverse dynamics (should ideally be equal to zero if Dyn_Point::F_cost is high)
    Vector3d slack;

    //constants
    //! The cost of force tracking in inverse dynamics. If high, then Dyn_Point::F_est tends to Dyn_Point::F_ref.
    Vector3d F_cost;
    //! The PID gains for controlling the joint in Cartesian space. ** Note the notation is P-D-I in the 3D vector
    Vector3d K;
    //! The slack cost of the desired Cartesian acceleration Dyn_Point::acc_ref in the inverse dynamics. If low, Dyn_Point::acc_ref is not realized in the output.
    Vector3d slack_cost;
};

/*!
 * \class Contact
 *
 * \brief Contains basic information about a contact
 *
 * In this class we include all the information about a contact point in the robot.
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
class Contact
{
public:
    Contact_Name name;

    //! The Cartesian point is located on which body of the robot
    int body;

    //! The offset of the point wrt. the body mass center
    Vector3d offset;

    //! Determines if this contact is translational, rotational or both
    constraint_type contact_type;

    //! Determines if the contact is active, deactive
    Point_Status status;

    //! If this cpntact is used for odometery
    bool used_for_odometry;

    //! Initial position
    Geom_Point init_p;

    //! Actual position
    Geom_Point p;

    //! Reference position
    Geom_Point ref_p;

    //! Translational dynamnics information
    Dyn_Point T;

    //! Rotational dynamnics information
    Dyn_Point R;

    //! The Center of Pressure of the joint, calculated in perception level
    Vector3d cop;

    //! Contact frame X direction
    Vector3d n1;

    //! Contact frame Y direction
    Vector3d n2;

    //! Contact frame Z direction (should be the contact normal used to calculate the friction polyhedral)
    Vector3d n3;

    //! Contact length (along X direction), approximated with rectangle if having a surface
    Vector3d w_x;

    //! Contact width (along Y direction), approximated with rectangle if having a surface
    Vector3d w_y;

    //! Coefficient of translational friction
    double mu;

    //! Coefficient of rotational friction, if having a surface
    double muR;

    // Constructor, Destructor
    Contact();
    ~Contact();

    /*!
    * \brief initializing a contact with most important information
    *
    * \param[in] Name the contact name among the list of ::Contact_Name
    * \param[in] Body the index of the body link on the robot
    * \param[in] Offset The offset of the point with respect to the body link, expressed in the link coordinates frame
    * \param[in] Contact_type the type of this contact (and number of constraints) among the list of ::constraint_type
    * \param[in] Status The control policy for this point chosen from ::Point_Status
    * \param[in] KP the translational PID gains
    * \param[in] KO the rotational PID gains
    * \note if a point is mearly translational, the the geometry, rotational friction and cop will be ignored
    * \warning First calls Contact::initialize_contact and then overrides all the values
    */
    void init(  Contact_Name Name, 
				int Body, 
				Vector3d Offset,
				constraint_type Contact_type, 
				Point_Status Status, 
				double length,
				double width);

    /*!
    * \brief Set all variables to the default values
    * \warning Look at the implementation for more details
    */
    void initialize_contact();
};

/*!
 * \class Contact_Manager
 *
 * \brief The list of contact points as well as updating and plotting functions
 *
 * [detailed description]
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
class Contact_Manager
{
public:

    //! The main vector of Cartesian end effectors
    std::vector<Contact> EE;

    //! A copy of the robot's model
    Model model;

	// check true if you use COM commands instead of pelvis
	bool ifCoM;

    // Constructor, destructor
    Contact_Manager();
    ~Contact_Manager();

    /*!
    * \brief Record initial positions
    *
    * In the first time the controller is called, this function records the geometric status of all the contacts.
    *
    * \warning sets velocities and accelerations to zero
    */
    void save_initial_contacts();

    /*!
    * \brief Update information for all contacts after perception is done
    *
    * Since sdfast does not calculate the derivative of the Jcobian, we use a sample joint acceleration
    * vector to calculate Cartesian accelerations and the subtract Jacobian * dq. Note that the term
    * Dyn_Point::dJdq is independent of accelerations. Therefore a zero acceleration vector already works.
    */
    void update_kinematics();

    /*!
    * \brief Returns size of the array
    */
    unsigned int size();

    /*!
    * \brief Indexing function for the array of contacts
    */
    Contact &operator [](int n) { return EE.data()[n];}

    /*!
    * \brief Calculates the summation of all jacobian transpose times contact forces
    *
    * \return VectorXd summation of all external forces in the Equation om Motion
    * \note Used for checking purposes
    * \warning after inverse dynamics calculations
    */
    VectorXd get_mult();

	/*!
    * \brief Calculates the summation of all jacobian transpose times virtual contact forces
    *
    * \return VectorXd summation of all external forces in the Equation om Motion
    * \note Used for checking purposes
    * \warning after inverse dynamics calculations
    */
    VectorXd get_virtual_mult();

    /*!
    * \brief Control a Cartesian point to the desired position/velocity/acceleration
    *
    * Combines Contact_Manager::control_position and Contact_Manager::control_orientation
    *
    * \param[in] index the contact index in the list of ::Contact_Name
    * \param[in] ref_pos the desired 3D position/velocity/acceleration
    * \param[in] ref_ori the desired 3D orientaion/velocity/acceleration
    */
    void control(Contact_Name i, Geom_Point ref_pos);

    /*!
    * \Define Cartesian tasks
    */
    void load_tasks(VectorXd com, VectorXd lf, VectorXd rf, VectorXd lh, VectorXd rh);
    void load_tasks(VectorXd com, VectorXd lf, VectorXd rf, VectorXd hands);

    /*!
    * \Tasks costs in inverse kinematics
    */
    void task_costs();

    /*!
    * \brief Printing all estimated forces Dyn_Point::F_est after inverse dynamics
    */
    void print_forces();

	/*!
    * \brief Printing all inverse kinematics errors
    */
	void print_IK_errors();
};

