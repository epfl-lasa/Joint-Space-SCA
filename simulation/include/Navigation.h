#pragma once

#include "Basics.h"
#include "lpvDS.h"
#include "utils.h"
#include "GPRwrap.h"

class Navigation
{
public:

    

	Navigation();
	~Navigation() {};
	Vector3d linear_DS(VectorXd target);
	Vector3d nonlinear_DS(VectorXd target, VectorXd robot_state);

   /* Variables for non-linear DS */
   VectorXd learned_target;
   std::unique_ptr<lpvDS> LPV_DS_;
   /* Variables for orientation */
//    std::unique_ptr<GPRwrap> GPR_ORIENT_;

};
