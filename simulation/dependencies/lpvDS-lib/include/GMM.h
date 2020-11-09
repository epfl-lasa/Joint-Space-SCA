/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Sina Mirrazavi and Nadia Figueroa
 * email:   {sina.mirrazavi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the EU project Cogimon H2020-ICT-23-2014.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */



#ifndef GMM_H
#define GMM_H

#include <stdlib.h>
#include <string>
#include "eigen3/Eigen/Dense"
#include "utils.h"


using namespace std;
using namespace Eigen;

const double PI = 3.14159265358979323846264338327950288419716939937510;

class GMM
{
    private:

        int 		K_;
        int 		M_;
        double 		*Prior_;
        VectorXd 	*Mu_;
        MatrixXd 	*Sigma_;
        double      threshold;

    public:

        void        initialize(int Num_Com,int Num_state);
        void        initialize_GMM(const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_,const char *path_threshold);
        double      PDF(VectorXd X);

    private:

        double 		GaussianPDF(VectorXd x,VectorXd Mu,MatrixXd Sigma);
        fileUtils   fileUtils_;
        void 	 	ERROR();

};

#endif // GMM_H
