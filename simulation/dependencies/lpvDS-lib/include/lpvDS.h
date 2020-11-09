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

#include <stdlib.h>
#include <string>
#include "eigen3/Eigen/Dense"
#include "utils.h"

using namespace std;
using namespace Eigen;

const double PI_ = 3.14159265358979323846264338327950288419716939937510;

class lpvDS
{	
private:

        int 		K_;
        int 		M_;
        VectorXd	gamma_;
		double 		*Prior_;
		VectorXd 	*Mu_;
		MatrixXd 	*Sigma_;
		MatrixXd 	*A_Matrix_;

public:             

        lpvDS(const char  *path_dims);
        lpvDS(const char  *path_dims, const char  *path_prior,const char  *path_mu,const char  *path_sigma, const char  *path_A);
        lpvDS(const int K, const int M, const MatrixXd Priors_fMatrix, const MatrixXd Mu_fMatrix, const MatrixXd Sigma_fMatrix, const MatrixXd A_fMatrix );
        lpvDS(const int K, const int M, const vector<double> Priors_vec, const vector<double> Mu_vec, const vector<double> Sigma_vec, const vector<double> A_vec);
        ~lpvDS(void);

        void        initialize_A(const char  *path_A);
        void        initialize_gamma(const char  *path_prior, const char  *path_mu, const char  *path_sigma);

        MatrixXd         compute_A(VectorXd xi);
        VectorXd         compute_f(VectorXd xi, VectorXd att);
        VectorXd         compute_gamma(VectorXd xi);

private:
        void        setup_params();
        void        initialize_A(const MatrixXd fMatrix);
        void        initialize_Priors(const MatrixXd fMatrix);
        void        initialize_Mu(const MatrixXd fMatrix);
        void        initialize_Sigma(const MatrixXd fMatrix);

        void        initialize_Priors_vec(const vector<double> Priors_vec);
        void        initialize_Mu_vec(const vector<double> Mu_vec);
        void        initialize_Sigma_vec(const vector<double> Sigma_vec);
        void        initialize_A_vec(const vector<double> A_vec);

        double 		GaussianPDF(VectorXd x,VectorXd Mu,MatrixXd Sigma);
        fileUtils   fileUtils_;
        void 	 	ERROR();
};





