/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Nadia Figueroa
 * email:   nadia.figueroafernandez@epfl.ch
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



#ifndef GPRwrap_H
#define GPRwrap_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <memory>
#include <fstream>
#include "armadillo"
#include "eigen3/Eigen/Dense"
#include "gaussian_process_regression/gaussian_process_regression.h"

using namespace std;
using namespace arma;

typedef Eigen::VectorXd vecEig;


/* GPRwrap: This structure holds the parameters of an RBF-SVM
 * D:        Datapoint Dimension
 * N_train:  # of training data
 * x_train:  Training input
 * y_train:  Training output
 * Hyper-parameters 
 *   float length_scale;
 *   float sigma_f;
 *   float sigma_n;
*/

struct GPRModel{
    unsigned int D;
    unsigned int N_train;
    vec   y_train;
    mat   x_train;
    double length_scale;
    double sigma_f;
    double sigma_n;

};

/* GPRwrap: This class computes the following function from an SMVGradModel
 *       y      = E(GPR(x))
*/


class GPRwrap
{
    private:

        GPRModel GPRModel_;
        double y;
        std::unique_ptr<GaussianProcessRegression<double>> GPR_;

    public:

        GPRwrap();
        GPRwrap(string& f_GPRmodel);


        void        loadModel(string& f_GPRmodel);
        vecEig      regress_GPR(vecEig x);
        void        arma2eigen(arma::vec x_in, vecEig& x_out);

};

#endif // GPRwrap_H
