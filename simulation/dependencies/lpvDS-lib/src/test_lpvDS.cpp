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


#include <stdio.h>
#include <fstream>
#include <time.h>
#include "eigen3/Eigen/Dense"
#include "lpvDS.h"
#include "utils.h"

using namespace std;

int
main (int argc, char **argv)
{
    string path_model  = "/home/sfaraji/Dropbox/LASA/lpvDS-lib/models/coManip-DS-1/";
    string path_dim    = path_model +  "dimensions";
    string path_Priors = path_model +  "Priors";
    string path_Mu     = path_model +  "Mu";
    string path_Sigma  = path_model +  "Sigma";
    string path_A      = path_model +  "A_k";

    /* Instantiate an LPV-DS class Option 1 */
    cout << "Initialization Test 1: " << endl;
    lpvDS lpvDS_test1(path_dim.c_str());
    lpvDS_test1.initialize_gamma(path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str());
    lpvDS_test1.initialize_A(path_A.c_str());

    /* Instantiate an LPV-DS class Option 2 */
    cout << "Initialization Test 2: " << endl;
    lpvDS lpvDS_test2 (path_dim.c_str(), path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str(), path_A.c_str());

    /* Instantiate an LPV-DS class Option 3 */
    cout << "Initialization Test 3: " << endl;
    fileUtils fileUtils_;
    MatrixXd dim, Priors, Mu, Sigma, A;
    dim     = fileUtils_.readMatrix(path_dim.c_str());
    Priors  = fileUtils_.readMatrix(path_Priors.c_str());
    Mu      = fileUtils_.readMatrix(path_Mu.c_str());
    Sigma   = fileUtils_.readMatrix(path_Sigma.c_str());
    A       = fileUtils_.readMatrix(path_A.c_str());
    int K = (int)dim(0,0);
    int M = (int)dim(1,0);
    lpvDS lpvDS_test3 (K, M, Priors, Mu, Sigma, A);

    /* Testing the LPV-DS on training data from MATLAB */
    cout << "Testing Accuracy of model..." << endl;
    string path_att    = path_model +  "attractor";
    string path_data   = path_model +  "Data";
    string path_xi_dot = path_model +  "xi_dot";
    MatrixXd attractor, Data, xi_dot;
    attractor = fileUtils_.readMatrix(path_att.c_str());
    Data      = fileUtils_.readMatrix(path_data.c_str());
    xi_dot    = fileUtils_.readMatrix(path_xi_dot.c_str());
    int samples = Data.cols();

    /* Fill in attractor */
    VectorXd att; att.resize(M);
    att = attractor.col(0);

    /* Fill in reference trajectories */
    MatrixXd xi_ref(M,samples);
    for (int i=0; i<M; i++)
        xi_ref.row(i) = Data.row(i);


    /* Compute estimated velocities from model */
    VectorXd xi_ref_test(M);
    VectorXd xi_dot_test(M);
    VectorXd xi_dot_mat(M);
    VectorXd xi_dot_error(M);
    MatrixXd A_matrix(M,M);
    VectorXd  est_error(samples);
    for (int i=0; i<samples; i++){

        /* Computing desired velocity */
        xi_ref_test = xi_ref.col(i);
        A_matrix = lpvDS_test2.compute_A(xi_ref_test);
        xi_dot_test = A_matrix*(xi_ref_test - att);
		cout << xi_ref_test.transpose() << "                 " << xi_dot_test.transpose() << endl;

        /* Computing error between this estimate and MATLAB */
        xi_dot_mat = xi_dot.col(i);
        xi_dot_error =  xi_dot_test-xi_dot_mat;
        est_error[i] = xi_dot_error.norm();

    }

    /* Stats on Estimation error between MATLAB-C++ model */
    cout << "Average Estimation Error" << " (Norm of predicted Matlab and C++ velocities): " << est_error.mean() << endl;
    return 0;
}
