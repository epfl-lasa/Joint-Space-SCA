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
#include "lagsDS.h"
#include "utils.h"

using namespace std;

int
main (int argc, char **argv)
{

    /* Path to the parameters text files for LAGS-DS */
    string path_model  = "../models/iCub-Narrow-Passage-LAGS/";
//    string path_model  = "../models/iCub-Object-Conveyor-LAGS/";
    string path_dim      = path_model +  "dimensions";
    string path_Priors   = path_model +  "Priors";
    string path_Mu       = path_model +  "Mu";
    string path_Sigma    = path_model +  "Sigma";
    string path_Ag       = path_model +  "A_g";
    string path_Al       = path_model +  "A_l";
    string path_Ad       = path_model +  "A_d";
    string path_att_g    = path_model +  "att_g";
    string path_att_l    = path_model +  "att_l";
    string path_w_l      = path_model +  "w_l";
    string path_b_l      = path_model +  "b_l";
    string path_scale    = path_model +  "scale";
    string path_b_g      = path_model +  "b_g";
    string path_gpr      = path_model +  "GPR_model.txt";


    /* Instantiate an LAGS-DS class Option 1 */
    cout << "Initialization Test 1: " << endl;
    lagsDS lagsDS_test1(path_dim.c_str());
    lagsDS_test1.initialize_gamma(path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str());
    lagsDS_test1.initialize_Ag(path_Ag.c_str());
    lagsDS_test1.initialize_Al(path_Al.c_str());
    lagsDS_test1.initialize_Ad(path_Ad.c_str());
    lagsDS_test1.initialize_local_params(path_att_l.c_str(), path_w_l.c_str(), path_b_l.c_str(), path_scale.c_str());
    lagsDS_test1.initialize_alpha_gpr(path_gpr.c_str());
    lagsDS_test1.initialize_bg(path_b_g.c_str());

    /* Instantiate an LAGS-DS class Option 2 */
    cout << endl <<"Initialization Test 2: " << endl;
    lagsDS lagsDS_test2 (path_dim.c_str(), path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str(), path_Ag.c_str(), path_Al.c_str(), path_Ad.c_str(),
                         path_att_l.c_str(),path_w_l.c_str(), path_b_l.c_str(), path_scale.c_str(), path_gpr.c_str(),path_b_g.c_str());


    /* Instantiate an LAGS-DS class Option 3 */
    cout << endl << "Initialization Test 3: " << endl;
    fileUtils fileUtils_;
    MatrixXd dim, Priors, Mu, Sigma, Ag, Al, Ad, att_l, w_l, b_l, scale, b_g;
    dim     = fileUtils_.readMatrix(path_dim.c_str());
    Priors  = fileUtils_.readMatrix(path_Priors.c_str());
    Mu      = fileUtils_.readMatrix(path_Mu.c_str());
    Sigma   = fileUtils_.readMatrix(path_Sigma.c_str());
    Ag      = fileUtils_.readMatrix(path_Ag.c_str());
    Al      = fileUtils_.readMatrix(path_Al.c_str());
    Ad      = fileUtils_.readMatrix(path_Ad.c_str());
    att_l   = fileUtils_.readMatrix(path_att_l.c_str());
    w_l     = fileUtils_.readMatrix(path_w_l.c_str());
    b_l     = fileUtils_.readMatrix(path_b_l.c_str());
    scale   = fileUtils_.readMatrix(path_scale.c_str());
    b_g     = fileUtils_.readMatrix(path_b_g.c_str());
    int K = (int)dim(0,0);
    int M = (int)dim(1,0);
    lagsDS lagsDS_test3 (K, M, Priors, Mu, Sigma, Ag, Al, Ad, att_l, w_l, b_l, scale, path_gpr.c_str(),b_g);


    /* Testing the LAGS-DS on training data from MATLAB */
    cout << endl << "Testing Accuracy of model..." << endl;
    string path_data        = path_model +  "Data";
    string path_xi_dot_g    = path_model +  "xi_dot_g";
    string path_xi_dot_l    = path_model +  "xi_dot_l";
    string path_xi_alpha    = path_model +  "xi_alpha";
    string path_xi_dot_lags = path_model +  "xi_dot_lags";
    MatrixXd attractor, Data, xi_dot_g, xi_dot_l, xi_alpha, xi_dot_lags;

    attractor    = fileUtils_.readMatrix(path_att_g.c_str());
    Data         = fileUtils_.readMatrix(path_data.c_str());
    xi_dot_g     = fileUtils_.readMatrix(path_xi_dot_g.c_str());
    xi_dot_l     = fileUtils_.readMatrix(path_xi_dot_l.c_str());
    xi_alpha     = fileUtils_.readMatrix(path_xi_alpha.c_str());
    xi_dot_lags  = fileUtils_.readMatrix(path_xi_dot_lags.c_str());
    int samples  = Data.cols();

    /* Fill in attractor */
    VectorXd att; att.resize(M);
    att = attractor.col(0);

    /* Fill in reference trajectories */
    MatrixXd xi_ref;  xi_ref.resize(M,samples);
    for (int i=0; i<M; i++)
        xi_ref.row(i) = Data.row(i);

    /* Compute estimated velocities from model */

    /* For Global Component Estimates */
    VectorXd xi_ref_test;  xi_ref_test.resize(M);
    VectorXd xi_dot_test;  xi_dot_test.resize(M);
    VectorXd xi_dot_mat;   xi_dot_mat.resize(M);
    MatrixXd A_matrix;     A_matrix.resize(M,M);

    /* For Local Component Estimates */
    VectorXd xi_dot_test_l;  xi_dot_test_l.resize(M);
    VectorXd xi_dot_mat_l; xi_dot_mat_l.resize(M);


    /* For LAGS Estimates */
    VectorXd xi_dot_test_lags;  xi_dot_test_lags.resize(M);
    VectorXd xi_dot_mat_lags; xi_dot_mat_lags.resize(M);

    /* For Error Estimates */
    VectorXd xi_dot_error;  xi_dot_error.resize(M);
    VectorXd est_error_1; est_error_1.resize(samples);
    VectorXd est_error_2; est_error_2.resize(samples);
    VectorXd est_error_3; est_error_3.resize(samples);
    VectorXd est_error_4; est_error_4.resize(samples);
    VectorXd est_error_5; est_error_5.resize(samples);
    double alpha_mat(0.0), alpha_test(0.0);

    lagsDS_test3.set_att_g(att);
    for (int i=0; i<samples; i++){

        /* Computing desired velocity */
        xi_ref_test  = xi_ref.col(i);
        xi_dot_mat   = xi_dot_g.col(i);

        /* Computing error between this estimate (using A-matrix) and MATLAB */
        A_matrix       =  lagsDS_test3.compute_Ag(xi_ref_test);
        xi_dot_test    =  A_matrix*(xi_ref_test - att);
        xi_dot_error   =  xi_dot_test-xi_dot_mat;
        est_error_1[i] =  xi_dot_error.norm();

        /* Computing error between this estimats (using f_g) and MATLAB */
        xi_dot_test    =  lagsDS_test3.compute_fg(xi_ref_test);
        xi_dot_error   =  xi_dot_test-xi_dot_mat;
        est_error_2[i] =  xi_dot_error.norm();


        /* Computing error between this estimats (using f_l) and MATLAB */
        xi_dot_mat_l   =  xi_dot_l.col(i);
        xi_dot_test_l  =  lagsDS_test3.compute_fl(xi_ref_test);
        xi_dot_error   =  xi_dot_test_l - xi_dot_mat_l;
        est_error_3[i] =  xi_dot_error.norm();

        /* Computing error between this estimate (using alpha) and MATLAB */
        alpha_mat    = xi_alpha.coeff(i,0);
        alpha_test   = lagsDS_test3.compute_alpha(xi_ref_test);
        est_error_4[i] = fabs(alpha_test-alpha_mat);

        /* Computing error between this estimate (using f-lags) and MATLAB */
        xi_dot_mat_lags   =  xi_dot_lags.col(i);
        xi_dot_test_lags  =  lagsDS_test3.compute_f(xi_ref_test);
        xi_dot_error      =  xi_dot_test_lags - xi_dot_mat_lags;
        est_error_5[i]    = xi_dot_error.norm();

    }

    /* Stats on Estimation error between MATLAB-C++ model */
    cout << "Average Estimation Error for Global Componnet f_g(x)" << " (Norm of predicted Matlab and C++ velocities): " << est_error_1.mean() << endl;
    cout << "Average Estimation Error for Global Componnet f_g(x)" << " (Norm of predicted Matlab and C++ velocities): " << est_error_2.mean() << endl;
    cout << "Average Estimation Error for Local Componnet f_l(x)"  << " (Norm of predicted Matlab and C++ velocities): " << est_error_3.mean() << endl;
    cout << "Average Estimation Error for Activation Function  alpha(x)"  << " (Absolute error of predicted Matlab and C++ alphas): " << est_error_4.mean() << endl;
    cout << "Average Estimation Error for LAGS-DS  f(x)"  << " (Norm of predicted Matlab and C++ velocities): " << est_error_5.mean() << endl;
    return 0;
}
