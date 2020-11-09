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
#include "gaussian_process_regression/gaussian_process_regression.h"
#include "armadillo"
#include "GPRwrap.h"

using namespace arma;
using namespace std;

struct GPRTestData{
    int D;         // Input dimensions
    int M;         // samples
    mat x;         // input
    vec y;         // output
};

void loadTestData(string& f_GPRTestData, GPRTestData& data)
{

    cout << "\nGPR Testing File: " << f_GPRTestData << endl;
    ifstream fin(f_GPRTestData.c_str());
    unsigned int d , s;

    // Load GPRTest Data
    fin >> data.D
        >> data.M;

    cout << "data.D: " << data.D << endl
         << "data.M: " << data.M << endl;

    data.x        = zeros<mat>(data.D, data.M);
    data.y        = zeros<colvec>(data.M);

    for( d=0; d<data.D; d++ ){
        for( s=0; s<data.M; s++ ){
            fin >> data.x(d,s);
        }
    }
    for( s=0; s<data.M; s++ ){
           fin >> data.y(s);
    }
    fin.close();
}

int
main (int argc, char **argv)
{
    /* These are the parameters I have to load from a file */
    int D_in(2); double length_scale (0.25), sigma_f(1.0), sigma_n (0.01);

    cout << "*********** Starting Random test ***********" << endl;
    // input_dim and output_dim are integers specifying the dimensionality of input and output data respectively
    GaussianProcessRegression<double> myGPR(D_in, 1);

    // the hyperparameters are float values specifying length-scale, signal variance and observation noise variance
    myGPR.SetHyperParams(length_scale, sigma_f, sigma_n);
    Eigen::VectorXd train_input(D_in);
    Eigen::VectorXd train_output(1);

    // add some training data
    int n_train = 100;
    for(int k=0; k<n_train; k++){
        train_input.setRandom();
        train_output.setRandom();
        myGPR.AddTrainingData(train_input,train_output);
    }

    // get output at a testing point
    Eigen::VectorXd test_input(D_in);
    Eigen::VectorXd test_output(1);
    test_input.setRandom();
    test_output = myGPR.DoRegression(test_input);
    cout << "**************** finished ******************" << endl << endl;



    cout << "*********** Testing Class Wrapper with Loaded Model/Test-Data ***********" << endl;
    /* Start testing my class wrapper */
    // string model_fileName = "../models/iCub-Narrow-Passage-LAGS/GPR_model.txt";
    // string data_fileName = "../models/iCub-Narrow-Passage-LAGS/GPR_data.txt";

    string model_fileName = "../models/iCub-Narrow-Passage-LAGS/Theta_GPR_model.txt";
    string data_fileName = "../models/iCub-Narrow-Passage-LAGS/Theta_GPR_data.txt";


    /* Instatiate Class and Load Model Parameters */
    GPRwrap gpr_(model_fileName);

    GPRTestData data_;
    loadTestData(data_fileName,data_);

    /* Testing the GPR Model on training data from MATLAB */
    cout << "Testing Accuracy of model..." << endl;


    /* Fill in reference trajectories */
    Eigen::VectorXd x_test(data_.D);
    Eigen::VectorXd y_test(1);
    vec             y_diff = zeros<vec>(data_.M);
    vec             y_time = zeros<vec>(data_.M);


    for (unsigned int  i=0;i<data_.M;i++){
        for(unsigned int d=0; d<data_.D; d++){
            x_test(d)  = data_.x(d,i);
        }

        clock_t t;
        t = clock();
        {
           y_test = gpr_.regress_GPR(x_test); 

        }
        t = clock() - t;

        y_time(i)  = ((float)t)/CLOCKS_PER_SEC;
        y_diff(i)  = fabs(data_.y(i) - y_test(0));
    }

    cout << "\n---Testing y regressed with Eigen Inputs---\n";
    cout << "Average y Numerical Error: "
         << sum(y_diff)/(double)data_.M << endl;
    cout << "Average Time y Calculation-: "
         << sum(y_time)/(double)data_.M << endl;
    return 0;
}

/* To write shit to file */
//    ofstream myfile;
//    myfile.open ("gpr_test_results.txt");
//    myfile << data_.y(i) << " " << y_test(0) << endl;
//    myfile.close();

