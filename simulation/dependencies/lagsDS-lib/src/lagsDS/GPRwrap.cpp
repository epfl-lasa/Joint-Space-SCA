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


#include "GPRwrap.h"

GPRwrap::GPRwrap(){
    cout << "Constructed GPRwrap class" << endl;
}

GPRwrap::GPRwrap(string& f_GPRwrapmodel)
{
    loadModel(f_GPRwrapmodel);
}

void GPRwrap::loadModel(string &f_GPRwrapmodel){

    cout << "GPR Model: " << f_GPRwrapmodel << endl;
    ifstream fin(f_GPRwrapmodel.c_str());
    unsigned int    d, s;

    // Load GPRwrap Model
    fin >> GPRModel_.D
        >> GPRModel_.N_train
        >> GPRModel_.length_scale
        >> GPRModel_.sigma_f
        >> GPRModel_.sigma_n;

    cout << "model.D:            " << GPRModel_.D << endl
         << "model.N_train:      " << GPRModel_.N_train << endl
         << "model.length_scale: " << GPRModel_.length_scale << endl
         << "model.sigma_f:      " << GPRModel_.sigma_f << endl
         << "model.sigma_n:      " << GPRModel_.sigma_n << endl;

    GPRModel_.y_train = zeros<colvec>(GPRModel_.N_train);
    GPRModel_.x_train = zeros<mat>(GPRModel_.D, GPRModel_.N_train);

    for( s=0; s<GPRModel_.N_train; s++ ){
        fin >> GPRModel_.y_train(s);
    }

    for( d=0; d<GPRModel_.D; d++ ){
        for( s=0; s<GPRModel_.N_train; s++ ){
            fin >> GPRModel_.x_train(d,s);
        }
    }
    fin.close();

    /* Instantiate GPR Model with parameters read from text file*/
    GPR_.reset(new GaussianProcessRegression<double>(GPRModel_.D, 1));
    GPR_->SetHyperParams(GPRModel_.length_scale, GPRModel_.sigma_f, GPRModel_.sigma_n);

    /* Add Trainig Data (could do batch version, but need train data to be Eigen*/
    vecEig x_train_i(GPRModel_.D); vecEig y_train_i(1);
    for(unsigned int i=0; i<GPRModel_.N_train; i++){        
        
        /* Converting Training data for GPR Class */
        arma2eigen(GPRModel_.x_train.col(i), x_train_i);
        y_train_i(0)   = GPRModel_.y_train(i);

        /* Converting Training data for GPR Class */        
        GPR_->AddTrainingData(x_train_i,y_train_i);

        /* For Debugging */
        // std::cout << "x_i: " << x_train_i << std::endl << "y_i:" << y_train_i << std::endl;
    }

}

vecEig GPRwrap::regress_GPR(vecEig x_test){
    vecEig y_test(1);
    y_test = GPR_->DoRegression(x_test);
    return y_test;
}


void GPRwrap::arma2eigen(arma::vec x_in, vecEig& x_out){
    x_out.resize(x_in.size());
    for (unsigned int i=0;i < x_in.size(); i++)
        x_out(i) = x_in(i);
}


