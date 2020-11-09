#ifndef NN_EVAL_H
#define NN_EVAL_H


#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <vector>
#include <eigen3/Eigen/Dense>

struct nn_state{
    Eigen::MatrixXd w1;
    Eigen::MatrixXd w2;   
    Eigen::MatrixXd w3;   
    Eigen::MatrixXd w4;   
    Eigen::VectorXd b1;
    Eigen::VectorXd b2;
    Eigen::VectorXd b3;
    Eigen::VectorXd b4;
};

nn_state nn_model_state_load(std::string fname);
void calcGamma_nn_manual(Eigen::VectorXd point, nn_state &model, double &Gamma, Eigen::VectorXd &GammaGrad);

#endif
