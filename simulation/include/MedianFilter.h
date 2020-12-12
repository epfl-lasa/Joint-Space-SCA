#pragma once

#include <Eigen/Dense>
#include "utils.h"
//using namespace std;
//! A median filter function
class MedianFilter
{
public:

    int n;
	int window;
    vector<VectorXd> x;
    int index;
    bool first_input;

    void init(int N, int win);
    VectorXd update(VectorXd X);
    bool if_frozen_signal();
};
