#include "MedianFilter.h"

using namespace Eigen;
using namespace std;

void MedianFilter::init(int N, int win)
{
    window = win;
	index = window-1;
    first_input = true;
    n = N;
    for(int i=0;i<window;i++)
        x.push_back(VectorXd::Zero(n));
}

VectorXd MedianFilter::update(VectorXd X)
{
    assert(X.size()==n);
    if(first_input)
    {
        for(int i=0;i<window;i++)
            x[i] = X;
        first_input = false;
    }
    else
    {
        index = (index+1)%window;
        x[index] = X;
    }
    VectorXd Y = X;
    for(int i=0;i<n;i++)
    {
        VectorXd win(window+1);
        for(int j=0;j<window;j++)
            win[j] = x[j][i];
        std::sort(&win[0], &win[window]);
        Y[i] = win[std::ceil(window/2)];
    }
    return Y;
}

bool MedianFilter::if_frozen_signal()
{
    VectorXd Y = x[0];
    for(int i=1;i<window;i++)
        if(!Y.isApprox(x[i]))
            return false;
    return true;
}
