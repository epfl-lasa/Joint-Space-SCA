/*!
 * \file Basics.hpp
 *
 * \author Salman Faraji
 * \date 10.2.2015
 *
 * The basic definitions, file inclusions and math functions
 */

#pragma once

// C++ standard headers
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <fstream>
#include <vector>
#include "vector"
#include <unistd.h>
#include <string>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <termios.h>
#include <sys/time.h>
#include <Eigen/Dense>


#define inf 1e20

// Eigen redefinition ///////////////////////////////////////////////////////////////////////////

typedef Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic , Eigen::RowMajor > Cmatrix;
typedef Eigen::VectorXd Cvector;

typedef Eigen::Vector3d Cvector3;
typedef Eigen::Vector4d Cvector4;

#define zero_v3 Cvector3(0.0,0.0,0.0)
#define zero_quat Cvector4(0.0,0.0,0.0,1.0)

#define wide    Eigen::IOFormat(3, 0, " ", " ", " ", " ", "\t", "\n")
#define nice    Eigen::IOFormat(1, Eigen::DontAlignCols, "\t", "\n", " ", " ", " ", "\n")
#define defaule Eigen::IOFormat()
// Eigen redefinition ///////////////////////////////////////////////////////////////////////////

/*!
 * \class Exception
 *
 * \brief In case of run-time error, this functrion could be called.
 *
 * It has the possibility to show an error message
 *
 * \author Salman Faraji
 * \date 10.2.2015
 */
class Exception
{
public:
    const char* msg;
    Exception(const char* arg)
    : msg(arg)
    {printf("Error: %s \n", arg);}
};

//! Cartesian space directions, used to access vectors
enum Dir_Axis {X_DIR=0, Y_DIR, Z_DIR};

//! Thresholding a value to certain bounds
/*!
 * \param[in] in the input value
 * \param[in] up the upper bound
 * \param[in] down the lower bound
 * \param[out] double truncated
 */
double truncate_command(double in, double up, double down);

//! A zero-mean guassian random generator
/*!
 * \param[in] sigma the standard deviation
 * \param[out] double random
 */
double gauss_random(double sigma);

//! Copying a matrix to a double* by scanning columns
void load_matrix(double * dest, Cmatrix in);

//! Copying a vector to a double*
void load_cvector(double * dest, Cvector in);

//! Copying a 3-element vector to a double *
void load_cvector3(double * dest, Cvector3 in);

//! Copying a double* to a 3-element vector
void save_Cvector3(Cvector3 &dest, double * in);

//! Concatenating two vectors
Cvector vectorbig(Cvector a, Cvector b);

//! Copying a double* to a 3-element vector
void Cvector3_convert(Cvector3& in, double out[3]);

//! Converting a double* to a 3-element vector
Cvector3 Cvector3_convert(double in[3]);

//! loading a double** to a matrix
void matrix3_convert(Cmatrix& in, double out[3][3]);

//! loading a matrix to double**
Cmatrix matrix3_convert(double in[3][3]);

//! Creating a 4x4 skew symmetric matrix
Cmatrix skew_symmetric_4d(Cvector I);

//! A median filter function
#define FILT_WIN 5
class median_filter
{
public:
    int n;
    Cvector x[FILT_WIN];
    int index;
    bool first_input;
    void init(int N)
    {
        index = FILT_WIN-1;
        first_input = true;
        n = N;
        for(int i=0;i<FILT_WIN;i++)
            x[i] = Cvector::Zero(n);
    }
    median_filter()
    {
        init(0);
    }
    median_filter(int N)
    {
        init(N);
    }
    Cvector update(Cvector X)
    {
        assert(X.size()==n);
        if(first_input)
        {
            for(int i=0;i<FILT_WIN;i++)
                x[i] = X;
            first_input = false;
        }
        else
        {
            index = (index+1)%FILT_WIN;
            x[index] = X;
        }
        Cvector Y = X;
        for(int i=0;i<n;i++)
        {
            Cvector window(FILT_WIN+1);
            for(int j=0;j<FILT_WIN;j++)
                window[j] = x[j][i];
            std::sort(&window[0], &window[FILT_WIN]);
            Y[i] = window[std::ceil(FILT_WIN/2)];
        }

        return Y;
    }
};

//! reading keyboard functions
int khbit();
void nonblock(int state);
bool keyState(char key);


