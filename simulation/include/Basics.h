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
#include <Quaternions.h>
//#include "utils.h"
using namespace std;

// #define inf 1e20

//! appending two vectors
VectorXd vectorbig(VectorXd a, VectorXd b);

//! truncating values
double truncate_command(double in, double up, double down);
VectorXd truncate_command(VectorXd in, double up, double down);

//! reading keyboard functions
int khbit();
void nonblock(int state);
bool keyState(char key);

// Eigen redefinition
#define zero_v6 VectorXd::Zero(6)
#define zero_v3 Vector3d(0.0,0.0,0.0)
#define zero_quat Vector4d(0.0,0.0,0.0,1.0)
#define zero_v7 vectorbig(zero_v3, zero_quat)
