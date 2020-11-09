# lpvDS-lib
This package provides a C++ library for execution of Gaussian Mixture Model (GMM) based Linear Parameter Varying (LPV) Dynamical Systems; i.e. GMM-based LPV-DS which have been used and introduced in [1,2,3]. 

This version of the LPV library focuses on the formulation proposed in [3] where a non-linear DS formulated as:
<p align="center">
<img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/f_x.gif"></>
  
is learned from demonstrations in a decoupled manner. Where the GMM parameters <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/theta_gamma.gif"> used to parametrize <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/gamma.gif"> are estimated via the physically-consistent GMM approach proposed in [3] and provided in [phys-gmm](https://github.com/nbfigueroa/phys-gmm) and the DS parameters <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/DS_params.gif"> are estimated by solving a semi-definite optimization problem that ensures global asymptotic stability of the system via constraints derived from either a:
- QLF (Quadratic Lyapunov Function): <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/stab_qlf.gif">
- P-QLF(Parametrized QLF):  <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/stab_pqlf.gif">  

The whole learning scheme is provided in [ds-opt](https://github.com/nbfigueroa/ds-opt). This formulation and learning approach enables the accurate encoding of highly non-linear, non-monotic trajectories, as the ones below:

<p align="center">
<img src="https://github.com/nbfigueroa/lpvDS-lib/blob/agnostic/img/coManip-ds-0.jpg"  width="400"><img src="https://github.com/nbfigueroa/lpvDS-lib/blob/agnostic/img/coManip-ds-1.jpg"  width="400"></>

while ensuring global asymptotic stability. To learn DS with this formulation and with any of the two Lyapunov constraints defined above go to the [ds-opt](https://github.com/nbfigueroa/ds-opt) package. 

**Note**: This package supports inputs from [eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 

### Installation (Via Cmake)
Before compiling and installing the library make sure that you have the [eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) library installed and that cmake can find the headers.

```bash
$ cd ~/lpvDS-lib
$ mkdir build && cd build
$ cmake .. && make
$ sudo make install
```
This will create one executable ``./test_lpvDS`` to test the class which will placed in ``~/lpvDS-lib/test``.

If the test works [see below for the expected output], it's safe to say that you can use this library in your own project, to include it, add the following lines to your CMakeLists.txt (assuming ```test-lpvDS.cpp``` is where you want to use the library)
```bash
find_package (Eigen3 REQUIRED) 
include_directories(include ${LPVDS_INCLUDE_DIRS})

add_executable(test-lpv src/test-lpvDS.cpp)
target_link_libraries(test-lpv lpvDS)
```
---

### Usage
First and foremost, one must have a GMM-based LPV-DS model, which requires the following parameters:
- GMM parameters: ``Priors``, ``Mu``,``Sigma``
- DS parameters:  ``A_k``,``b_k``

The ``lpvDS`` class can read these parameters in different formats:
- Text files: A text file of each parameter is needed.; i.e. ``Priors.txt``,``Mu.txt``,``Sigma.txt``, ``A_k.txt``, ``b_k.txt``,``attractor.txt``, ``dimensions.txt``, where the last file should contain 2 numbers ``[K M]`` indicating the number of Gaussian components ``K`` and the dimensionality of the states ``M``.

Examples of these files are provided in the ``models/`` folder. To generate these files follow the ``demo_learn_lpvDS.m`` script in the [ds-opt](https://github.com/nbfigueroa/ds-opt) package. Once you have your lpv-DS model, you can either initialize an instance of the lpv-DS class as follows:

- For text files, you have multiple initialization options:
```C++
  /* Include Library Headers */
  #include "lpvDS.h"
  #include "utils.h"
  
  /* Instantiate an LPV-DS class Option 1 */
  lpvDS lpvDS_(path_dim.c_str());
  lpvDS_.initialize_gamma(path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str());
  lpvDS_.initialize_A(path_A.c_str());
  
  /* Instantiate an LPV-DS class Option 2 */
  lpvDS lpvDS_ (path_dim.c_str(), path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str(), path_A.c_str());
```
Or you can read the parameter files using the ``fileUtils`` class and initialize an lpvDS class instance as follows:
```C++
   /* Include Library Headers */
   #include "lpvDS.h"
   #include "utils.h"
  
   /* Instantiate an LPV-DS class Option 3 */
   fileUtils fileUtils_;
   MatrixXd dim, Priors, Mu, Sigma, A;
   dim     = fileUtils_.readMatrix(path_dim.c_str());
   Priors  = fileUtils_.readMatrix(path_Priors.c_str());
   Mu      = fileUtils_.readMatrix(path_Mu.c_str());
   Sigma   = fileUtils_.readMatrix(path_Sigma.c_str());
   A       = fileUtils_.readMatrix(path_A.c_str());
   int K = (int)dim(0,0);
   int M = (int)dim(1,0);
   lpvDS lpvDS_ (K, M, Priors, Mu, Sigma, A);
```
Where ``K`` is the number of the Gaussian components and ``M`` is the dimension of the system.

#### In the loop:
Once you have the lpvDS class instantiated and initialized in any of the available formats, you can use it in the loop as follows:
- For ``Eigen::VectorXd`` type inputs:

```C++
VectorXd att = ...;                        /* attractor */
VectorXd xi  = ...;                        /* current state */
VextorXd xi_dot;                           /* desired velocity */

 /* Option 1: Computing desired velocity manually*/
MatrixXd  A_matrix = lpv_DS_.compute_A(xi) /* computing the weight sum of A matrices */
xi_dot = A_matrix*(xi - att);              /* computing the desired velocity */

/* Option 2: Computing desired velocity directly*/
xi_dot = lpvDS_.compute_f(xi_ref_test, att);
```
This is a minimalistic code, proper resizing of vectors and matrices must be implemented.

---
### Testing the class with txt files
You can try the testing script as well, this was generated by compiling the library,  type the following:
```bash
$ cd ~/lpvDS-lib/test
$ ./test_lpvDS
```
To run it correctly, you must modify the ```path_model``` variable:
```C++
string path_model  = "/home/nbfigueroa/proj/catkin_ws_icub/src/lpvDS/models/coManip-DS-0/";
```
This test will load a stored lpv-DS model (the one that generates the C-Shape above) and the demonstrations that were used to train it. It instantiates an lpv-DS model with the three different ways shown above for text files and checks for numerical errors in the estimated velocities, you should see the following:
`
```bash
Initialization Test 1: 
[File Dimensionality] row 2 1
Initialized an M:2 dimensional GMM-based LPV-DS with K: 6 Components
[File Dimensionality] row 1 6
** Initializing Priors **
[File Dimensionality] row 2 6
** Initializing Mu **
[File Dimensionality] row 12 2
** Initializing Sigma **
[File Dimensionality] row 12 2
** Initializing A's' **
Initialization Test 2: 
[File Dimensionality] row 2 1
Initialized an M:2 dimensional GMM-based LPV-DS with K: 6 Components
[File Dimensionality] row 1 6
** Initializing Priors **
[File Dimensionality] row 2 6
** Initializing Mu **
[File Dimensionality] row 12 2
** Initializing Sigma **
[File Dimensionality] row 12 2
** Initializing A's' **
Initialization Test 3: 
[File Dimensionality] row 2 1
[File Dimensionality] row 1 6
[File Dimensionality] row 2 6
[File Dimensionality] row 12 2
[File Dimensionality] row 12 2
Initialized an M:2 dimensional GMM-based LPV-DS with K: 6 Components
** Initializing Priors **
** Initializing Mu **
** Initializing Sigma **
** Initializing A's' **
Testing Accuracy of model...
[File Dimensionality] row 2 1
[File Dimensionality] row 4 546
[File Dimensionality] row 2 546
Average Estimation Error (Norm of predicted Matlab and C++ velocities): 3.3914e-06
```



**References**     
> [1] Mirrazavi Salehian, S. S., Khoramshahi, M. and Billard, A. (2016) A Dynamical System Approach for Catching Softly a Flying Object: Theory and Experiment. in IEEE Transactions on Robotics, vol. 32, no. 2, pp. 462-471, April 2016.  
> [2] Mirrazavi Salehian, S. S. (2018) Compliant control of Uni/ Multi- robotic arms with dynamical systems. PhD Thesis  
> [3] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL). Zurich, Swizterland.

This package was initially implemented by [Sina Mirrazavi](http://lasa.epfl.ch/people/member.php?SCIPER=233855) and has been extended and modified by [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387).  

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the EU project [Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon) H2020-ICT-23-2014.
